// 离线重放 AVI 与原始采集时间戳；只记录候选关联，不打开相机或串口。

#include "rotating_candidate_policy.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/yolo.hpp"

namespace {

struct Options {
  std::string config;
  std::string video;
  std::string timing_csv;
  std::string output;
};

struct TimingRow {
  double capture_s = 0;
  int original_candidates = 0;
  int width = 0;
  int height = 0;
  std::string phase;
  std::string source;
};

struct InputEvidence {
  std::filesystem::path video;
  std::filesystem::path timing_csv;
  std::filesystem::path config;
  std::filesystem::path config_snapshot;
  std::filesystem::path session;
  std::filesystem::path manifest;
  std::filesystem::path model_xml;
  std::filesystem::path model_bin;
  std::filesystem::path classifier_model;
  bool manifest_verified = false;
  std::map<std::string, std::string> hashes;
  std::map<std::string, std::string> session_fields;
};

void usage(const char * program) {
  std::cout << "用法：" << program
            << " --config 配置.yaml --video 视频.avi"
               " --timing-csv 原采集.csv --output 关联.csv\n"
               "只读离线重放；不支持串口、相机或云台控制参数。\n";
}

Options parse_args(int argc, char ** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) throw std::runtime_error("缺少参数值：" + arg);
    const std::string value = argv[++i];
    if (arg == "--config") options.config = value;
    else if (arg == "--video") options.video = value;
    else if (arg == "--timing-csv") options.timing_csv = value;
    else if (arg == "--output") options.output = value;
    else throw std::runtime_error("未知参数：" + arg);
  }
  if (options.config.empty() || options.video.empty() || options.timing_csv.empty() ||
      options.output.empty()) {
    throw std::runtime_error("须同时指定配置、视频、时间戳 CSV 和输出 CSV");
  }
  return options;
}

std::filesystem::path local_regular_file(const std::string & requested,
                                         const char * purpose) {
  namespace fs = std::filesystem;
  // VideoCapture(string) can interpret URLs or capture backends. Only pass it a
  // canonical absolute path after checking that the caller named a regular file.
  if (requested.find(':') != std::string::npos) {
    throw std::runtime_error(std::string(purpose) + " 不接受 URL 或设备协议");
  }
  std::error_code error;
  const auto status = fs::symlink_status(requested, error);
  if (error || status.type() != fs::file_type::regular) {
    throw std::runtime_error(std::string(purpose) + " 必须是本地普通文件，不能是设备或链接");
  }
  const auto canonical = fs::canonical(requested);
  const auto full = canonical.string();
  for (const char * prefix : {"/dev/", "/proc/", "/sys/", "/run/"}) {
    if (full.rfind(prefix, 0) == 0) {
      throw std::runtime_error(std::string(purpose) + " 不能位于设备或内核虚拟文件系统");
    }
  }
  return canonical;
}

void check_avi_signature(const std::filesystem::path & path) {
  std::ifstream input(path, std::ios::binary);
  std::array<char, 12> header{};
  if (!input.read(header.data(), header.size()) ||
      std::string(header.data(), 4) != "RIFF" ||
      std::string(header.data() + 8, 4) != "AVI ") {
    throw std::runtime_error("AVI 头不是 RIFF/AVI");
  }
}

std::string sha256_file(const std::filesystem::path & path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("无法读取待校验文件：" + path.string());
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (!context) throw std::runtime_error("SHA-256 初始化失败");
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("SHA-256 初始化失败");
  }
  std::array<char, 1 << 16> buffer{};
  while (input) {
    input.read(buffer.data(), buffer.size());
    const auto count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(context, buffer.data(),
                                      static_cast<std::size_t>(count)) != 1) {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("SHA-256 计算失败");
    }
  }
  if (!input.eof()) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("读取待校验文件失败：" + path.string());
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context, digest, &length) != 1 || length != 32) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("SHA-256 收尾失败");
  }
  EVP_MD_CTX_free(context);
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) {
    result << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return result.str();
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r");
  if (first == std::string::npos) return "";
  const auto last = value.find_last_not_of(" \t\r");
  return value.substr(first, last - first + 1);
}

std::map<std::string, std::string> read_session_fields(const std::filesystem::path & path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("无法读取会话标记：" + path.string());
  std::map<std::string, std::string> fields;
  std::string line;
  while (std::getline(input, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) throw std::runtime_error("会话标记格式错误");
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key.empty() || !fields.emplace(key, value).second) {
      throw std::runtime_error("会话标记字段为空或重复");
    }
  }
  if (!input.eof()) throw std::runtime_error("会话标记读取失败");
  return fields;
}

const std::string & session_value(const InputEvidence & evidence, const char * key) {
  const auto it = evidence.session_fields.find(key);
  if (it == evidence.session_fields.end()) {
    throw std::runtime_error(std::string("会话标记缺少字段：") + key);
  }
  return it->second;
}

void verify_manifest(InputEvidence & evidence) {
  namespace fs = std::filesystem;
  const auto manifest = evidence.video.parent_path() / "SHA256SUMS";
  std::error_code error;
  const auto status = fs::symlink_status(manifest, error);
  if (status.type() == fs::file_type::not_found &&
      (!error || error == std::errc::no_such_file_or_directory)) return;
  evidence.manifest = local_regular_file(manifest.string(), "SHA256SUMS");
  std::ifstream input(evidence.manifest);
  if (!input) throw std::runtime_error("无法读取 SHA256SUMS");

  std::map<std::string, std::string> expected;
  std::string line;
  while (std::getline(input, line)) {
    if (line.size() < 67 || line[64] != ' ' ||
        (line[65] != ' ' && line[65] != '*')) {
      throw std::runtime_error("SHA256SUMS 格式错误");
    }
    const std::string digest = line.substr(0, 64);
    if (!std::all_of(digest.begin(), digest.end(), [](unsigned char c) {
          return std::isxdigit(c) != 0;
        })) {
      throw std::runtime_error("SHA256SUMS 含非法 SHA-256 值");
    }
    const std::string name = line.substr(66);
    if (!expected.emplace(name, digest).second) {
      throw std::runtime_error("SHA256SUMS 含重复文件名：" + name);
    }
  }
  if (!input.eof()) throw std::runtime_error("SHA256SUMS 读取失败");
  const std::array<fs::path, 4> required = {
    evidence.video, evidence.timing_csv, evidence.config_snapshot, evidence.session};
  for (const auto & path : required) {
    const auto key = "./" + path.filename().string();
    const auto it = expected.find(key);
    if (it == expected.end() || evidence.hashes.at(path.filename().string()) != it->second) {
      throw std::runtime_error("SHA256SUMS 缺少文件或摘要不匹配：" + key);
    }
  }
  evidence.manifest_verified = true;
  evidence.hashes["SHA256SUMS"] = sha256_file(evidence.manifest);
}

InputEvidence verify_input_files(const Options & options) {
  namespace fs = std::filesystem;
  InputEvidence evidence;
  evidence.video = local_regular_file(options.video, "AVI");
  evidence.timing_csv = local_regular_file(options.timing_csv, "原始 CSV");
  evidence.config = local_regular_file(options.config, "配置");
  if (evidence.video.extension() != ".avi" || evidence.timing_csv.extension() != ".csv" ||
      evidence.config.extension() != ".yaml") {
    throw std::runtime_error("输入后缀须分别是 .avi、.csv、.yaml");
  }
  if (evidence.video.parent_path() != evidence.timing_csv.parent_path() ||
      evidence.video.stem() != evidence.timing_csv.stem()) {
    throw std::runtime_error("AVI 与原始 CSV 须在同一目录且同名");
  }
  check_avi_signature(evidence.video);
  const auto prefix = evidence.video.parent_path() / evidence.video.stem();
  evidence.config_snapshot = local_regular_file(prefix.string() + ".config.yaml", "配置快照");
  evidence.session = local_regular_file(prefix.string() + ".session.txt", "会话标记");
  evidence.session_fields = read_session_fields(evidence.session);
  if (session_value(evidence, "status") != "complete_frame_count_verified" ||
      session_value(evidence, "schema") != "1" ||
      session_value(evidence, "config_snapshot") !=
        evidence.config_snapshot.filename().string()) {
    throw std::runtime_error("会话状态、格式或配置快照名不匹配");
  }
  for (const auto & path : {evidence.video, evidence.timing_csv,
                           evidence.config_snapshot, evidence.session}) {
    evidence.hashes[path.filename().string()] = sha256_file(path);
  }
  evidence.hashes["requested_config"] = sha256_file(evidence.config);
  if (evidence.hashes.at("requested_config") !=
      evidence.hashes.at(evidence.config_snapshot.filename().string())) {
    throw std::runtime_error("--config 与现场配置快照的内容不同");
  }
  const auto yaml = YAML::LoadFile(evidence.config.string());
  evidence.model_xml = local_regular_file(yaml["yolov5_model_path"].as<std::string>(),
                                          "YOLO 模型 XML");
  auto model_bin = evidence.model_xml;
  model_bin.replace_extension(".bin");
  evidence.model_bin = local_regular_file(model_bin.string(), "YOLO 模型 BIN");
  evidence.classifier_model = local_regular_file(
    yaml["classify_model"].as<std::string>(), "分类模型");
  evidence.hashes["model_xml"] = sha256_file(evidence.model_xml);
  evidence.hashes["model_bin"] = sha256_file(evidence.model_bin);
  evidence.hashes["classifier_model"] = sha256_file(evidence.classifier_model);
  verify_manifest(evidence);
  return evidence;
}

std::vector<std::string> split_csv_row(const std::string & line) {
  // 原探针生成的 timing CSV 不含引号字段；此处只读取帧号、取帧时刻及候选数。
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const auto comma = line.find(',', start);
    fields.push_back(trim(line.substr(start, comma - start)));
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return fields;
}

int parse_int(const std::string & value) {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("CSV 整数字段不合法");
  return result;
}

double parse_double(const std::string & value) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::runtime_error("CSV 浮点字段不合法");
  }
  return result;
}

std::vector<TimingRow> read_timing(const std::string & path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("无法读取原始 CSV：" + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("原始 CSV 为空");
  const auto header = split_csv_row(line);
  auto column = [&](const std::string & name) {
    const auto it = std::find(header.begin(), header.end(), name);
    if (it == header.end()) throw std::runtime_error("原始 CSV 缺少列：" + name);
    return static_cast<std::size_t>(it - header.begin());
  };
  const auto frame_column = column("frame");
  const auto capture_column = column("capture_s");
  const auto count_column = column("blue_candidates");
  const auto width_column = column("width");
  const auto height_column = column("height");
  const auto phase_column = column("phase");
  const auto source_column = column("source");
  const auto max_column = std::max({frame_column, capture_column, count_column,
                                    width_column, height_column, phase_column, source_column});

  std::vector<TimingRow> rows;
  while (std::getline(input, line)) {
    const auto fields = split_csv_row(line);
    if (fields.size() <= max_column) throw std::runtime_error("原始 CSV 行缺少字段");
    const int frame = parse_int(fields[frame_column]);
    const double capture_s = parse_double(fields[capture_column]);
    const int count = parse_int(fields[count_column]);
    const int width = parse_int(fields[width_column]);
    const int height = parse_int(fields[height_column]);
    if (frame != static_cast<int>(rows.size()) || capture_s < 0 || count < 0 ||
        width <= 0 || height <= 0 ||
        (!rows.empty() && capture_s <= rows.back().capture_s)) {
      throw std::runtime_error("原始 CSV 帧号、尺寸、取帧时刻或候选数异常");
    }
    rows.push_back({capture_s, count, width, height,
                    fields[phase_column], fields[source_column]});
  }
  if (!input.eof() || rows.empty()) throw std::runtime_error("原始 CSV 不完整");
  return rows;
}

void verify_timing_session(const std::vector<TimingRow> & timings,
                           const InputEvidence & evidence) {
  if (parse_int(session_value(evidence, "saved_frames")) !=
        static_cast<int>(timings.size()) ||
      parse_int(session_value(evidence, "requested_frames")) <
        static_cast<int>(timings.size()) ||
      session_value(evidence, "phase") != "rotating" ||
      (session_value(evidence, "source") != "camera" &&
       session_value(evidence, "source") != "video")) {
    throw std::runtime_error("会话帧数、阶段或来源与旋转靶重放要求不符");
  }
  for (std::size_t i = 0; i < timings.size(); ++i) {
    if (timings[i].phase != session_value(evidence, "phase") ||
        timings[i].source != session_value(evidence, "source")) {
      throw std::runtime_error("原始 CSV 阶段/来源与会话标记不同：frame=" +
                               std::to_string(i));
    }
  }
}

std::vector<rotating_probe::Candidate> valid_blue_candidates(
    const std::list<auto_aim::Armor> & armors, const cv::Size & size,
    double min_confidence) {
  std::vector<rotating_probe::Candidate> candidates;
  for (const auto & armor : armors) {
    if (armor.color != auto_aim::Color::blue || armor.confidence < min_confidence ||
        armor.points.size() != 4 || !std::isfinite(armor.center.x) ||
        !std::isfinite(armor.center.y) || armor.center.x < 0 || armor.center.y < 0 ||
        armor.center.x >= size.width || armor.center.y >= size.height) continue;
    candidates.push_back({static_cast<int>(armor.color), static_cast<int>(armor.name),
                          static_cast<int>(armor.type), armor.center.x, armor.center.y,
                          armor.confidence});
  }
  return candidates;
}

void write_row(std::ostream & output, int frame, const TimingRow & timing,
               const std::vector<rotating_probe::Candidate> & candidates,
               int candidate_index, bool count_mismatch,
               const rotating_probe::Decision & decision) {
  output << frame << ',' << timing.capture_s << ',' << timing.original_candidates << ','
         << candidates.size() << ',' << static_cast<int>(count_mismatch) << ','
         << candidate_index << ',';
  if (candidate_index >= 0) {
    const auto & c = candidates.at(static_cast<std::size_t>(candidate_index));
    output << auto_aim::COLORS.at(c.color) << ',' << auto_aim::ARMOR_NAMES.at(c.name) << ','
           << auto_aim::ARMOR_TYPES.at(c.type) << ',' << c.x << ',' << c.y << ','
           << c.confidence;
  } else {
    output << ",,,,,";
  }
  output << ',' << static_cast<int>(decision.locked_before) << ','
         << rotating_probe::event_name(decision.event) << ','
         << static_cast<int>(decision.locked_after) << ',' << decision.epoch << ','
         << decision.acquisition_streak << ',' << decision.match_count << ','
         << decision.selected_index << ',';
  if (decision.selected_index >= 0) {
    const auto & selected = candidates.at(static_cast<std::size_t>(decision.selected_index));
    output << selected.x << ',' << selected.y << ',' << decision.step_px;
  } else {
    output << ",,";
  }
  output << '\n';
}

struct CountMismatch {
  int frame;
  int original;
  int replay;
};

int max_count_mismatches(std::size_t frame_count) {
  // AVI 是 MJPEG，再编码和推理可让阈值边缘的一帧发生变化；容差为每 400 帧
  // 最多 1 帧（900 帧最多 3 帧），且仍须处在原记录的候选数切换边界。
  return static_cast<int>((frame_count + 399) / 400);
}

bool expected_transition_boundary(const std::vector<TimingRow> & timings,
                                  int frame, int replay_count) {
  return (frame > 0 && timings[frame - 1].original_candidates == replay_count) ||
         (frame + 1 < static_cast<int>(timings.size()) &&
          timings[frame + 1].original_candidates == replay_count);
}

void write_evidence(std::ostream & output, const InputEvidence & evidence,
                    const std::filesystem::path & result_path,
                    const std::string & result_sha256, int frames,
                    int max_mismatches, const std::vector<CountMismatch> & mismatches) {
  output << "input_video=" << evidence.video.string() << '\n'
         << "input_video_sha256=" << evidence.hashes.at(evidence.video.filename().string()) << '\n'
         << "input_csv=" << evidence.timing_csv.string() << '\n'
         << "input_csv_sha256=" << evidence.hashes.at(evidence.timing_csv.filename().string()) << '\n'
         << "input_config=" << evidence.config.string() << '\n'
         << "input_config_sha256=" << evidence.hashes.at("requested_config") << '\n'
         << "config_snapshot=" << evidence.config_snapshot.string() << '\n'
         << "config_snapshot_sha256="
         << evidence.hashes.at(evidence.config_snapshot.filename().string()) << '\n'
         << "session=" << evidence.session.string() << '\n'
         << "session_sha256=" << evidence.hashes.at(evidence.session.filename().string()) << '\n'
         << "manifest=" << (evidence.manifest_verified ? evidence.manifest.string() : "absent")
         << '\n'
         << "manifest_sha256="
         << (evidence.manifest_verified ? evidence.hashes.at("SHA256SUMS") : "absent") << '\n'
         << "manifest_verified=" << (evidence.manifest_verified ? "true" : "false") << '\n'
         << "model_xml=" << evidence.model_xml.string() << '\n'
         << "model_xml_sha256=" << evidence.hashes.at("model_xml") << '\n'
         << "model_bin=" << evidence.model_bin.string() << '\n'
         << "model_bin_sha256=" << evidence.hashes.at("model_bin") << '\n'
         << "classifier_model=" << evidence.classifier_model.string() << '\n'
         << "classifier_model_sha256=" << evidence.hashes.at("classifier_model") << '\n';
  for (const auto & [key, value] : evidence.session_fields) {
    output << "session_" << key << '=' << value << '\n';
  }
  output << "decoded_frames=" << frames << '\n'
         << "count_mismatch_limit=" << max_mismatches << '\n'
         << "count_mismatch_frames=" << mismatches.size() << '\n';
  for (const auto & mismatch : mismatches) {
    output << "count_mismatch_frame=" << mismatch.frame << ",original="
           << mismatch.original << ",replay=" << mismatch.replay << '\n';
  }
  output << "output_csv=" << result_path.string() << '\n'
         << "output_csv_sha256=" << result_sha256 << '\n'
         << "note=SHA256SUMS is an unsigned checksum list; detector model assets are not in it.\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::filesystem::path final_output;
  std::filesystem::path incomplete_output;
  std::filesystem::path incomplete_evidence;
  std::filesystem::path evidence_path;
  bool created_incomplete_output = false;
  bool created_incomplete_evidence = false;
  bool created_final_output = false;
  try {
    const auto options = parse_args(argc, argv);
    final_output = options.output;
    evidence_path = options.output + ".evidence.txt";
    if (std::filesystem::exists(options.output) || std::filesystem::exists(evidence_path)) {
      throw std::runtime_error("输出文件已存在，请换新名字：" + options.output);
    }
    incomplete_output = options.output + ".partial";
    incomplete_evidence = evidence_path.string() + ".partial";
    if (std::filesystem::exists(incomplete_output) ||
        std::filesystem::exists(incomplete_evidence)) {
      throw std::runtime_error("未完成输出文件已存在，请换新名字");
    }
    const auto evidence = verify_input_files(options);
    const auto timings = read_timing(evidence.timing_csv.string());
    verify_timing_session(timings, evidence);
    const int mismatch_limit = max_count_mismatches(timings.size());
    const double min_confidence =
      YAML::LoadFile(evidence.config.string())["min_confidence"].as<double>();
    if (!std::isfinite(min_confidence) || min_confidence < 0.7 || min_confidence > 1) {
      throw std::runtime_error("min_confidence 须在 0.7 到 1.0 之间");
    }
    cv::VideoCapture video(evidence.video.string(), cv::CAP_FFMPEG);
    if (!video.isOpened()) throw std::runtime_error("无法打开 AVI：" + evidence.video.string());
    auto_aim::YOLO yolo(evidence.config.string(), false);
    const auto parent = std::filesystem::path(options.output).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream output(incomplete_output);
    if (!output) throw std::runtime_error("无法创建输出：" + incomplete_output.string());
    created_incomplete_output = true;
    output << "frame,capture_s,original_blue_candidates,replay_blue_candidates,"
              "count_mismatch,"
              "candidate_index,color,name,type,x,y,confidence,locked_before,event,"
              "locked_after,lock_epoch,acquisition_streak,match_count,selected_index,"
              "selected_x,selected_y,step_px\n";
    output << std::fixed << std::setprecision(6);

    rotating_probe::CandidatePolicy policy;
    int frames = 0;
    std::vector<CountMismatch> count_mismatches;
    int selected_frames = 0;
    int selected_multi_frames = 0;
    int lost_no_match = 0;
    int lost_multiple_matches = 0;
    int lost_time_gap = 0;
    double max_tracked_step = 0;
    cv::Mat image;
    while (video.read(image)) {
      if (frames >= static_cast<int>(timings.size())) {
        throw std::runtime_error("AVI 帧数多于原始 CSV");
      }
      if (image.cols != timings[frames].width || image.rows != timings[frames].height) {
        throw std::runtime_error("AVI 与原始 CSV 的逐帧尺寸不同：frame=" +
                                 std::to_string(frames));
      }
      const auto armors = yolo.detect(image, frames);
      const auto candidates = valid_blue_candidates(armors, image.size(), min_confidence);
      const auto decision = policy.step(timings[frames].capture_s, candidates);
      if (decision.event == rotating_probe::Event::invalid_input) {
        throw std::runtime_error("策略收到非法时间或候选");
      }
      const int replay_count = static_cast<int>(candidates.size());
      const int original_count = timings[frames].original_candidates;
      const bool mismatch = replay_count != original_count;
      if (mismatch) {
        count_mismatches.push_back({frames, original_count, replay_count});
        if (std::abs(replay_count - original_count) != 1 ||
            !expected_transition_boundary(timings, frames, replay_count) ||
            static_cast<int>(count_mismatches.size()) > mismatch_limit) {
          throw std::runtime_error("重放候选数逐帧差异超过门槛或不在切换边界：frame=" +
                                   std::to_string(frames));
        }
      }
      if (decision.selected_index >= 0) {
        ++selected_frames;
        if (candidates.size() > 1) ++selected_multi_frames;
      }
      if (decision.event == rotating_probe::Event::lost_no_match) ++lost_no_match;
      if (decision.event == rotating_probe::Event::lost_multiple_matches) {
        ++lost_multiple_matches;
      }
      if (decision.event == rotating_probe::Event::lost_time_gap) ++lost_time_gap;
      if (decision.event == rotating_probe::Event::tracked) {
        max_tracked_step = std::max(max_tracked_step, decision.step_px);
      }
      if (candidates.empty()) {
        write_row(output, frames, timings[frames], candidates, -1, mismatch, decision);
      } else {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          write_row(output, frames, timings[frames], candidates, static_cast<int>(i), mismatch,
                    decision);
        }
      }
      ++frames;
    }
    if (frames != static_cast<int>(timings.size())) {
      throw std::runtime_error("AVI 可解码帧数少于原始 CSV");
    }
    output.close();
    if (!output) throw std::runtime_error("输出 CSV 收尾失败");
    const std::string result_sha256 = sha256_file(incomplete_output);
    std::ofstream evidence_output(incomplete_evidence);
    if (!evidence_output) {
      throw std::runtime_error("无法创建输入证据：" + incomplete_evidence.string());
    }
    created_incomplete_evidence = true;
    write_evidence(evidence_output, evidence, std::filesystem::absolute(options.output),
                   result_sha256, frames, mismatch_limit, count_mismatches);
    evidence_output.close();
    if (!evidence_output) throw std::runtime_error("输入证据收尾失败");
    std::filesystem::rename(incomplete_output, options.output);
    created_incomplete_output = false;
    created_final_output = true;
    std::filesystem::rename(incomplete_evidence, evidence_path);
    created_incomplete_evidence = false;
    created_final_output = false;
    if (!count_mismatches.empty()) {
      std::cout << "警告：重放候选数有 " << count_mismatches.size()
                << " 帧与原始 CSV 不同，容许上限 " << mismatch_limit << " 帧；";
      for (const auto & mismatch : count_mismatches) {
        std::cout << " frame=" << mismatch.frame << " 原=" << mismatch.original
                  << " 重放=" << mismatch.replay << ';';
      }
      std::cout << '\n';
    }
    if (!evidence.manifest_verified) {
      std::cout << "警告：会话目录没有 SHA256SUMS；已核对同名会话及配置快照，"
                   "但无法用原始清单确认文件摘要。\n";
    }
    std::cout << "只读离线重放完成：" << frames << " 帧；候选数与原始记录不同 "
              << count_mismatches.size() << " 帧；关联输出 " << selected_frames
              << " 帧，其中多候选 " << selected_multi_frames << " 帧。\n"
              << "丢锁：无匹配 " << lost_no_match << " 次、多匹配 "
              << lost_multiple_matches << " 次、时间空档 " << lost_time_gap
              << " 次；已跟踪相邻步长最大 " << std::fixed << std::setprecision(2)
              << max_tracked_step << " px。\n"
              << "门控：" << rotating_probe::CandidatePolicy::gate_px << " px，"
              << rotating_probe::CandidatePolicy::max_gap_s * 1000 << " ms，连续单候选 "
              << rotating_probe::CandidatePolicy::acquire_frames << " 帧建锁。\n"
              << "输出：" << options.output << "；输入证据：" << evidence_path << "\n"
              << "这些数字只是该视频上的关联输出率，不是真值准确率或实机验证。"
              << std::endl;
    return 0;
  } catch (const std::exception & error) {
    if (created_incomplete_output) {
      std::error_code ignored;
      std::filesystem::remove(incomplete_output, ignored);
    }
    if (created_incomplete_evidence) {
      std::error_code ignored;
      std::filesystem::remove(incomplete_evidence, ignored);
    }
    if (created_final_output) {
      std::error_code ignored;
      std::filesystem::remove(final_output, ignored);
    }
    std::cerr << "只读离线重放失败：" << error.what() << '\n';
    return 1;
  }
}
