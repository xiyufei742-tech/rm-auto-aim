#ifndef CONTEST_ROTATING_CANDIDATE_POLICY_HPP
#define CONTEST_ROTATING_CANDIDATE_POLICY_HPP

// 仅做离线候选关联；不包含串口、云台命令或开火逻辑。
// 新建锁会增加 epoch，调用方不能把不同 epoch 的像素位置当作连续目标。

#include <cmath>
#include <cstdint>
#include <vector>

namespace rotating_probe {

struct Candidate {
  int color;
  int name;
  int type;
  double x;
  double y;
  double confidence;
};

enum class Event {
  waiting_missing,
  waiting_multiple,
  acquiring,
  acquired,
  tracked,
  lost_no_match,
  lost_multiple_matches,
  lost_time_gap,
  invalid_input,
};

inline const char * event_name(Event event) {
  switch (event) {
    case Event::waiting_missing: return "waiting_missing";
    case Event::waiting_multiple: return "waiting_multiple";
    case Event::acquiring: return "acquiring";
    case Event::acquired: return "acquired";
    case Event::tracked: return "tracked";
    case Event::lost_no_match: return "lost_no_match";
    case Event::lost_multiple_matches: return "lost_multiple_matches";
    case Event::lost_time_gap: return "lost_time_gap";
    case Event::invalid_input: return "invalid_input";
  }
  return "unknown";
}

struct Decision {
  Event event = Event::waiting_missing;
  bool locked_before = false;
  bool locked_after = false;
  std::uint64_t epoch = 0;
  int acquisition_streak = 0;
  int match_count = 0;
  int selected_index = -1;
  double step_px = 0;
};

class CandidatePolicy {
public:
  static constexpr int acquire_frames = 3;
  static constexpr double gate_px = 30.0;
  static constexpr double max_gap_s = 0.060;

  Decision step(double capture_s, const std::vector<Candidate> & candidates) {
    Decision result;
    result.locked_before = locked_;
    result.epoch = epoch_;

    if (!std::isfinite(capture_s) ||
        (have_frame_time_ && capture_s <= last_frame_time_) ||
        !valid_candidates(candidates)) {
      clear_lock();
      have_frame_time_ = false;
      result.event = Event::invalid_input;
      return finish(result);
    }

    const bool frame_gap = have_frame_time_ &&
      capture_s - last_frame_time_ > max_gap_s;
    last_frame_time_ = capture_s;
    have_frame_time_ = true;

    if (frame_gap) {
      const bool was_locked = locked_;
      clear_lock();
      if (was_locked) {
        result.event = Event::lost_time_gap;
        return finish(result);  // 大时间空档的这一帧不能直接重新建锁。
      }
    }

    if (locked_) {
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto & candidate = candidates[i];
        if (!same_class(candidate, locked_candidate_)) continue;
        const double distance = distance_px(candidate, locked_candidate_);
        if (distance > gate_px) continue;
        ++result.match_count;
        result.selected_index = static_cast<int>(i);
        result.step_px = distance;
      }
      if (result.match_count == 1) {
        locked_candidate_ = candidates[result.selected_index];
        result.event = Event::tracked;
        return finish(result);
      }
      result.selected_index = -1;
      result.step_px = 0;
      clear_lock();
      result.event = result.match_count == 0 ? Event::lost_no_match :
                                         Event::lost_multiple_matches;
      return finish(result);
    }

    if (candidates.size() != 1) {
      streak_ = 0;
      result.event = candidates.empty() ? Event::waiting_missing : Event::waiting_multiple;
      return finish(result);
    }

    const auto & candidate = candidates.front();
    if (streak_ > 0 && same_class(candidate, last_acquisition_candidate_) &&
        distance_px(candidate, last_acquisition_candidate_) <= gate_px) {
      ++streak_;
    } else {
      streak_ = 1;
    }
    last_acquisition_candidate_ = candidate;
    if (streak_ >= acquire_frames) {
      locked_candidate_ = candidate;
      locked_ = true;
      streak_ = 0;
      ++epoch_;
      result.epoch = epoch_;
      result.selected_index = 0;
      result.event = Event::acquired;
    } else {
      result.event = Event::acquiring;
    }
    return finish(result);
  }

private:
  static bool same_class(const Candidate & a, const Candidate & b) {
    return a.color == b.color && a.name == b.name && a.type == b.type;
  }

  static double distance_px(const Candidate & a, const Candidate & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static bool valid_candidates(const std::vector<Candidate> & candidates) {
    for (const auto & candidate : candidates) {
      if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y) ||
          !std::isfinite(candidate.confidence)) return false;
    }
    return true;
  }

  void clear_lock() {
    locked_ = false;
    streak_ = 0;
  }

  Decision finish(Decision result) const {
    result.locked_after = locked_;
    result.acquisition_streak = streak_;
    result.epoch = epoch_;
    return result;
  }

  bool have_frame_time_ = false;
  double last_frame_time_ = 0;
  bool locked_ = false;
  Candidate locked_candidate_{};
  Candidate last_acquisition_candidate_{};
  int streak_ = 0;
  std::uint64_t epoch_ = 0;
};

}  // namespace rotating_probe

#endif  // CONTEST_ROTATING_CANDIDATE_POLICY_HPP
