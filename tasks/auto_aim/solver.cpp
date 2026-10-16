// 【学习导读｜后续参考：PnP 位姿解算，当前像素跟随未调用】
// 问题：YOLO 只给图上的四个点，怎样知道装甲板离相机多远、朝向哪里？
// 输入：四角像素、对应四个实物点、相机内参和畸变、相机安装外参、云台姿态。
// 输出：Armor 中的相机/云台/世界位置与朝向。位置单位 m，角度单位 rad。
// 读 solve → set_R_gimbal2world → optimize_yaw → reproject_armor。
// 本文件在精简版保留供学习，未加入当前 auto_aim 库；标定与赛题模型适配尚未完成。

#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
// 这里描述的是所选“角点之间”的真实几何尺寸，56e-3 表示 0.056 m。
// 检测点若在灯条端点，3D 点也必须对应灯条端点；不能直接换成整块外壳的长宽。
// 下面四个点位于装甲板局部坐标系 x=0 的平面，顺序须与 armor.points 一一对应。
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

// 构造时读相机内参、畸变系数、安装旋转 R 和平移 t。
// 内参 K 把相机坐标映射到像素；外参描述两个坐标系之间的位置/方向关系。
// R_camera2gimbal * p_camera + t_camera2gimbal 得到云台坐标中的同一个点。
// 保留配置中的数字不代表已适合你的这只镜头、分辨率与安装位置。
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

// 四元数 q 来自原项目姿态接口，通过固定轴系变换转成云台到世界的旋转矩阵。
// .transpose() 对正交旋转矩阵相当于求逆；乘法顺序不能调换。
// 当前 USB 回传的是欧拉角，正式接入时需确认轴序、符号、单位及时间对齐。
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）
// 本函数就地修改传入的 Armor（参数是引用），不是返回一块新装甲板。
// 先根据大小选 3D 模板，再进行平面目标位姿求解。
void Solver::solve(Armor & armor) const
{
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  // 【核心】IPPE 用于平面点的位姿求解：rvec 是旋转向量，tvec 是模板原点在相机系的位置。
  // 它不是从单个中心像素直接测距；尺寸、内参或四角错误都会影响结果。
  // 【待核查】此调用在文件两处出现；应检查 solvePnP 返回值及结果有效性再用于控制。
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  // 把轴角形式的旋转向量转为 3×3 矩阵，便于串联相机、云台与世界的轴系转换。
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 把位置 xyz 变为从原点看目标的 yaw、pitch、距离。
  // “看向目标的 yaw”与“装甲板自身朝向 yaw”不同，分别存 ypd[0] 与 ypr[0]。
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}

// 重投影是 PnP 的反方向：假定装甲板的三维位置/朝向，再算它应该出现在图像哪里。
// 可以拿预测四角与实际检测四角比较，评价这个朝向假设是否合理。
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 原开源按兵种假设装甲板倾斜 ±15°。这是目标几何先验，不是现场已经测出的参数。
  // 比赛旋转靶若安装角度不同，需要修改模型或关闭这项优化后验证。
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  // 【核心】IPPE 用于平面点的位姿求解：rvec 是旋转向量，tvec 是模板原点在相机系的位置。
  // 它不是从单个中心像素直接测距；尺寸、内参或四角错误都会影响结果。
  // 【待核查】此调用在文件两处出现；应检查 solvePnP 返回值及结果有效性再用于控制。
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  // 把轴角形式的旋转向量转为 3×3 矩阵，便于串联相机、云台与世界的轴系转换。
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 把位置 xyz 变为从原点看目标的 yaw、pitch、距离。
  // “看向目标的 yaw”与“装甲板自身朝向 yaw”不同，分别存 ypd[0] 与 ypr[0]。
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

// 固定已经解出的三维位置，在云台 yaw 周围搜索 140 个候选（间隔 1°）。
// 逐个重投影，取四角误差最小者；这段实际是离散搜索，不是梯度下降。
// 搜索得到的是装甲板朝向估计，不是直接发给电机的 yaw 目标。
void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // 保留未优化前的结果，方便比较；随后只替换世界系装甲板朝向的 yaw。
  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

// 保留的另一种重投影代价函数；当前 armor_reprojection_error 没有调用它。
// 因此汇报当前实现时，不能把这里的加权边长/方向误差说成实际正在执行的算法。
double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

// 当前实际代价是四对角点欧氏距离之和，单位 px；越小表示预测投影越接近观测。
double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
// 将世界中的点画回图像；相机后方的点被过滤，所以输出数目可能小于输入。
// 这是可视化/投影工具，不会控制云台。
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim