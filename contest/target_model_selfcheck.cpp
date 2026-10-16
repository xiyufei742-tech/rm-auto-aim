#include <cmath>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

// 只在内存中构造几何观测，不加载模型、不打开设备。
// 二装甲路径以前会无条件读取不存在的第三块装甲；也检查三/四块原策略不回退。
int main()
{
  try {
    auto_aim::Armor armor(9, 0.95f, cv::Rect(0, 0, 135, 56),
                         {{0, 0}, {135, 0}, {135, 56}, {0, 56}});
    if (armor.priority != auto_aim::ArmorPriority::fifth) {
      throw std::runtime_error("YOLO armor priority must have a deterministic default");
    }
    armor.name = auto_aim::ArmorName::four;
    armor.type = auto_aim::ArmorType::small;
    armor.priority = auto_aim::ArmorPriority::first;
    armor.xyz_in_world = {5.0, 0.0, 0.0};
    armor.ypr_in_world = {0.0, 0.0, 0.0};
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    const auto now = std::chrono::steady_clock::now();
    const Eigen::VectorXd covariance = Eigen::VectorXd::Ones(11);
    for (const int count : {1, 2, 3, 4}) {
      auto_aim::Target target(armor, now, 0.2, count, covariance);
      for (int step = 0; step < 6; ++step) {
        target.predict(0.02);
        target.update(armor);
      }
      if (!target.ekf_x().allFinite() || target.last_id != 0 ||
          target.armor_xyza_list().size() != static_cast<std::size_t>(count)) {
        throw std::runtime_error("stationary association regression for armor count " +
                                 std::to_string(count));
      }
    }
    bool rejected_zero = false;
    try {
      auto_aim::Target target(armor, now, 0.2, 0, covariance);
    } catch (const std::invalid_argument &) {
      rejected_zero = true;
    }
    if (!rejected_zero) throw std::runtime_error("zero armors should be rejected");

    auto_aim::Target simulated(5.0, 1.0, 0.2, 0.0);
    if (simulated.name != auto_aim::ArmorName::four || simulated.last_id != 0 ||
        simulated.jumped || simulated.convergened()) {
      throw std::runtime_error("simulation constructor state is not initialized");
    }
    simulated.predict(0.1);
    if (!simulated.ekf_x().allFinite() || std::abs(simulated.ekf_x()[6] - 0.1) > 1e-9) {
      throw std::runtime_error("simulation angular prediction regression");
    }
    std::cout << "PASS: 1/2/3/4 armor association bounds, zero-count rejection, initialized simulation\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
