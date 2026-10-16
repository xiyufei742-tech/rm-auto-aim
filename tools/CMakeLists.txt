# 【学习导读｜模块构建清单】
# YOLO 使用基础工具；Tracker/Aimer 还需要 EKF 与弹道工具。

add_library(tools STATIC exiter.cpp img_tools.cpp logger.cpp math_tools.cpp
  extended_kalman_filter.cpp trajectory.cpp)
target_link_libraries(tools PUBLIC ${OpenCV_LIBS} fmt::fmt spdlog::spdlog Eigen3::Eigen)
