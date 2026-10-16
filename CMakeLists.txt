# 【学习导读｜精简版构建入口】
# CMake描述源文件怎样组成库/程序；cmake配置生成构建规则，随后build才进行编译链接。
# 源代码注释不会改变算法；.cpp改动后仍需重新构建，生成可执行文件才会更新。
# Solver/Tracker/Target/Aimer/Shooter 编入算法库供检查；
# 现场联调程序使用 YOLO 与受限像素 yaw/pitch；CBoard 类型仅保留为后续算法接口参考。

cmake_minimum_required(VERSION 3.16)
project(self_aim_contest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找本机已经安装的依赖；REQUIRED表示找不到就停止配置。
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
if(NOT OpenVINO_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvino_2024.6.0/runtime/cmake")
  set(OpenVINO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvino_2024.6.0/runtime/cmake")
endif()
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

# 库集中保存可复用代码；这里多个程序共享比赛协议打包/解包实现。
add_library(srm_protocol contest/protocol.cpp)
target_include_directories(srm_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/contest)

add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test PRIVATE io tools ${OpenCV_LIBS} fmt::fmt)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
target_link_libraries(camera_detect_test PRIVATE auto_aim io tools ${OpenCV_LIBS} fmt::fmt)

# 每个可执行目标各有自己的main。probe只读；step单步；follow短时连续修正。
# 文件夹src并不决定哪个才是主程序，真正入口由这些add_executable指定。
add_executable(srm_live_pixel_probe contest/live_pixel_probe.cpp)
add_executable(srm_live_pixel_step_safe contest/live_pixel_step_safe.cpp)
add_executable(srm_live_pixel_follow_safe contest/live_pixel_follow_safe.cpp)
add_executable(srm_live_pixel_pitch_follow_safe contest/live_pixel_pitch_follow_safe.cpp)
add_executable(srm_live_pixel_xy_follow_safe contest/live_pixel_xy_follow_safe.cpp)
foreach(target srm_live_pixel_probe srm_live_pixel_step_safe srm_live_pixel_follow_safe
               srm_live_pixel_pitch_follow_safe srm_live_pixel_xy_follow_safe)
  target_link_libraries(${target} PRIVATE auto_aim io tools srm_protocol ${OpenCV_LIBS} fmt::fmt)
  target_compile_options(${target} PRIVATE -Wall -Wextra)
endforeach()

# 旋转靶候选关联只支持 AVI + 原采集时间戳离线重放；不链接比赛串口协议。
add_executable(srm_rotating_candidate_replay contest/rotating_candidate_replay.cpp)
target_link_libraries(srm_rotating_candidate_replay PRIVATE auto_aim ${OpenCV_LIBS} fmt::fmt OpenSSL::Crypto)
target_compile_options(srm_rotating_candidate_replay PRIVATE -Wall -Wextra)

add_executable(srm_protocol_demo contest/protocol_demo.cpp)
target_link_libraries(srm_protocol_demo PRIVATE srm_protocol)
# 以下自检验证协议、策略与虚拟串口行为；不会打开真实相机或驱动现场云台。
# ctest通过说明这些覆盖的逻辑符合断言，不等于真实目标居中或命中验证通过。
enable_testing()
add_executable(srm_protocol_selfcheck contest/protocol_selfcheck.cpp)
target_link_libraries(srm_protocol_selfcheck PRIVATE srm_protocol)
add_test(NAME srm_protocol_selfcheck COMMAND srm_protocol_selfcheck)

add_executable(srm_pixel_step_policy_selfcheck contest/pixel_step_policy_selfcheck.cpp)
target_link_libraries(srm_pixel_step_policy_selfcheck PRIVATE srm_protocol)
add_test(NAME srm_pixel_step_policy_selfcheck COMMAND srm_pixel_step_policy_selfcheck)
add_executable(srm_pixel_follow_policy_selfcheck contest/pixel_follow_policy_selfcheck.cpp)
add_test(NAME srm_pixel_follow_policy_selfcheck COMMAND srm_pixel_follow_policy_selfcheck)
add_executable(srm_pitch_follow_policy_selfcheck contest/pitch_follow_policy_selfcheck.cpp)
add_test(NAME srm_pitch_follow_policy_selfcheck COMMAND srm_pitch_follow_policy_selfcheck)
add_executable(srm_xy_follow_policy_selfcheck contest/xy_follow_policy_selfcheck.cpp)
add_test(NAME srm_xy_follow_policy_selfcheck COMMAND srm_xy_follow_policy_selfcheck)
add_executable(srm_rotating_candidate_policy_selfcheck
  contest/rotating_candidate_policy_selfcheck.cpp)
target_compile_options(srm_rotating_candidate_policy_selfcheck PRIVATE -Wall -Wextra)
add_test(NAME srm_rotating_candidate_policy_selfcheck
  COMMAND srm_rotating_candidate_policy_selfcheck)

add_executable(srm_command_worker_selfcheck contest/command_worker_selfcheck.cpp)
target_link_libraries(srm_command_worker_selfcheck PRIVATE auto_aim io tools srm_protocol ${OpenCV_LIBS} fmt::fmt util)
add_test(NAME srm_command_worker_selfcheck COMMAND srm_command_worker_selfcheck)
add_executable(srm_follow_worker_selfcheck contest/follow_worker_selfcheck.cpp)
target_link_libraries(srm_follow_worker_selfcheck PRIVATE auto_aim io tools srm_protocol ${OpenCV_LIBS} fmt::fmt util)
add_test(NAME srm_follow_worker_selfcheck COMMAND srm_follow_worker_selfcheck)
add_executable(srm_pitch_follow_worker_selfcheck contest/pitch_follow_worker_selfcheck.cpp)
target_link_libraries(srm_pitch_follow_worker_selfcheck PRIVATE auto_aim io tools srm_protocol ${OpenCV_LIBS} fmt::fmt util)
add_test(NAME srm_pitch_follow_worker_selfcheck COMMAND srm_pitch_follow_worker_selfcheck)
add_executable(srm_xy_follow_worker_selfcheck contest/xy_follow_worker_selfcheck.cpp)
target_link_libraries(srm_xy_follow_worker_selfcheck PRIVATE auto_aim io tools srm_protocol ${OpenCV_LIBS} fmt::fmt util)
add_test(NAME srm_xy_follow_worker_selfcheck COMMAND srm_xy_follow_worker_selfcheck)

# 10 月 13 日的核心算法回归：Target 的边界检查必须编译到被测源文件本身。
add_executable(srm_target_model_selfcheck
  contest/target_model_selfcheck.cpp tasks/auto_aim/target.cpp tasks/auto_aim/armor.cpp)
target_compile_definitions(srm_target_model_selfcheck PRIVATE _GLIBCXX_ASSERTIONS)
target_link_libraries(srm_target_model_selfcheck PRIVATE tools ${OpenCV_LIBS} fmt::fmt yaml-cpp)
add_test(NAME srm_target_model_selfcheck COMMAND srm_target_model_selfcheck)
set_tests_properties(srm_target_model_selfcheck PROPERTIES TIMEOUT 10)

add_executable(srm_trajectory_selfcheck contest/trajectory_selfcheck.cpp tools/trajectory.cpp)
add_test(NAME srm_trajectory_selfcheck COMMAND srm_trajectory_selfcheck)
set_tests_properties(srm_trajectory_selfcheck PROPERTIES TIMEOUT 10)

add_executable(srm_aimer_selfcheck
  contest/aimer_selfcheck.cpp tasks/auto_aim/aimer.cpp
  tasks/auto_aim/target.cpp tasks/auto_aim/armor.cpp)
target_compile_definitions(srm_aimer_selfcheck PRIVATE _GLIBCXX_ASSERTIONS)
target_link_libraries(srm_aimer_selfcheck PRIVATE tools ${OpenCV_LIBS} fmt::fmt yaml-cpp)
add_test(NAME srm_aimer_selfcheck COMMAND srm_aimer_selfcheck)
set_tests_properties(srm_aimer_selfcheck PROPERTIES TIMEOUT 10)

add_executable(srm_angle_wrap_selfcheck contest/angle_wrap_selfcheck.cpp tools/math_tools.cpp)
target_link_libraries(srm_angle_wrap_selfcheck PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
add_test(NAME srm_angle_wrap_selfcheck COMMAND srm_angle_wrap_selfcheck)
set_tests_properties(srm_angle_wrap_selfcheck PROPERTIES TIMEOUT 10)
