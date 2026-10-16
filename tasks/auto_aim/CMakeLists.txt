# 【模块构建清单】
# 当前现场入口仍只调用 YOLO；把后续 PnP、跟踪和瞄准源码一起编译，
# 以便及时发现接口错误。编译通过不代表这些算法已经接入实机。

add_library(auto_aim STATIC
  armor.cpp classifier.cpp detector.cpp yolo.cpp yolos/yolov5.cpp
  solver.cpp tracker.cpp target.cpp aimer.cpp shooter.cpp)
target_link_libraries(auto_aim PUBLIC io tools openvino::runtime yaml-cpp ${OpenCV_LIBS})
