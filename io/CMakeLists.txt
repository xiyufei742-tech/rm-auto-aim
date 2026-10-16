# 【学习导读｜模块构建清单】
# 当前io库只含Camera包装与HikRobot驱动；SDK使用amd64二进制，CBoard不在此构建。

add_library(io STATIC camera.cpp hikrobot/hikrobot.cpp)
target_include_directories(io PUBLIC hikrobot/include)
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(FATAL_ERROR "This compact copy includes only the x86_64 Hikrobot SDK")
endif()
target_link_libraries(io PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64/libMvCameraControl.so"
  usb-1.0 yaml-cpp tools ${OpenCV_LIBS})
