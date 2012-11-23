cmake_minimum_required(VERSION 2.8)
project(camera_calibration)
find_package( OpenCV REQUIRED )
add_executable(camera_calibration src/camera_capture.cpp src/camera_calibrator.cpp src/calibrator.cpp src/main.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS} )

