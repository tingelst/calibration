cmake_minimum_required(VERSION 2.8)
project(laser_calibration)
find_package( OpenCV REQUIRED )
add_executable(laser_calibration camera_capture.cpp lasercalibrator.cpp calibrator.cpp main.cpp)
target_link_libraries(laser_calibration ${OpenCV_LIBS} )

