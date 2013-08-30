cmake_minimum_required(VERSION 2.8)

project(laser_scanner)

#SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package( OpenCV REQUIRED )
find_package(Boost COMPONENTS python numpy REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

# add_executable( laser_scanner_cpp main.cpp scanner.cpp )
# target_link_libraries( laser_scanner_cpp
#   ${OpenCV_LIBS})
  
add_library(laser_scanner_py SHARED scanner_py.cpp scanner.cpp)
target_link_libraries( laser_scanner_py
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES})