cmake_minimum_required(VERSION 3.5)
project(field_occupy_slam_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

include_directories(${PROJECT_SOURCE_DIR}/include /usr/include/eigen3)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(TBB REQUIRED)

file(GLOB PROJECT_SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(fos SHARED ${PROJECT_SOURCE_FILES})
target_link_libraries(fos Sophus::Sophus ${OpenCV_LIBS} yaml-cpp g2o_core g2o_stuff TBB::tbb)
ament_target_dependencies(fos rclcpp sensor_msgs)

add_subdirectory(${PROJECT_SOURCE_DIR}/example)

ament_package()
