find_package(rosbag2_cpp REQUIRED)

add_executable(run_mapping_2d BagIO.cc main.cc)
target_link_libraries(run_mapping_2d fos)
ament_target_dependencies(run_mapping_2d rosbag2_cpp)

