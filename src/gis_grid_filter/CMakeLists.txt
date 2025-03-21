cmake_minimum_required(VERSION 3.0.2)
project(gis_grid_filter)  # 여기 추가

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_filter_node src/grid_filter_node.cpp)
target_link_libraries(grid_filter_node ${catkin_LIBRARIES})
