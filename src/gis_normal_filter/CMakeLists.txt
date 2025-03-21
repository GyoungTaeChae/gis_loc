cmake_minimum_required(VERSION 3.0.2)
project(gis_normal_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  std_msgs
  )
  
catkin_package()
find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(gis_normal_filter src/normal_filter.cpp)

target_link_libraries(gis_normal_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

