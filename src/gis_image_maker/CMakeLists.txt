cmake_minimum_required(VERSION 3.0.2)
project(gis_image_maker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  cv_bridge
)

catkin_package()
find_package(OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(gis_image_maker src/pointcloud_processor_image.cpp)

target_link_libraries(gis_image_maker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
