cmake_minimum_required(VERSION 3.0.2)
project(gis_pointcloud_rotator)  # 프로젝트 이름 지정
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
)
find_package(PCL REQUIRED) # PCL 라이브러리 찾기

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} # PCL 헤더 파일 포함
)

link_directories(${PCL_LIBRARY_DIRS}) # PCL 라이브러리 경로 추가

add_definitions(${PCL_DEFINITIONS}) # PCL 정의 추가

## Add executable for pointcloud_rotator
add_executable(gis_pointcloud_rotator src/pointcloud_rotator.cpp)
target_link_libraries(gis_pointcloud_rotator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
