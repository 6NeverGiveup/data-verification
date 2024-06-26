cmake_minimum_required(VERSION 3.0)
project(data_check)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
find_package(PCL REQUIRED QUIET)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rosbag
  std_msgs
)
catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories( 
    ${PCL_LIBRARY_DIRS} 
)

add_library(rosbag_checker src/rosbag_check.cpp)
target_link_libraries(rosbag_checker ${catkin_LIBRARIES})
add_library(pcd_checker src/pcd_check.cpp)
target_link_libraries(pcd_checker ${catkin_LIBRARIES})
add_library(imu_checker src/imu_check.cpp)
target_link_libraries(imu_checker ${catkin_LIBRARIES})
add_library(gnss_checker src/gnss_check.cpp)
target_link_libraries(gnss_checker ${catkin_LIBRARIES})
add_library(yaml_checker src/yaml_check.cpp)
target_link_libraries(yaml_checker ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp stdc++ rosbag_checker pcd_checker imu_checker gnss_checker yaml_checker)
