cmake_minimum_required(VERSION 2.8.3)
project(data_bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosbag
  nav_msgs
  sensor_msgs
  turtlesim
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)

## 用于坐标系转换
# find_package(GeographicLib REQUIRED)
# include_directories(${GeographicLib_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_pub
   CATKIN_DEPENDS geometry_msgs roscpp rospy rosbag nav_msgs sensor_msgs turtlesim message_runtime pcl_ros pcl_conversions
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
# include
	${PCL_LIBRARY_DIRS}
)

# add_executable(ins_txt2bag src/ins_txt2bag.cpp)
# target_link_libraries(ins_txt2bag ${catkin_LIBRARIES})

# add_executable(bag2txt__nav_msgs_Odometry 
#                src/bag2txt__nav_msgs_Odometry.cpp)
# target_link_libraries(bag2txt__nav_msgs_Odometry 
#                       ${catkin_LIBRARIES}
#                       ${GeographicLib_LIBRARIES})

## ins
add_executable(ins_bridge 
               src/ins/ins_bridge.cpp)
target_link_libraries(ins_bridge 
                      ${catkin_LIBRARIES}
                      ${GeographicLib_LIBRARIES})

add_executable(ins_synce
               src/ins/ins_synce.cpp)
target_link_libraries(ins_synce
                      ${catkin_LIBRARIES}
                      ${GeographicLib_LIBRARIES})

## pointCloud
add_executable(pointCloud_bridge
               src/pointCloud/pointCloud_bridge.cpp)
target_link_libraries(pointCloud_bridge
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})
