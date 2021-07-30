cmake_minimum_required(VERSION 3.0.2)
project(kitti_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV 4.2 REQUIRED) # 找OpenCV 4.2 安装路径

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kitti_tutorials
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # 加载OpenCV 4.2 安装路径
)



add_executable(cam_node src/cam_node.cpp)
target_link_libraries(cam_node	${catkin_LIBRARIES})
target_link_libraries(cam_node	${OpenCV_LIBS})
