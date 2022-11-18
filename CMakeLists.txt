cmake_minimum_required(VERSION 3.0.2)
project(segpc_yolov7)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  yolov7_ros
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs std_msgs yolov7_ros image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(segment_pc_node src/segment_pc_node.cpp)
target_link_libraries(segment_pc_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})