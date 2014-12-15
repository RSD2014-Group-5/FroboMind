cmake_minimum_required(VERSION 2.8.3)
project(redline_following)

##  ROS
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport dynamic_reconfigure msgs pcl_ros roscpp sensor_msgs tf visualization_msgs message_generation std_msgs)

##	Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

##	OpenCV
find_package(OpenCV REQUIRED)

##  PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

##  Messages
add_message_files(FILES Line.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES row_extractor
#  CATKIN_DEPENDS dynamic_reconfigure laser_geometry msgs roscpp sensor_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(redline_following_node src/RedlineFollowingNode.cpp)

add_dependencies(redline_following_node msgs_gencpp)

target_link_libraries(redline_following_node
  ${catkin_LIBRARIES}
)
