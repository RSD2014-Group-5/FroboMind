cmake_minimum_required(VERSION 2.8.3)
project(line_nodes)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp cv_bridge image_transport)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
   FILES
   line_points.msg
)


add_executable(barcode_det src/bar_code_detect.cpp)
add_executable(line_det src/line_detector.cpp src/cv_functions.cpp)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   DEPENDS message_runtime std_msgs
)



#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(Eigen REQUIRED)


#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#rosbuild_add_executable(my_publisher src/my_publisher.cpp)
#rosbuild_add_executable(my_subscriber src/my_subscriber.cpp)
#rosbuild_add_executable(bar_camera src/bar_camera.cpp)
#rosbuild_add_executable(bar_code_detect src/bar_code_detect.cpp)
#rosbuild_add_executable(line_detector src/line_detector.cpp src/cv_functions.cpp)



#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(my_subscriber ${catkin_LIBRARIES} zbar)
target_link_libraries(barcode_det ${catkin_LIBRARIES} zbar)
target_link_libraries(barcode_det ${OpenCV_LIBRARIES})
target_link_libraries(line_det ${catkin_LIBRARIES})
#target_link_libraries(line_detector ${catkin_LIBRARIES} eigen3)


