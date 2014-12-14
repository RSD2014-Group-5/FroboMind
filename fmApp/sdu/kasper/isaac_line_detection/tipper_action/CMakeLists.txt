cmake_minimum_required(VERSION 2.8.3)
project(lift_tipper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib )



add_action_files( DIRECTORY action FILES tipper.action )




generate_messages(DEPENDENCIES actionlib_msgs std_msgs )


catkin_package(	CATKIN_DEPENDS actionlib_msgs  )


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(tipper_client src/client.cpp)
add_executable(tipper_service  src/server.cpp)


target_link_libraries(tipper_client ${catkin_LIBRARIES})
target_link_libraries(tipper_service ${catkin_LIBRARIES})

#add_dependencies(
#  tipper_client ${learning_actionlib_EXPORTED_TARGETS}
 # tipper_server ${learning_actionlib_EXPORTED_TARGETS}

#)


