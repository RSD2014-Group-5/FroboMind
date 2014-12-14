cmake_minimum_required(VERSION 2.8.3)
project(line_til_cross_action_backwards)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib cv_bridge image_transport)



add_action_files( DIRECTORY action FILES Gocell.action )




generate_messages(DEPENDENCIES actionlib_msgs std_msgs )





catkin_package(	CATKIN_DEPENDS actionlib_msgs  )


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(line_client_back src/action_client.cpp)
add_executable(line_client_back_rob_x src/action_client_rob_3.cpp)

add_executable(line_server_back  src/action_server.cpp)




target_link_libraries(line_client_back ${catkin_LIBRARIES})
target_link_libraries(line_client_back_rob_x ${catkin_LIBRARIES})
target_link_libraries(line_server_back ${catkin_LIBRARIES})

add_dependencies(
  line_server_back ${learning_actionlib_EXPORTED_TARGETS}
  line_client_back ${learning_actionlib_EXPORTED_TARGETS}
  line_client_back_rob_x ${learning_actionlib_EXPORTED_TARGETS}


)


