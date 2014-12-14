#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <learning_actionlib/FibonacciAction.h>
#include <line_til_cross_action_backwards/GocellAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<line_til_cross_action_backwards::GocellAction> ac("Gocell_back", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  //learning_actionlib::FibonacciGoal goal;
  line_til_cross_action_backwards::GocellGoal goal;

//  goal.order = 20;

  //goal.dishwasher_id=20;


  goal.cell_name="";
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}



























//#include <line_til_cross_action/GocellAction.h>
//#include <actionlib/client/simple_action_client.h>

//typedef actionlib::SimpleActionClient<line_til_cross_action::GocellAction> Client;

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "go_til_cell_client");
//	Client client("go_til_cell", true); // true -> don't need ros::spin()
//    ROS_INFO("before wait for server");
//    client.waitForServer();
//	line_til_cross_action::GocellGoal goal;
//	// Fill in goal here
//   	goal.dishwasher_id=5;
//    ROS_INFO("before server call");
//	client.sendGoal(goal);
//    ROS_INFO("server called");

//    client.waitForResult(ros::Duration(5.0));
//	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    printf("Cross_found");
//	printf("Current State: %s\n", client.getState().toString().c_str());
//	return 0;
//}
