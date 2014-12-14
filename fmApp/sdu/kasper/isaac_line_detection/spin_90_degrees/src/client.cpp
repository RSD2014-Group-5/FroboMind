#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <spin_90_degrees/spin_degreesAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "spin_90_degrees");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<spin_90_degrees::spin_degreesAction> ac("spin_degrees", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  //learning_actionlib::FibonacciGoal goal;
  spin_90_degrees::spin_degreesGoal goal;

  //goal.order = 20;

  //goal.dishwasher_id=20;

  goal.direction="right";
 // goal.cell_name="";
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



