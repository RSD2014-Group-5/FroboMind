#include <ros/ros.h>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <lift_tipper/tipperAction.h>

#include <std_msgs/Bool.h>


#define TIPPER_START "/tip_start"
#define TIPPER_DONE "/tip_done"

class tipperAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<lift_tipper::tipperAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  lift_tipper::tipperFeedback feedback_;
  lift_tipper::tipperResult result_;
  lift_tipper::tipperGoal goal_;

  ros::Publisher tipper_pub_;
  ros::Subscriber tipper_sub_;

  bool finished_;


public:

    tipperAction(std::string name) :

    // action server
    as_(nh_, name, boost::bind(&tipperAction::executeCB, this, _1), false),
    action_name_(name)
  {
        // Subscribers and publishers

    tipper_sub_ = nh_.subscribe(TIPPER_DONE, 1, &tipperAction::tipper_done_callback, this);
    tipper_pub_=nh_.advertise<std_msgs::Bool>(TIPPER_START,1);




    
    as_.start();
  }

  ~tipperAction(void)
  {

  }


    void tipper_done_callback(const std_msgs::BoolPtr& msg)
    {
        finished_=msg->data;

    }

  void executeCB(const lift_tipper::tipperGoalConstPtr &goal)
  {
        // helper variables
        finished_=false;


        //publish the lift message
        std_msgs::Bool start;

        start.data=true;

        tipper_pub_.publish(start);

        int counter=0;

        ros::Rate r(10);
        while (ros::ok())
        {
            if(counter>100)
            {
                break;
            }

            if(finished_)
            {
                break;
            }
            counter++;

          r.sleep();
        }

        result_.finished=finished_;

         if(finished_){
            as_.setSucceeded(result_);


         }else{

             as_.setAborted(result_);

         }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tipper");

  tipperAction spin_degrees(ros::this_node::getName());
  ros::spin();
  return 0;
}























