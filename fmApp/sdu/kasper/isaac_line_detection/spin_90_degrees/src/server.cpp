#include <ros/ros.h>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <spin_90_degrees/spin_degreesAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>

#define cmd_vel_PUBLISHER "/fmCommand/cmd_vel"
#define IMU_SUBSCRIBER "/fmInformation/imu"

class spin_degreesAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<spin_90_degrees::spin_degreesAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  spin_90_degrees::spin_degreesFeedback feedback_;
  spin_90_degrees::spin_degreesResult result_;
  spin_90_degrees::spin_degreesGoal goal_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber imu_sub_;





    double init_orientation_;
    bool success_;
    bool initial_;
    int counter_;


public:

    spin_degreesAction(std::string name) :

    // action server
    as_(nh_, name, boost::bind(&spin_degreesAction::executeCB, this, _1), false),
    action_name_(name)
  {
        // Subscribers and publishers

    imu_sub_ = nh_.subscribe(IMU_SUBSCRIBER, 1, &spin_degreesAction::imu_callback, this);
    cmd_vel_pub_=nh_.advertise<geometry_msgs::TwistStamped>(cmd_vel_PUBLISHER,1);



    counter_=0;
    success_=false;
    
    as_.start();
  }

  ~spin_degreesAction(void)
  {

  }


    void imu_callback(const sensor_msgs::ImuPtr& msg)
    {
        const double rad2dec=180/3.14;
        const double angle=70/rad2dec;
        // Transform to RPY rotation
         tf::Quaternion quat(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

         double roll, pitch, yaw;
         tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


        // std::cout<<"Y= "<<rad2dec*yaw<<std::endl;

         if(initial_)
         {
             std::cout<<"init Y= "<<rad2dec*yaw<<std::endl;
             init_orientation_=yaw;
             initial_=false;
         }

         // if spin positive
         if(goal_.direction=="left")
         {
             // normal situation
             if(init_orientation_ + angle<3.14)
             {
                 if(yaw > (init_orientation_+angle) )
                 {
                    success_=true;
                 }
             }else{
                 if((yaw > (-6.28+init_orientation_+angle))&(yaw<0))
                 {
                    success_=true;
                 }
             }

         // if turning right
         }else{

             // normal situation
             if((init_orientation_ - angle)> -3.14)
             {
                 if(yaw < (init_orientation_-angle) )
                 {
                    success_=true;
                 }
             }else{
                 if((yaw < (6.28+init_orientation_-angle))&(yaw>0))
                 {
                    success_=true;
                 }
             }
         }

    }

  void executeCB(const spin_90_degrees::spin_degreesGoalConstPtr &goal)
  {
    // helper variables

    goal_.direction=goal->direction;
    success_ = false;
    initial_= true;


    geometry_msgs::TwistStamped vel_cmd;


    vel_cmd.header.stamp=ros::Time::now();

    vel_cmd.twist.linear.x=0.0;
    vel_cmd.twist.linear.y=0.0;
    vel_cmd.twist.linear.z=0.0;
    vel_cmd.twist.angular.x=0.0;
    vel_cmd.twist.angular.y=0.0;


    if(goal->direction=="left")
    {
        vel_cmd.twist.angular.z=0.4;
    }else{
        vel_cmd.twist.angular.z=-0.4;
    }

    // Publish the image to the line_detector
    // Publish output image

    //If the image is not empty

    counter_=0;

    cmd_vel_pub_.publish(vel_cmd);

    ros::Rate r(25);
    while (ros::ok())
    {
        if(counter_>300)
        {
            break;
        }

        if(success_)
        {
            break;
        }

	 cmd_vel_pub_.publish(vel_cmd);
      r.sleep();
    }


      //result_.sequence = feedback_.sequence;
        result_.success="spinned";
         ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded

         vel_cmd.twist.angular.z=0;
         cmd_vel_pub_.publish(vel_cmd);


         if(success_){
            as_.setSucceeded(result_);

	  
         }else{

             as_.setAborted(result_);

         }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "spin_degrees");

  spin_degreesAction spin_degrees(ros::this_node::getName());
  ros::spin();
  return 0;
}























