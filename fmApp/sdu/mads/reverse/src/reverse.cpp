#include "reverse.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "Reverse");

    Reverse reverseNode;
    reverseNode.spinItDJ();

    return 0;
}

Reverse::Reverse()
{
    ros::NodeHandle n;
    ros::NodeHandle local_nh = ros::NodeHandle("~");

    local_nh.param<std::string>("reverse_pub", parameters.reverse_pub, "/reverse_done");
    reverse_pub = n.advertise<std_msgs::Bool>(parameters.reverse_pub, 10);

    local_nh.param<std::string>("reverse_sub", parameters.reverse_sub, "/reverse_start");
    reverse_sub = n.subscribe<std_msgs::Float32>(parameters.reverse_sub, 10, &Reverse::reverseCallback, this);

    local_nh.param<std::string>("odometry_sub", parameters.odometry_sub, "/odom");
    odometry_sub = n.subscribe<nav_msgs::Odometry>(parameters.odometry_sub, 10, &Reverse::odometryCallback, this);

    local_nh.param<std::string>("cmd_vel_pub", parameters.cmd_vel_pub, "/fmCommand/cmd_vel");
    velocity_pub = n.advertise<geometry_msgs::TwistStamped>(parameters.cmd_vel_pub, 10);

    //local_nh.param<std::string>("deadman_pub", parameters.deadman_pub, "/fmCommand/deadman");
    //deadman_pub = n.advertise<std_msgs::Bool>(parameters.deadman_pub, 10);

    /* Loads parameters from launch file */
    local_nh.param<double>("speed", parameters.speed, 1);

    reverse = 0;

    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
}

void Reverse::reverseCallback(const std_msgs::Float32::ConstPtr &data)
{
    //If the reverse went from false to true, then reset the odometry
    if(data->data != reverse)
        this->start_pose = this->odometryPose;

    reverse = data->data;
}

void Reverse::odometryCallback(const nav_msgs::Odometry::ConstPtr &data)
{
    this->odometryPose = data->pose;
}

void Reverse::spinItDJ()
{
    ros::Rate r(100);
    while (ros::ok())
    {
        if(reverse > 0)
        {
            //Calculates the distance moved.
            double d = fabs( sqrt(pow(this->start_pose.pose.position.x - this->odometryPose.pose.position.x, 2) + pow(this->start_pose.pose.position.y - this->odometryPose.pose.position.y, 2)) );

            //If the robot has moved more than the distance, then stop reversing.
            if(d > reverse)
                reverse = 0;
            else
                moveRobot(parameters.speed, 0); //Reverse the robot
        }

        ros::spinOnce();
        r.sleep();
     }
}

void Reverse::moveRobot(double forward, double turn)
{
    //Publish deadman
    /*std_msgs::Bool deadman = std_msgs::Bool();
    deadman.data = true;
    deadman_pub.publish(deadman);*/

    geometry_msgs::TwistStamped cmd_velocity;
    cmd_velocity.header.stamp = ros::Time::now();
    cmd_velocity.twist.angular.z = turn; //angle; // * -1; //Negative = turning right, Positive = turning left
    cmd_velocity.twist.linear.x = forward;

    velocity_pub.publish(cmd_velocity);
}


