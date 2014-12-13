#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <math.h>
#include <sys/time.h>

class Reverse
{
public:
    Reverse();
    void spinItDJ();
    void reverseCallback(const std_msgs::Float32::ConstPtr& data);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &data);
    void gotoWaypoint();
    void moveRobot(double forward, double turn);

    //Publishers
    ros::Publisher reverse_pub;
    ros::Publisher velocity_pub;
    ros::Publisher deadman_pub;

    //Subscribers
    ros::Subscriber reverse_sub;
    ros::Subscriber odometry_sub;

    geometry_msgs::PoseWithCovariance odometryPose;
    float reverse;
    geometry_msgs::PoseWithCovariance start_pose;

    struct
    {
        std::string reverse_sub;
        std::string reverse_pub;
        std::string odometry_sub;
        std::string cmd_vel_pub;
        std::string deadman_pub;

        double speed;

    } parameters;
};

#endif // WAYPOINT_H
