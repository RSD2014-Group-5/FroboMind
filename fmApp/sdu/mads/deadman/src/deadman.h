#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sdu_rsd_waypoint/Waypoint.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <eigen3/Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>

#include <math.h>
#include <sys/time.h>

typedef pcl::PointXYZ PointT;

class Deadman
{
public:
    Deadman();
    void spinItDJ();
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);
    void activeCallback(const std_msgs::Bool::ConstPtr &data);
    bool obstacleDetection();

    //Publishers
    ros::Publisher deadman_pub;

    //Subscribers
    ros::Subscriber active_sub;
    ros::Subscriber laserScan;

    tf::TransformListener tfListener_;
    laser_geometry::LaserProjection projector_;
    pcl::PointCloud<PointT> laserScanCloud;

    bool active;
    int closest_count;

    //
    std::vector<sdu_rsd_waypoint::Waypoint> waypoints;
    geometry_msgs::PoseWithCovariance odometryPose;

    struct
    {
        std::string active_sub;
        std::string deadman_pub;
        std::string laserscan_sub;
        std::string laserscan_frame;

        int max_points;
        double max_distance;
        int angle;

    } parameters;
};

#endif // WAYPOINT_H
