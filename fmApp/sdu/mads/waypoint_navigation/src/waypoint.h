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

#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Eigen>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <math.h>
#include <sys/time.h>

typedef pcl::PointXYZ PointT;

struct LineStruct
{
    double a;
    double b;
    double c;

    double distance;

    bool operator < (const LineStruct& ls) const
    {
        return (distance < ls.distance);
    }
};


class WayPoint
{
public:
    WayPoint();
    void spinItDJ();
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);
    void waypointCallback(const geometry_msgs::Point::ConstPtr& data);
    void visualizeWaypoint(geometry_msgs::Point waypoint);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &data);
    void gotoWaypoint();
    bool obstacleDetection();

    //Publishers
    ros::Publisher marker_pub;
    ros::Publisher velocity_pub;
    ros::Publisher deadman_pub;
    ros::Publisher waypointreached_pub;

    //Subscribers
    ros::Subscriber waypoint_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber laserScan;

    tf::TransformListener tfListener_;
    laser_geometry::LaserProjection projector_;
    pcl::PointCloud<PointT> laserScanCloud;

    //
    std::vector<geometry_msgs::Point> waypoints;
    geometry_msgs::PoseWithCovariance odometryPose;

    struct
    {
        std::string waypoint_sub;
        std::string odometry_sub;
        std::string waypointreached_pub;
        std::string cmd_vel_pub;
        std::string deadman_pub;
        std::string laserscan_sub;
        std::string laserscan_frame;

        double kp_angle;
        double ki_angle;
        double kd_angle;

        double kp_distance;
        double ki_distance;
        double kd_distance;

        double kp_obstacle;
        double kd_obstacle;
        double speed_obstacle;

        double max_turn_output;
        double max_distance_obstacle;
        double waypoint_reached_threshold;
        int min_closepoints;
        double clearance_distance;

    } parameters;
};

#endif // WAYPOINT_H
