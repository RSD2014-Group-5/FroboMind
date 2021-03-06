#ifndef RANSAC_H
#define RANSAC_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
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


class FindLines
{
public:
    FindLines();
    void spinItDJ();
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);
    void activeCallback(const std_msgs::Bool::ConstPtr& data);
    std::vector<LineStruct> ransac(pcl::PointCloud<PointT>::Ptr laserRangePointCloud, int min_inliers, double threshold);
    void followWall(std::vector<LineStruct> lines, double desiredDistance);

    //Publishers
    ros::Publisher marker_pub;
    ros::Publisher velocity_pub;
    ros::Publisher deadman_pub;

    //Subscribers
    ros::Subscriber laserScan;
    ros::Subscriber active_sub;

    tf::TransformListener tfListener_;
    laser_geometry::LaserProjection projector_;
    pcl::PointCloud<PointT> laserScanCloud;

    //
    bool leftWall;

    struct
    {
        std::string cmd_vel_pub;
        std::string deadman_pub;
        std::string active_sub;
        std::string laserscan_sub;
        std::string laserscan_frame;

        bool active;

        double wall_distance;

        double ransac_threshold;
        double ransac_inliers;
        bool show_lines;

        double lidar_offset_x;
        double lidar_offset_y;


        double kp;
        double ki;
        double kd;

        double forward_velocity;
        double turn_velocity;
        double turn_factor;
    } parameters;
};

#endif // RANSAC_H
