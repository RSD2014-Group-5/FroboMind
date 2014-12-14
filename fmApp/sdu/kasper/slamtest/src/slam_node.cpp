#include "ros/ros.h"
#include "slam.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "slam_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  std::string odom_sub_topic;
  std::string odom_gnss_sub_topic;
  std::string lidar_sub_topic;
  std::string map_sub_topic;
  std::string viz_marker_pub_topic;
  std::string waypoint_pub_topic;
  std::string wp_reached_sub_topic;
  std::string clicked_point_sub_topic;
  std::string pose_pub_topic;
  std::string initialpose_pub_topic;
  std::string reinit_sub_topic;
  std::string gps_pose_sub_topic;
  std::string nav_mode_sub_topic;
  std::string robot_area_pos_pub_topic;

  nh.param<std::string>("odom_sub_topic",odom_sub_topic,"/fmKnowledge/pose");
  //nh.param<std::string>("odom_gnss_sub_topic",odom_sub_topic,"/fmKnowledge/pose_gnss");
  nh.param<std::string>("lidar_sub_topic", lidar_sub_topic, "/LaserScanner/scan");

  nh.param<std::string>("viz_marker_pub_topic", viz_marker_pub_topic, "/pose_marker");
  nh.param<std::string>("waypoint_pub_topic", waypoint_pub_topic, "/waypoint_goto");
  nh.param<std::string>("wp_reached_sub_topic", wp_reached_sub_topic, "/waypoint_reached");
  nh.param<std::string>("clicked_point_sub_topic", clicked_point_sub_topic, "/clicked_point");
  nh.param<std::string>("pose_pub_topic", pose_pub_topic, "/slam_pose");
  nh.param<std::string>("initialpose_pub_topic", initialpose_pub_topic, "/initialpose");
  nh.param<std::string>("reinit_sub_topic", reinit_sub_topic, "/reinit_amcl");
  nh.param<std::string>("gps_pose_sub_topic", gps_pose_sub_topic,"/fmKnowledge/pose_gnss");
  nh.param<std::string>("nav_mode_sub_topic", nav_mode_sub_topic, "/nav_mode");
  nh.param<std::string>("robot_area_pos_pub_topic", robot_area_pos_pub_topic, "/robot_area_pos");


  SLAMPose sp(0);

  sp.initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_pub_topic.c_str(), 1);
  sp.odom_sub = n.subscribe<nav_msgs::Odometry>(odom_sub_topic.c_str(),1,&SLAMPose::PositionCallback,&sp);
//sp.odom_gnss_sub = n.subscribe<nav_msgs::Odometry>(odom_gnss_sub_topic.c_str(),1,&SLAMPose::PositionCallback,&sp);
  sp.laser_scan_sub = n.subscribe<sensor_msgs::LaserScan> (lidar_sub_topic.c_str(), 1, &SLAMPose::LaserScanCallback, &sp);
  sp.pose_marker_pub = n.advertise<visualization_msgs::MarkerArray>(viz_marker_pub_topic.c_str(), 1);
  sp.waypoint_pub = n.advertise<sdu_rsd_waypoint::Waypoint>(waypoint_pub_topic.c_str(), 1);
  //sp.wp_reached_sub = n.subscribe<std_msgs::Bool>(wp_reached_sub_topic.c_str(), 1,&SLAMPose::WaypointCallback,&sp);
  sp.clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>(clicked_point_sub_topic.c_str(), 1,&SLAMPose::ClickedPointCallback,&sp);
  sp.pose_pub = n.advertise<nav_msgs::Odometry>(pose_pub_topic.c_str(), 1);
  sp.reinit_amcl_sub = n.subscribe<std_msgs::Bool>(reinit_sub_topic.c_str(), 1,&SLAMPose::ReinitCallback,&sp);
  sp.gps_pose_sub = n.subscribe<nav_msgs::Odometry>(gps_pose_sub_topic.c_str(),1,&SLAMPose::GPSPoseCallback,&sp);
  sp.nav_mode_sub = n.subscribe<std_msgs::UInt32>(nav_mode_sub_topic.c_str(),1,&SLAMPose::NavModeCallback,&sp);
  sp.robot_area_pos_pub = n.advertise<std_msgs::String>(robot_area_pos_pub_topic.c_str(), 1);
  ros::spin();
  return 0;
}
