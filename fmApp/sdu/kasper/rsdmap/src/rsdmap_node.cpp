/*
 * rsdmap_node.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: kasper
 */

#include "ros/ros.h"
#include "rsdmap.h"
#include "particlefilter.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "rsdmap_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle n;

  std::string viz_marker_pub_topic;
  std::string map_pub_topic;
  std::string lidar_sub_topic;
  std::string point_cloud_pub_topic;
  std::string odom_sub_tupic;

  int numberOfParticles;
  double len_x;
  double len_y;
  double off_x;
  double off_y;
  double max_ang;
  double measurements_noise;
  double movement_noise;
  double turning_noise;
  double min_valid_measurements;

  double map_resolution;

  nh.param<int>("particles", numberOfParticles, 100);
  nh.param<double>("length_x", len_x, 3.5);
  nh.param<double>("length_y", len_y, 0.5);
  nh.param<double>("off_x", off_x, 5);
  nh.param<double>("off_y", off_y, 5);
  nh.param<double>("max_ang", max_ang, 2*M_PI);
  nh.param<double>("measurement_noise", measurements_noise, 0.02);
  nh.param<double>("movement_noise", movement_noise, 0.03);
  nh.param<double>("turning_noise", turning_noise, M_PI/16);
  nh.param<double>("min_valid_measurements", min_valid_measurements, 10);

  nh.param<double>("map_resolution",map_resolution,0.05);



  nh.param<std::string>("lidar_sub_topic", lidar_sub_topic, "/base_scan");
  nh.param<std::string>("visualization_marker_pub_topic", viz_marker_pub_topic, "/fmExtractors/viz_marker_particle");
  nh.param<std::string>("point_cloud_pub_topic", point_cloud_pub_topic, "/fmExtractors/point_cloud");
  nh.param<std::string>("map_pub_topic", map_pub_topic, "/fmExtractors/map");
  nh.param<std::string>("odom_sum_topic",odom_sub_tupic,"/odom");

  //(int particles,double Len_x,double Len_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double map_res);
  //RSDMap rm(100,3.5,0.5,2,0.02,0.05,0.05,0.05);
  RSDMap rm(numberOfParticles,len_x,len_y,off_x,off_y,max_ang,measurements_noise,movement_noise,turning_noise, min_valid_measurements,map_resolution);
  //RSDMap rm(100,3.5,3.5,2,0.03,0.01,0.02,0.05);


  rm.map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_pub_topic.c_str(),1);
  rm.marker_pub = n.advertise<visualization_msgs::MarkerArray>(viz_marker_pub_topic.c_str(), 1);
  rm.odom_sub = n.subscribe<nav_msgs::Odometry>(odom_sub_tupic.c_str(),2,&RSDMap::PositionCallback,&rm);
  rm.delta_position_pub = n.advertise<nav_msgs::Odometry>("/delta_odom",1);
  rm.laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan> (lidar_sub_topic.c_str(), 2, &RSDMap::LaserScanCallback, &rm);
  rm.point_cloud_pub = n.advertise<sensor_msgs::PointCloud>(point_cloud_pub_topic.c_str(), 1);

  rm.createMap(15.0,15.0,map_resolution,5.0,5.0);
  rm.publishMap();
  //ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}
