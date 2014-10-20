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

  double map_resolution;

  nh.param<double>("map_resolution",map_resolution,0.05);

  nh.param<std::string>("lidar_sub_topic", lidar_sub_topic, "/base_scan");
  nh.param<std::string>("visualization_marker_pub_topic", viz_marker_pub_topic, "/fmExtractors/viz_marker_particle");
  nh.param<std::string>("point_cloud_pub_topic", point_cloud_pub_topic, "/fmExtractors/point_cloud");
  nh.param<std::string>("map_pub_topic", map_pub_topic, "/fmExtractors/map");

  //(int NumberOfParticles,double Len_x,double Len_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double map_res);
  RSDMap rm(10,3.5,0.5,2,0.05,0.05,0.05,0.05);
  //RSDMap rm(100,3.5,3.5,2,0.03,0.01,0.02,0.05);
  //RSDMap rm(20,3.5,3.5,2,0.0,0.0,0.0,0.05);

  rm.map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_pub_topic.c_str(),1);
  rm.marker_pub = n.advertise<visualization_msgs::MarkerArray>(viz_marker_pub_topic.c_str(), 1);
  rm.odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",2,&RSDMap::PositionCallback,&rm);
  rm.delta_position_pub = n.advertise<nav_msgs::Odometry>("/delta_odom",1);
  rm.laser_scan_sub = nh.subscribe<sensor_msgs::LaserScan> (lidar_sub_topic.c_str(), 2, &RSDMap::LaserScanCallback, &rm);
  rm.point_cloud_pub = n.advertise<sensor_msgs::PointCloud>(point_cloud_pub_topic.c_str(), 1);

  rm.createMap(15.0,15.0,0.05,5.0,5.0);
  rm.publishMap();
  ros::Rate loop_rate(10);
 /* while(ros::ok()){
	  ros::spinOnce();
	  //rm.publishMap();
	  //rm.sendMapTransform();
	  //rm.templateParticleMarkerUpdate();

	  loop_rate.sleep();
  }
*/
  ros::spin();
  return 0;
}
