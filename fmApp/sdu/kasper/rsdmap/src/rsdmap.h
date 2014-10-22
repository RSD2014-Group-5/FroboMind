/*
 * rsdmap.h
 *
 *  Created on: Oct 16, 2014
 *      Author: kasper
 */

#ifndef SOURCE_DIRECTORY__FMAPP_SDU_KASPER_RSDMAP_H_
#define SOURCE_DIRECTORY__FMAPP_SDU_KASPER_RSDMAP_H_

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
//#include "pcl/io/pcd_io.h"
#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#include <tf/transform_datatypes.h>
#include "particlefilter.h"

class RSDMap
{
	private:

		ParticleFilter particle_filter;
		int numberOfParticles;
		double len_x,off_x,len_y,off_y,max_ang, measurements_noise, movement_noise, turning_noise, min_valid_measurements;
		tf::TransformBroadcaster map_broadcaster;



		nav_msgs::Odometry delta_odom;
		nav_msgs::Odometry last_odom;
		tf::Quaternion q_last;
		tf::Quaternion q_new;
		tf::Quaternion q_delta;



	public:

		RSDMap(int NumberOfParticles,double Len_x,double Len_y, double offset_x, double offset_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double min_laserpoints, double map_res);
		ros::Publisher marker_pub;
		ros::Publisher map_pub;
		ros::Subscriber odom_sub;
		ros::Publisher delta_position_pub;

		ros::Publisher point_cloud_pub;
		ros::Subscriber laser_scan_sub;

		laser_geometry::LaserProjection projector;
		nav_msgs::OccupancyGrid map;
		int coordinateToIndex(int x, int y);
		void createMap(double map_sizex, double map_sizey, double mapres, double startx, double starty);
		void publishMap();
		void sendMapTransform();
		void PositionCallback(const nav_msgs::OdometryConstPtr& odom_msg);
		void LaserScanCallback(sensor_msgs::LaserScan laser_scan);

		nav_msgs::Odometry placeholder;

};


#endif /* SOURCE_DIRECTORY__FMAPP_SDU_KASPER_RSDMAP_H_ */
