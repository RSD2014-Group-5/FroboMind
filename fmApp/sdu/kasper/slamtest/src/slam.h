
#ifndef SOURCE_DIRECTORY__FMAPP_SDU_KASPER_SLAM_H_
#define SOURCE_DIRECTORY__FMAPP_SDU_KASPER_SLAM_H_

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <numeric>

#include <tf/transform_datatypes.h>
#include <sdu_rsd_waypoint/Waypoint.h>
class SLAMPose
{
	private:

		nav_msgs::Odometry last_pos;
		nav_msgs::Odometry slam_pose;
		nav_msgs::Odometry robot_pose;
		geometry_msgs::PoseStamped map_pose;
		geometry_msgs::PoseStamped odom_pose;
		geometry_msgs::PoseWithCovarianceStamped reinit_pose;
		geometry_msgs::Point wp_point;
		nav_msgs::Odometry next_wp;
		sdu_rsd_waypoint::Waypoint wp_obs;
		std_msgs::String robot_area_pos;

		geometry_msgs::Point Robot_point;

		geometry_msgs::Point Ramp_In_A;
		geometry_msgs::Point Ramp_In_B;
		geometry_msgs::Point Ramp_In_C;

		geometry_msgs::Point Ramp_Out_A;
		geometry_msgs::Point Ramp_Out_B;
		geometry_msgs::Point Ramp_Out_C;

		geometry_msgs::Point InBox_A;
		geometry_msgs::Point InBox_B;
		geometry_msgs::Point InBox_C;

		geometry_msgs::Point Dispenser_A;
		geometry_msgs::Point Dispenser_B;
		geometry_msgs::Point Dispenser_C;

		geometry_msgs::Point Dock1Dock2_inbox;
		geometry_msgs::Point Dock1Dock2_wall;
		geometry_msgs::Point Dock2Dock3_inbox;
		geometry_msgs::Point Dock2Dock3_wall;
		geometry_msgs::Point RampIn_corner;
		geometry_msgs::Point Dock3_corner;

//		geometry_msgs::Point R;
//		geometry_msgs::Point AB;
//		geometry_msgs::Point BC;
//		geometry_msgs::Point AR;
//		geometry_msgs::Point BR;
		geometry_msgs::Point tmp;
		geometry_msgs::Point AB, BC, AR, BR;
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::Marker temp_marker;
		std::vector<geometry_msgs::Point> corner_points;


		//0 = floor
		//1 = dispenser

		int nav_mode;
		int angle_print_cnt;
		bool reinit;
		int waypoint_number;

	public:

		SLAMPose(int t);

		ros::Publisher robot_area_pos_pub;
		ros::Publisher pose_pub;
		ros::Subscriber odom_sub;
		ros::Subscriber odom_gnss_sub;
		ros::Subscriber map_sub;
		ros::Publisher pose_marker_pub;
		ros::Subscriber laser_scan_sub;
		ros::Subscriber wp_reached_sub;
		ros::Publisher waypoint_pub;
		ros::Subscriber clicked_point_sub;
		ros::Publisher initialpose_pub;
		ros::Subscriber reinit_amcl_sub;
		ros::Subscriber gps_pose_sub;
		ros::Subscriber nav_mode_sub;

		tf::TransformListener tf_listener;
		tf::StampedTransform transform;

		std::vector<geometry_msgs::PoseStamped> wp_list;
		geometry_msgs::PoseStamped wp_to_push;


		void PositionCallback(const nav_msgs::OdometryConstPtr& odom_msg);
		void LaserScanCallback(sensor_msgs::LaserScan laser_scan);
		void WaypointCallback(std_msgs::Bool wp_reached);
		void ClickedPointCallback(geometry_msgs::PointStamped clicked_point);
		void ReinitCallback(std_msgs::Bool reinit_amcl);
		void GPSPoseCallback(const nav_msgs::OdometryConstPtr& gps_pose_msg);
		void NavModeCallback(std_msgs::UInt32 nav_mode_msg);
		void SendRobotAreaPosition(nav_msgs::Odometry);
		float dotP(geometry_msgs::Point U, geometry_msgs::Point V);
		bool withinArea(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, geometry_msgs::Point R);
		void updateMarkers();

};


#endif
