#include "slam.h"

SLAMPose::SLAMPose(int t)
{

	waypoint_number = 0;
	angle_print_cnt = 0;
	reinit = true;
	//0 = floor
	//1 = dispenser
	nav_mode = 0;
	reinit_pose.pose.pose.position.x = 0.55;
	reinit_pose.pose.pose.position.y = -0.34;
	reinit_pose.pose.pose.position.z = 0;
	reinit_pose.pose.pose.orientation.x = 0.0;
	reinit_pose.pose.pose.orientation.y = 0.0;
	reinit_pose.pose.pose.orientation.z = -0.333999438659;
	reinit_pose.pose.pose.orientation.w = 0.942573273001;
	reinit_pose.header.frame_id = "/map";
	reinit_pose.header.stamp = ros::Time();
	//initialpose_pub.publish(reinit_pose);
}

void SLAMPose::ReinitCallback(const std_msgs::Bool reinit_amcl)
{
	reinit = reinit_amcl.data;
	/*
	  		reinit_pose.pose.pose.position.x = odom_msg->pose.pose.position.x;
			reinit_pose.pose.pose.position.y = odom_msg->pose.pose.position.y;
			reinit_pose.pose.pose.position.z = odom_msg->pose.pose.position.z;
			reinit_pose.pose.pose.orientation = odom_msg->pose.pose.orientation;
			reinit_pose.header.frame_id = "/map";
			reinit_pose.header.stamp = ros::Time();
			initialpose_pub.publish(reinit_pose);
	 */
}

void SLAMPose::NavModeCallback(const std_msgs::UInt32 nav_mode_msg)
{
	nav_mode = nav_mode_msg.data;
	if(nav_mode_msg.data == 1)
	{
		//Initialize AMCL to a fixed position when switching to dispenser area mode
		reinit_pose.pose.pose.position.x = 0.55;
		reinit_pose.pose.pose.position.y = -0.34;
		reinit_pose.pose.pose.position.z = 0;
		reinit_pose.pose.pose.orientation.x = 0.0;
		reinit_pose.pose.pose.orientation.y = 0.0;
		reinit_pose.pose.pose.orientation.z = -0.333999438659;
		reinit_pose.pose.pose.orientation.w = 0.942573273001;
		reinit_pose.header.frame_id = "/map";
		reinit_pose.header.stamp = ros::Time();
		initialpose_pub.publish(reinit_pose);
	}
}

void SLAMPose::GPSPoseCallback(const nav_msgs::OdometryConstPtr& gps_pose_msg)
{

	robot_pose.pose.pose.position.x = gps_pose_msg->pose.pose.position.x;
	robot_pose.pose.pose.position.y = gps_pose_msg->pose.pose.position.y;
	robot_pose.pose.pose.position.z = gps_pose_msg->pose.pose.position.z;
	robot_pose.pose.pose.orientation = gps_pose_msg->pose.pose.orientation;
	robot_pose.header.frame_id = gps_pose_msg->header.frame_id;
	robot_pose.header.stamp = ros::Time();
	if(nav_mode == 0)
	{
		if(angle_print_cnt == 10)
		{
			angle_print_cnt = 0;
			ROS_INFO("Angle: %f", (tf::getYaw(robot_pose.pose.pose.orientation)*180)/M_PI);
		}
		angle_print_cnt++;
		pose_pub.publish(robot_pose);
	}
}

void SLAMPose::PositionCallback(const nav_msgs::OdometryConstPtr& odom_msg){
	if(nav_mode == 1)
	{
		/*
		 * Dispenser area
		 * Get the map > odom transform from AMCL and use that to calculate and
		 * publish the pose in the map frame
		 */
		try
		{
			odom_pose.pose.position.x = odom_msg->pose.pose.position.x;
			odom_pose.pose.position.y = odom_msg->pose.pose.position.y;
			odom_pose.pose.position.z = odom_msg->pose.pose.position.z;
			odom_pose.pose.orientation = odom_msg->pose.pose.orientation;
			odom_pose.header.frame_id = odom_msg->header.frame_id;
			//odom_pose.header.stamp = odom_msg->header.stamp;
			tf_listener.transformPose("/map",odom_pose,map_pose);
			slam_pose.header.frame_id = map_pose.header.frame_id;
			slam_pose.header.stamp = ros::Time();
			//slam_pose.header.seq = odom_pose.header.seq;
			slam_pose.pose.pose.position.x = map_pose.pose.position.x;
			slam_pose.pose.pose.position.y = map_pose.pose.position.y;
			slam_pose.pose.pose.position.z = map_pose.pose.position.z;
			slam_pose.pose.pose.orientation = map_pose.pose.orientation;
			pose_pub.publish(slam_pose);

		}

		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
	}

}

void SLAMPose::WaypointCallback(std_msgs::Bool wp_reached){
	ROS_INFO("Wpcallback");

	geometry_msgs::PoseStamped next_wp_pose;
	geometry_msgs::PoseStamped temp;

	if(waypoint_number < wp_list.size()){
		temp = wp_list[waypoint_number];
		temp.header.stamp = ros::Time();
		temp.pose.orientation = last_pos.pose.pose.orientation;
		next_wp.pose.pose.position.x = temp.pose.position.x;
		next_wp.pose.pose.position.y = temp.pose.position.y;
		next_wp.pose.pose.position.z = 0;
		next_wp.pose.pose.orientation = temp.pose.orientation;
		next_wp.header.frame_id ="/map";

		//waypoint_pub.publish(next_wp);

		wp_point.x = next_wp.pose.pose.position.x;
		wp_point.y = next_wp.pose.pose.position.y;
		//waypoint_pub.publish(wp_point);
	}
	waypoint_number++;

}

void SLAMPose::ClickedPointCallback(geometry_msgs::PointStamped clicked_point){
	ROS_INFO("Next WP (x,y): (%f,%f)", clicked_point.point.x,clicked_point.point.y);

	wp_obs.X = clicked_point.point.x;
	wp_obs.Y = clicked_point.point.y;
	wp_obs.Theta = -1;
	wp_obs.Obstacle_avoidance = false;
	waypoint_pub.publish(wp_obs);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "waypoint";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = clicked_point.point.x;
	marker.pose.position.y = clicked_point.point.y;
	marker.pose.position.z = 0;
	marker.scale.x = 0.07;
	marker.scale.y = 0.07;
	marker.scale.z = 0.07;
	marker.color.a = 1;
	marker.color.r = 1;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	pose_marker_pub.publish(marker);

}

void SLAMPose::LaserScanCallback(sensor_msgs::LaserScan laser_scan){

}

