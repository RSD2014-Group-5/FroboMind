#include "slam.h"

SLAMPose::SLAMPose(int t){

	waypoint_number = 0;
}

void SLAMPose::PositionCallback(const nav_msgs::OdometryConstPtr& odom_msg){

	try{
		//tf_listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
		//ROS_INFO("x,y: %f,%f",transform.getOrigin().getX(),transform.getOrigin().getY());
		//ROS_INFO("try");
		odom_pose.pose.position.x = odom_msg->pose.pose.position.x;
		odom_pose.pose.position.y = odom_msg->pose.pose.position.y;
		odom_pose.pose.position.z = odom_msg->pose.pose.position.z;
		odom_pose.pose.orientation = odom_msg->pose.pose.orientation;
		odom_pose.header.frame_id = odom_msg->header.frame_id;
		//odom_pose.header.stamp = odom_msg->header.stamp;
		tf_listener.transformPose("/map",odom_pose,map_pose);
		slam_pose.header.frame_id = map_pose.header.frame_id;
		slam_pose.header.stamp = ros::Time();
		slam_pose.header.seq = odom_pose.header.seq;
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
		waypoint_pub.publish(wp_point);
	}
	waypoint_number++;

}

void SLAMPose::ClickedPointCallback(geometry_msgs::PointStamped clicked_point){
	ROS_INFO("Next WP (x,y): (%f,%f)", clicked_point.point.x,clicked_point.point.y);

	wp_obs.X = clicked_point.point.x;
	wp_obs.Y = clicked_point.point.y;
	wp_obs.Theta = 20;
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

