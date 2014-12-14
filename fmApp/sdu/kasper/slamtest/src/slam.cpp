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

	Ramp_In_A.x = 1.0;
	Ramp_In_A.y = -0.41;

	Ramp_In_B.x = 0.72;
	Ramp_In_B.y = -0.80;

	Ramp_In_C.x = 2.24;
	Ramp_In_C.y = -1.90;
	corner_points.clear();
	corner_points.push_back(Ramp_In_A);
	corner_points.push_back(Ramp_In_B);
	corner_points.push_back(Ramp_In_C);

	for (int i = 0; i < corner_points.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "Ramp_In";
		marker.id = i;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = corner_points[i].x;
		marker.pose.position.y = corner_points[i].y;
		marker.pose.position.z = 0;
		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		marker.color.a = 1;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;

		marker_array.markers.push_back(marker);
	}

	Ramp_Out_A.x = 0.72;
	Ramp_Out_A.y = -0.80;

	Ramp_Out_B.x = 0.11;
	Ramp_Out_B.y = -1.38;

	Ramp_Out_C.x = 2.63;
	Ramp_Out_C.y = -2.36;
	corner_points.clear();
	corner_points.push_back(Ramp_Out_A);
	corner_points.push_back(Ramp_Out_B);
	corner_points.push_back(Ramp_Out_C);

	for (int i = 0; i < corner_points.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "Ramp_Out";
		marker.id = i+3;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = corner_points[i].x;
		marker.pose.position.y = corner_points[i].y;
		marker.pose.position.z = 0;
		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		marker.color.a = 1;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker_array.markers.push_back(marker);
	}

	InBox_A.x = 2.46;
	InBox_A.y = -1.54;

	InBox_B.x = 2.24;
	InBox_B.y = -1.90;

	InBox_C.x = 3.58;
	InBox_C.y = -2.77;
	corner_points.clear();
	corner_points.push_back(InBox_A);
	corner_points.push_back(InBox_B);
	corner_points.push_back(InBox_C);

	for (int i = 0; i < corner_points.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "InBox";
		marker.id = i+6;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = corner_points[i].x;
		marker.pose.position.y = corner_points[i].y;
		marker.pose.position.z = 0;
		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		marker.color.a = 1;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker_array.markers.push_back(marker);
	}

	Dispenser_A.x = 2.63;
	Dispenser_A.y = -2.36;

	Dispenser_B.x = 3.58;
	Dispenser_B.y = -2.77;

	Dispenser_C.x = 3.0;
	Dispenser_C.y = -3.51;
	corner_points.clear();
	corner_points.push_back(Dispenser_A);
	corner_points.push_back(Dispenser_B);
	corner_points.push_back(Dispenser_C);

	for (int i = 0; i < corner_points.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "Dispenser";
		marker.id = i+9;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = corner_points[i].x;
		marker.pose.position.y = corner_points[i].y;
		marker.pose.position.z = 0;
		marker.scale.x = 0.08;
		marker.scale.y = 0.08;
		marker.scale.z = 0.08;
		marker.color.a = 1;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;

		marker_array.markers.push_back(marker);
	}

	//charger
	/*
	 *
	pose:
    position:
      x: 2.94097087345
      y: -1.36965526915
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.444895489954
      w: 0.895582493698
	 *
	 *
	 *Waypoint reached at 2.974795, -1.331806, -1.000000 Robot(2.941834, -1.368542, 52.189346)
	 */

	/*
	 * ramp out / dispenser intersection
	  x: 2.63252301018
      y: -2.36282965026
	 *
	 *ramp in / ramp out skillelinie
	 *       x: 0.720235221179
      y: -0.802941231511
	 *
	 */

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
        reinit_pose.pose.pose.position.x = 0.333693197925;
        reinit_pose.pose.pose.position.y = -0.213405099882;
		reinit_pose.pose.pose.position.z = 0;
        reinit_pose.pose.pose.orientation.x = 0;
        reinit_pose.pose.pose.orientation.y = 0;
        reinit_pose.pose.pose.orientation.z = -0.309535494611;
        reinit_pose.pose.pose.orientation.w = 0.950887889068;
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
		SendRobotAreaPosition(robot_pose);
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
			SendRobotAreaPosition(slam_pose);
			pose_pub.publish(slam_pose);

		}

		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
	}

}

void SLAMPose::WaypointCallback(std_msgs::Bool wp_reached){
	/*
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
*/
}

void SLAMPose::SendRobotAreaPosition(nav_msgs::Odometry current_pose)
{
	Robot_point.x = current_pose.pose.pose.position.x;
	Robot_point.y = current_pose.pose.pose.position.y;
	//First test which nav_mode the robot is in (since there is no global / absolute coordinate frame)
	switch (nav_mode)
	{
		case 0:
		{
			if(current_pose.pose.pose.position.x > -0.1 && current_pose.pose.pose.position.x < 2.0)
			{
				robot_area_pos.data = "FloorIn";
			}
			else if(current_pose.pose.pose.position.x < -0.1 && current_pose.pose.pose.position.x  > -2.0)
			{
				robot_area_pos.data = "FloorOut";
			}
		}
		break;
		case 1:
		{
			if(withinArea(Ramp_In_A,Ramp_In_B,Ramp_In_C, Robot_point))
			{
				robot_area_pos.data = "RampIn";
			}
			else if(withinArea(Ramp_Out_A,Ramp_Out_B,Ramp_Out_C, Robot_point))
			{
				robot_area_pos.data = "RampOut";
			}
			else if(withinArea(InBox_A,InBox_B,InBox_C, Robot_point))
			{
				robot_area_pos.data = "InBox";
			}
			else if(withinArea(Dispenser_A,Dispenser_B,Dispenser_C, Robot_point))
			{
				robot_area_pos.data = "Dispenser";
			}
		}
		break;
		case 2:
		{
			robot_area_pos.data = "Line";
		}
		break;
	}
	robot_area_pos_pub.publish(robot_area_pos);
}

void SLAMPose::ClickedPointCallback(geometry_msgs::PointStamped clicked_point){
	ROS_INFO("Next WP (x,y): (%f,%f)", clicked_point.point.x,clicked_point.point.y);

	tmp.x = clicked_point.point.x;
	tmp.y = clicked_point.point.y;

	withinArea(Ramp_In_A,Ramp_In_B,Ramp_In_C,tmp);

	wp_obs.X = clicked_point.point.x;
	wp_obs.Y = clicked_point.point.y;
	wp_obs.Theta = -1;
	wp_obs.Obstacle_avoidance = false;
	waypoint_pub.publish(wp_obs);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "waypoint";
	marker.type = visualization_msgs::Marker::SPHERE;
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

	marker_array.markers.push_back(marker);
	//pose_marker_pub.publish(marker_array);
	updateMarkers();

}
bool SLAMPose::withinArea(geometry_msgs::Point A, geometry_msgs::Point B, geometry_msgs::Point C, geometry_msgs::Point R)
{
	//ROS_INFO("withinArea function called");
	AB.x = B.x-A.x;
	AB.y = B.y-A.y;

	BC.x = C.x-B.x;
	BC.y = C.y-B.y;

	AR.x = R.x-A.x;
	AR.y = R.y-A.y;

	BR.x = R.x-B.x;
	BR.y = R.y-B.y;
/*
	ROS_INFO("dotP(AB,AR): %f", dotP(AB,AR));
	ROS_INFO("dotP(AB,AB): %f", dotP(AB,AB));

	ROS_INFO("dotP(BC,BR): %f", dotP(BC,BR));
	ROS_INFO("dotP(BC,BC): %f", dotP(BC,BC));
*/
	if(0 <= dotP(AB,AR) && dotP(AB,AR) <= dotP(AB,AB) && 0 <= dotP(BC,BR) && dotP(BC,BR) <= dotP(BC,BC))
	{
		ROS_INFO("True: Point is within rectangle");
		return true;

	}
	ROS_INFO("False: Point is outside rectangle");
	return false;
}


void SLAMPose::updateMarkers(){
	pose_marker_pub.publish(marker_array);
}

float SLAMPose::dotP(geometry_msgs::Point U, geometry_msgs::Point V)
{
	return U.x*V.x+U.y*V.y;
}

void SLAMPose::LaserScanCallback(sensor_msgs::LaserScan laser_scan){

}

