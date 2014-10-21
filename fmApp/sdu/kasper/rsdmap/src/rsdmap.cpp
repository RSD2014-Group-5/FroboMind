/*
 * rsdmap.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: kasper
 */



#include "rsdmap.h"


RSDMap::RSDMap(int NumberOfParticles,double Len_x,double Len_y,double Max_ang, double Measurements_noise, double Movement_noise, double Turning_noise, double map_res){

	numberOfParticles = NumberOfParticles;
	len_x = Len_x;
	len_y = Len_y;
	max_ang = Max_ang;
	measurements_noise = Measurements_noise;
	movement_noise = Movement_noise;
	turning_noise = Turning_noise;

	//off_x = 8;
	//off_y = 3.2;
	off_x = 5;
	off_y = 5;

	particle_filter = ParticleFilter(numberOfParticles,len_x,off_x,len_y,off_y,max_ang, measurements_noise, movement_noise, turning_noise);
}

void RSDMap::publishMap()
{
	map_pub.publish(map);
}

int RSDMap::coordinateToIndex(int x, int y) {
	int index = x + y * map.info.width;
	return index;
}

void RSDMap::createMap(double map_sizex, double map_sizey, double mapres, double startx, double starty) {
	double map_size_x = map_sizex;
	double map_size_y = map_sizey;
	double map_resolution = mapres;
	double start_x = startx;
	double start_y = starty;
	//ROS_INFO("CreateMap");
	map.info.width = (uint32_t)(map_size_x / map_resolution);
	map.info.height= (uint32_t)(map_size_y / map_resolution);
	map.info.resolution = map_resolution;
	map.info.map_load_time = ros::Time::now();
	map.header.stamp = ros::Time::now();
	map.header.frame_id ="/map";

	map.data.clear();

	int range_x, range_y;
	range_x = map.info.width;
	range_y = map.info.height;

	//Set all map to zeros
	for(int y = 0; y< range_y; y++)
		for(int x = 0; x< range_x; x++)
			map.data.push_back(0);

	//4 walls going "vertical"
	//ramp wall right
	int start_cord_x = 5.8/map_resolution;
	int start_cord_y = 3.70/map_resolution;
	int end_cord_x = 8.15/map_resolution;
	int end_cord_y = 3.90/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}
	//ramp wall left
	start_cord_x = 5.8/map_resolution;
	start_cord_y = 2.65/map_resolution;
	end_cord_x = 8.15/map_resolution;
	end_cord_y = 2.85/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//box wall left
	start_cord_x = 8.2/map_resolution;
	start_cord_y = 1.9/map_resolution;
	end_cord_x = 10.1/map_resolution;
	end_cord_y = 2.10/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//box wall right
	start_cord_x = 8.2/map_resolution;
	start_cord_y = 4.1/map_resolution;
	end_cord_x = 10.1/map_resolution;
	end_cord_y = 4.25/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//top horizontal wall
	start_cord_x = 9.95/map_resolution;
	start_cord_y = 2.0/map_resolution;
	end_cord_x = 10.1/map_resolution;
	end_cord_y = 4.2/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//bottom left horizontal wall
	start_cord_x = 8.1/map_resolution;
	start_cord_y = 1.85/map_resolution;
	end_cord_x = 8.25/map_resolution;
	end_cord_y = 2.85/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//bottom right horizontal wall
	start_cord_x = 8.1/map_resolution;
	start_cord_y = 3.70/map_resolution;
	end_cord_x = 8.25/map_resolution;
	end_cord_y = 4.25/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//Pillar #2
	start_cord_x = 5.8/map_resolution;
	start_cord_y = 1.45/map_resolution;
	end_cord_x = 6.2/map_resolution;
	end_cord_y = 1.85/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//Pillar #3
	start_cord_x = 5.8/map_resolution;
	start_cord_y = 4.95/map_resolution;
	end_cord_x = 6.2/map_resolution;
	end_cord_y = 5.35/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}

	//Pillar #4
	start_cord_x = 5.8/map_resolution;
	start_cord_y = 8.45/map_resolution;
	end_cord_x = 6.2/map_resolution;
	end_cord_y = 8.8/map_resolution;
	for(int y = start_cord_y;  y<= end_cord_y; y++){
		for(int x = start_cord_x;  x <= end_cord_x; x++){
			map.data[coordinateToIndex(y,x)] = 100;
		}
	}
}

void RSDMap::sendMapTransform() {

	geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(0.0);

	geometry_msgs::TransformStamped maptf;
	maptf.header.stamp = ros::Time::now();
	maptf.header.frame_id = "/map";
	maptf.child_frame_id = "odom";

	maptf.transform.translation.x = 5.0;
	maptf.transform.translation.y = 5.0;
	maptf.transform.translation.z = 0.0;
	maptf.transform.rotation = map_quat;

	map_broadcaster.sendTransform(maptf);

}

void RSDMap::PositionCallback(const nav_msgs::OdometryConstPtr& odom_msg){

	//calc and publish delta odometry data
	//x and y are subtractions
	delta_odom.pose.pose.position.x = (odom_msg->pose.pose.position.x - last_odom.pose.pose.position.x);
	delta_odom.pose.pose.position.y = (odom_msg->pose.pose.position.y - last_odom.pose.pose.position.y);
	//tf::quaternionMsgToTF(last_odom.pose.pose.orientation, q_last);
	//tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q_new);

	//do some stuff with quaternions
	q_new.setW(odom_msg->pose.pose.orientation.w);
	q_new.setZ(odom_msg->pose.pose.orientation.z);
	q_new.setY(odom_msg->pose.pose.orientation.y);
	q_new.setX(odom_msg->pose.pose.orientation.x);
	//ROS_INFO("q_last: x: %f, y: %f, z: %f , w: %f",q_last.getX(),q_last.getY(),q_last.getZ(),q_last.getW());
	//ROS_INFO("odomsg: x: %f, y: %f, z: %f , w: %f",odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z,odom_msg->pose.pose.orientation.w);

	//difference between 2 quaternions is one inversed times the other.
	q_delta = (q_last.inverse() * q_new);
	delta_odom.pose.pose.orientation.w = q_delta.getW();
	delta_odom.pose.pose.orientation.z = q_delta.getZ();
	delta_odom.pose.pose.orientation.x = q_delta.getX();
	delta_odom.pose.pose.orientation.y = q_delta.getY();
	//tf::quaternionTFToMsg(q_delta,delta_odom.pose.pose.orientation);

	//publish the delta odometry (it is actually just used as a variable inside this class)
	delta_position_pub.publish(delta_odom);
	//store value for next callback
	last_odom.pose.pose.position.x = odom_msg->pose.pose.position.x;
	last_odom.pose.pose.position.y = odom_msg->pose.pose.position.y;
	q_last.setW(odom_msg->pose.pose.orientation.w);
	q_last.setZ(odom_msg->pose.pose.orientation.z);
	q_last.setY(odom_msg->pose.pose.orientation.y);
	q_last.setX(odom_msg->pose.pose.orientation.x);

}

void RSDMap::LaserScanCallback(sensor_msgs::LaserScan laser_scan){
	sensor_msgs::PointCloud cloud;

	try{
    	projector.projectLaser(laser_scan, cloud);
    }
    catch (tf::TransformException& e){
        std::cout << "Error: " << e.what();
        return;
    }

    cloud.header.frame_id = "/map";
    cloud.header.stamp = ros::Time::now();
    point_cloud_pub.publish(cloud);

    //Run an iteration of the particle filter (motion update, measurement update, resampling, pose estimation)
	placeholder = particle_filter.update(cloud,delta_odom,map);

	//ROS_INFO("Position in map: x: %.3f y: %.3f th: %.3f",vehicle_position.position.x ,vehicle_position.position.y,vehicle_position.position.th);
	//ROS_INFO("Odom: x: %.3f y: %.3f th: %.3f",position.x ,position.y,position.th);

	//update visualization markers for rviz
	particle_filter.updateParticlesMarker();
	visualization_msgs::MarkerArray markerArray = particle_filter.getParticlesMarker();
	marker_pub.publish(markerArray);

	publishMap();
	sendMapTransform();



}
