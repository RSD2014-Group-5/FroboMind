/*
 * particlefilter.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: kasper
 */

#include "particlefilter.h"

#define X_RANGE 3.00
#define Y_RANGE 3.00
#define MIN_VALID_MEASUREMENTS 25
#define MAX_DISTANCE 0.10
#define MAX_ANGLE M_PI/12 // 15 degrees

ParticleFilter::ParticleFilter(){

}

ParticleFilter::ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double measurements_noise, double movement_noise, double turning_noise){

	num_particles = numberOfParticles;
	length_y = len_x;
	offset_y = off_x;
	length_x = len_y;
	offset_x = off_y;
	max_angle = max_ang;

	measurement_noise = measurements_noise;
	move_noise = movement_noise;
	turn_noise = turning_noise;

	print = 1;

	max_prob = 1;

	yaw = 0;

	time_t sec;
	time(&sec);
	srand(uint(sec));
	for (int i = 0; i < num_particles; i++)
	{
		double x = (double)rand()/double(RAND_MAX) * length_x + offset_x - 0.5 * length_x;
		srand(uint(x*1000.0+i*1000));
		double y = (double)rand()/double(RAND_MAX) * length_y + offset_y - 0.5 * length_y;
		srand(uint(y*1000.0+i*2000));
		double theta = (double)rand()/double(RAND_MAX) * max_angle - 0.5 * max_angle;
		if (theta < 0)
			theta += 2*M_PI;
		srand(uint(theta*1000.0+i*3000));

		particles.push_back(Frobit(x,y,theta));
	}

	std::cout << "ParticleFilter created:" << std::endl;

	//ROS_INFO("Particles: %d, len_x: %.3f, off_x: %.3f, len_y: %.3f, off_y: %.3f, max_ang: %.3f, meas_noise: %.3f, move_noise: %.3f, turn_noise: %.3f", numberOfParticles, len_x, off_x, len_y, off_y, max_ang, measurements_noise, movement_noise, turning_noise);



}

ParticleFilter::~ParticleFilter()
{
}

visualization_msgs::MarkerArray ParticleFilter::getParticlesMarker(void)
{
	return particles_marker;
}

double ParticleFilter::gaussian(double mu, double sigma, double x)
{
	return exp(-(pow(mu - x,2.0) / pow(sigma,2.0) / 2));
}

void ParticleFilter::updateParticlesMarker(void)
{
	particles_marker.markers.clear();

	for (int i = 0; i < num_particles; i++)
	{
		double prob = (particles[i].w / max_prob);
		geometry_msgs::Quaternion pose =  tf::createQuaternionMsgFromYaw(particles[i].theta);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "particles";
		marker.id = i;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = particles[i].x;
		marker.pose.position.y = particles[i].y;
		marker.pose.position.z = 0;
		marker.pose.orientation = pose;
		marker.scale.x = 0.2;
		marker.scale.y = 0.02;
		marker.scale.z = 0.05;
		marker.color.a = 0.7;
		//marker.color.r = prob;
		marker.color.r = 0;
		marker.color.g = 0.0;
		marker.color.b = 1;
		//marker.color.b = 1-prob;

		particles_marker.markers.push_back(marker);
	}


	//Make a green arrow for the frobit pose (the chosen particle by the robust mean)
	visualization_msgs::Marker marker;

	geometry_msgs::Quaternion pose = tf::createQuaternionMsgFromYaw(last_pos.theta);
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "Frobit pose";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = last_pos.x;
	marker.pose.position.y = last_pos.y;
	marker.pose.position.z = 0;
	marker.pose.orientation = pose;
	marker.scale.x = 0.25;
	marker.scale.y = 0.05;
	marker.scale.z = 0.08;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	particles_marker.markers.push_back(marker);

}

void ParticleFilter::motionUpdate(const nav_msgs::Odometry& delta_position){
	boost::mt19937 rng(time(0));

	boost::normal_distribution<> nd_x(0.0, move_noise);
	boost::normal_distribution<> nd_y(0.0, move_noise);
	boost::normal_distribution<> nd_theta(0.0, turn_noise);

	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_x(rng, nd_x);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_y(rng, nd_y);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_theta(rng, nd_theta);

	bt_q.setW(delta_position.pose.pose.orientation.w);
	bt_q.setZ(delta_position.pose.pose.orientation.z);
	bt_q.setY(delta_position.pose.pose.orientation.y);
	bt_q.setX(delta_position.pose.pose.orientation.x);
	yaw = tf::getYaw(bt_q);

	//Remember x and y are flipped for now because of the stupid map(can maybe get fixed later)
	for (int i = 0; i < num_particles; i++){
		//not swapped commented
		//particles[i].y += sqrt(pow(delta_position.pose.pose.position.x,2.0)+pow(delta_position.pose.pose.position.y,2.0))*sin(particles[i].theta) + var_y();
		//particles[i].x += sqrt(pow(delta_position.pose.pose.position.x,2.0)+pow(delta_position.pose.pose.position.y,2.0))*cos(particles[i].theta) + var_x();
		particles[i].x += sqrt(pow(delta_position.pose.pose.position.x,2.0)+pow(delta_position.pose.pose.position.y,2.0))*sin(particles[i].theta) + var_y();
		particles[i].y += sqrt(pow(delta_position.pose.pose.position.x,2.0)+pow(delta_position.pose.pose.position.y,2.0))*cos(particles[i].theta) + var_x();

		particles[i].theta += yaw + var_theta();
		if (particles[i].theta < 0)
			particles[i].theta += 2*M_PI;
		else if (particles[i].theta > 2*M_PI)
			particles[i].theta -= 2*M_PI;

		//ROS_INFO("particle[%d]: x: %f, y: %f, th: %f",i,particles[i].x,particles[i].y,particles[i].theta);

	}
	//ROS_INFO("particle x: %f, y: %f, th: %f",particles[num_particles].x,particles[num_particles].y,particles[num_particles].theta);
}

void ParticleFilter::measurementUpdate(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::OccupancyGrid& map){
	double prob;
	double temp_error = 0;
	int valid_measurements = 0;
	float res = map.info.resolution;
	int width = map.info.width;
	int height = map.info.height;

	for (int i = 0; i < num_particles; i++){
		//Reset weight
		prob = 1;
		for (int j = 0; j < pointCloud.points.size(); j++){
			//Transpose laserscan points
			geometry_msgs::Point32 t;
			t.x = pointCloud.points[j].x * cos(particles[i].theta) - pointCloud.points[j].y * sin(particles[i].theta);
			t.y = pointCloud.points[j].x * sin(particles[i].theta) + pointCloud.points[j].y * cos(particles[i].theta);

			//Check with range restrictions and calculate error
			if ((t.x < X_RANGE/2 && t.x > -X_RANGE/2) && (t.y < Y_RANGE/2 && t.y > -Y_RANGE/2)){
				t.y += particles[i].y;
				t.x += particles[i].x;

				valid_measurements++;

				int y = (int)((float)(t.y)/res);
				int x = (int)((float)(t.x)/res);

				// Beregner fejlen
				//if (map.data[y*width+x] == 0)
					//temp_error = 0.25;
				//else
					temp_error = (100 - map.data[y*width+x]) * 0.001;

				//Product of guassians
				prob *= gaussian(0,measurement_noise,temp_error);
			}
		}

		if(valid_measurements > MIN_VALID_MEASUREMENTS)
			particles[i].w = prob;
		else
			particles[i].w = 0;

	}

}

//Resampling wheel
void ParticleFilter::resampling(){
	srand(time(0));
	std::vector<Frobit> temp_particles;

	// initialize the start index to a random particle
	int index = (double)rand()/double(RAND_MAX)*num_particles;

	max_prob = 0;
	// Find the highest weight
	for (int i = 0; i < num_particles; i++)
		if (particles[i].w > max_prob)
			max_prob = particles[i].w;

	double beta = 0;
	// Perform the selection
	for (int i = 0; i < num_particles; i++)
	{
		srand(uint(beta+i));
		// Increase beta with a random number between 0 and 2 * max_prob
		beta += (double)rand()/double(RAND_MAX) * 2.0 * max_prob;
		while (beta > particles[index].w)
		{
			beta -= particles[index].w;
			index = (index + 1) % num_particles;
		}
		Frobit fb = Frobit(particles[index].x,particles[index].y,particles[index].theta,particles[index].w);
		temp_particles.push_back(fb);
	}
	particles.clear();
	for (int i = 0; i<num_particles; i++)
		particles.push_back(temp_particles[i]);
}

// Find Vehicle using the robust mean method
Frobit ParticleFilter::findVehicle()
{
	double x(0),y(0),theta(0);
	double norm = 0;

	max_prob = 0;
	int index;
	for (int i = 0; i < num_particles; i++)
		if (particles[i].w > max_prob)
		{
			max_prob = particles[i].w;
			index = i;
		}
	Frobit fb = Frobit(particles[index].x,particles[index].y,particles[index].theta,particles[index].w);

	for (int i = 0; i < num_particles; i++)
	{
		if (sqrt(pow(fb.x-particles[i].x,2.0) + pow(fb.y-particles[i].y,2.0)) < MAX_DISTANCE && (abs(fb.theta - particles[i].theta < MAX_ANGLE) || abs(fb.theta + 2*M_PI - particles[i].theta) < MAX_ANGLE || abs(fb.theta - 2*M_PI - particles[i].theta) < MAX_ANGLE))
		{
			x += particles[i].x * particles[i].w;
			y += particles[i].y * particles[i].w;

			// orientation is tricky because it is cyclic. By normalizing
			// around the first particle we are somewhat more robust to
			// the 0=2pi problem
			double temp_theta = (particles[i].theta - particles[0].theta + M_PI);
			if (temp_theta < 0)
				temp_theta += 2*M_PI;
			else if (temp_theta > 2*M_PI)
				temp_theta -= 2*M_PI;
			temp_theta += particles[0].theta - M_PI;

			theta += temp_theta * particles[i].w;

			norm += particles[i].w;
		}
	}
	last_pos.x = x / norm;
	last_pos.y = y / norm;
	last_pos.theta = theta / norm;
	last_pos.w = max_prob;
	//last_pos.header.stamp = ros::Time::now();

	Frobit r;
	r.y = x / norm;
	r.x = y = y / norm;
	r.theta = theta / norm;
	r.theta = 2*M_PI - r.theta;
	if (r.theta > 2*M_PI)
		r.theta -= 2*M_PI;
	else if (r.theta < 0)
		r.theta += 2*M_PI;

	r.w = max_prob;
	//r.header.stamp = ros::Time::now();

	return r;

}

nav_msgs::Odometry ParticleFilter::update(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::Odometry& delta_position, const nav_msgs::OccupancyGrid& map){
	nav_msgs::Odometry placeh;
	motionUpdate(delta_position);
	measurementUpdate(pointCloud,map);
	resampling();
	findVehicle();
	return placeh;
}
