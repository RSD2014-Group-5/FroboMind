/*
 * particlefilter.h
 *
 *  Created on: Oct 17, 2014
 *      Author: kasper
 */

#ifndef SOURCE_DIRECTORY__FMAPP_SDU_KASPER_PARTICLE_FILTER_SRC_PARTICLEFILTER_H_
#define SOURCE_DIRECTORY__FMAPP_SDU_KASPER_PARTICLE_FILTER_SRC_PARTICLEFILTER_H_

#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"

#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"

#include "tf/transform_listener.h"
#include <tf/LinearMath/Quaternion.h>

#include <tf/transform_datatypes.h>
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <vector>

struct Frobit
{
	Frobit()
	{
		x = 0; y = 0; theta = 0; w = 0;
	}
	Frobit(double X, double Y, double Theta)
	{
		x = X; y = Y; theta = Theta; w = 0;
	}
	Frobit(double X, double Y, double Theta, double W)
	{
		x = X; y = Y; theta = Theta; w = W;
	}
	double x;
	double y;
	double theta;
	double w;
};

class ParticleFilter{
	private:
		std::vector<Frobit> particles;
		int num_particles;

		double move_noise;
		double turn_noise;
		double measurement_noise;

		double offset_x;
		double length_x;
		double offset_y;
		double length_y;
		double max_angle;

		double max_prob;

		double min_valid_measurements;

		int printstuff;
		Frobit last_pos;
		visualization_msgs::MarkerArray particles_marker;

		int print;

		void newParticles(double ratio);
		void printParticles();

		void measurementUpdate(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::OccupancyGrid& map);
		void resampling();
		double gaussian(double mu, double sigma, double x);
		void addRandomGaussianNoise();

		tf::Quaternion bt_q;
		double yaw;

	public:

		ParticleFilter();
		ParticleFilter(int numberOfParticles,double len_x,double off_x,double len_y,double off_y,double max_ang, double measurements_noise, double movement_noise, double turning_noise, double min_laserpoints);
		~ParticleFilter();

		void updateParticlesMarker(void);
		void motionUpdate(const nav_msgs::Odometry& delta_position);
		Frobit findVehicle();
		visualization_msgs::MarkerArray getParticlesMarker(void);
		void resetParticleFilter(double off_x, double off_y);
		//nav_msgs::Odometry update(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::Odometry& delta_position, const nav_msgs::OccupancyGrid& map);
		Frobit update(const sensor_msgs::PointCloud& pointCloud, const nav_msgs::Odometry& delta_position, const nav_msgs::OccupancyGrid& map);
};

#endif /* SOURCE_DIRECTORY__FMAPP_SDU_KASPER_PARTICLE_FILTER_SRC_PARTICLEFILTER_H_ */
