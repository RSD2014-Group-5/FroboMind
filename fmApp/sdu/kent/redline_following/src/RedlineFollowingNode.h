/*
 * RowExtractorNode.h
 *
 *  Created on: Oct 1, 2014
 *      Author: kent
 */

#ifndef REDLINEEXTRACTORNODE_H_
#define REDLINEEXTRACTORNODE_H_

#include <vector>

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/filter_indices.h>

namespace imageEncoding = sensor_msgs::image_encodings;

typedef pcl::PointXYZ PointT;

//struct PointT : pcl::PointXYZ {inline PointT(double x, double y, double z) {this->x = x; this->y = y; this->z = z;}};

class RedlineExtractorNode
{
	//	Node handler
	ros::NodeHandle nodeHandler;
	int loopRate;

	//	System input
	struct
	{
		std::string imageTopic;
		std::string imageLink;
		ros::Subscriber imageSubscriber;

		cv_bridge::CvImage::Ptr cvImage;
	} input;

	//	System output
	struct
	{
	    std::string poiCloudTopic;
	    std::string poiCloudLink;
	    ros::Publisher poiCloudPublisher;
	    
	    sensor_msgs::PointCloud2 poiCloudMsg;
	} output;

    //  Parameters
    struct
    {
        double threshold;
        double redWeight;
        double greenWeight;
        double blueWeight;
        
        double ransacDistance;
    } params;

	//	Processed data
	pcl::PointCloud<PointT> modelCloud;
	pcl::PointCloud<PointT> poiCloud;		//	Points of interest (Point Cloud)
	pcl::PointCloud<PointT> lineOnePoints;
	pcl::PointCloud<PointT> lineTwoPoints;
	cv::Mat poiImage;                       //	Points of interest (Image for visualization purpose)

	//	Callback
	void cameraImageCallback(const sensor_msgs::Image::ConstPtr& data);

	//	Processors
	void processImage (void);
	void extractLines (void);

public:
	RedlineExtractorNode();
	virtual ~RedlineExtractorNode();

	void makeItSpin (void);
};

#endif /* REDLINEEXTRACTORNODE_H_ */
