/*
 * RowExtractorNode.h
 *
 *  Created on: Oct 1, 2014
 *      Author: kent
 */

#ifndef REDLINEEXTRACTORNODE_H_
#define REDLINEEXTRACTORNODE_H_

#include <vector>
#include <sstream>

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
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>

#include <redline_following/Line.h>

namespace imageEncoding = sensor_msgs::image_encodings;

typedef pcl::PointXYZ PointT;

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
	    
	    std::string lineTopic;
	    std::string lineLink;
	    ros::Publisher linePublisher;
	    
	    redline_following::Line lineMsg;
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
    
    //  Line models
    struct
    {
        bool lineOneFound;
        bool lineTwoFound;
        Eigen::VectorXf lineOne;
        Eigen::VectorXf lineTwo;
    } models;

	//	Processed data
	cv::Mat poiImage;                       //	Points of interest (Image for visualization purpose)
	pcl::PointCloud<PointT> poiCloud;		//	Points of interest (Point Cloud)
	pcl::PointCloud<PointT> ransacLineOne;  //	Points of interest (Point Cloud from ransac line one)
	pcl::PointCloud<PointT> ransacLineTwo;  //	Points of interest (Point Cloud from ransac line two)

	//	Callback
	void cameraImageCallback(const sensor_msgs::Image::ConstPtr& data);

	//	Processors
	void processImage (void);
	void makeLineModels (void);
	void generateMsg (void);
	
public:
	RedlineExtractorNode();
	virtual ~RedlineExtractorNode();

	void makeItSpin (void);
};

#endif /* REDLINEEXTRACTORNODE_H_ */
