/*
 * RowExtractorNode.cpp
 *
 *  Created on: Oct 1, 2013
 *      Author: kent
 */

#include "RedlineFollowingNode.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "RedlineExtractorNode");

	RedlineExtractorNode rowNode;
	rowNode.makeItSpin();

	return 0;
}

RedlineExtractorNode::RedlineExtractorNode()
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loop_rate", this->loopRate, 10);

    //  Setup system parameters
    this->nodeHandler.param<double>("red_weight", this->params.redWeight, 2.0);
    this->nodeHandler.param<double>("green_weight", this->params.greenWeight, 1.0);
    this->nodeHandler.param<double>("blue_weight", this->params.blueWeight, 1.0);
    this->nodeHandler.param<double>("excess_red_treshold", this->params.threshold, 0.0);
    
    this->nodeHandler.param<double>("ransac_distance", this->params.ransacDistance, 20.0);

    //	Setup system output
    this->nodeHandler.param<std::string>("poi_cloud_topic", this->output.poiCloudTopic, "poi_cloud");
    this->nodeHandler.param<std::string>("poi_cloud_link", this->output.poiCloudLink, "/camera_link");
    this->output.poiCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud2>(this->output.poiCloudTopic, 10);

	//	Setup system input
	this->input.cvImage.reset(new cv_bridge::CvImage);

	this->nodeHandler.param<std::string>("camera_image_topic", this->input.imageTopic, "/usbCameraDriver/image_raw");
	this->nodeHandler.param<std::string>("camera_image_link", this->input.imageLink, "/camera_link");
	this->input.imageSubscriber = this->nodeHandler.subscribe<sensor_msgs::Image>(this->input.imageTopic, 10, &RedlineExtractorNode::cameraImageCallback, this);
}

RedlineExtractorNode::~RedlineExtractorNode()
{
	// TODO Auto-generated destructor stub
}

void RedlineExtractorNode::makeItSpin()
{
    //  Setup loop rate
	ros::Rate r(this->loopRate);

    //  Create OpenCV windows
    cv::namedWindow("Camera image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("POI image", cv::WINDOW_AUTOSIZE);

    //  Loop
	while (ros::ok() && this->nodeHandler.ok())
	{
		//	Update event que
		ros::spinOnce();

        try
        {
      		//	Process data
    		this->processImage();
    		this->extractLines();
        
            //  Update image window if any image has ben received
            cv::imshow("Camera image", this->input.cvImage->image);
            cv::imshow("POI image", this->poiImage);
            cv::waitKey(3);
        }
        catch (...)
        {
            ROS_INFO("No image to show ..."); 
        }
        
        //  Publish points of interest point cloud (poiCloud)
        this->output.poiCloudMsg.header.stamp = ros::Time::now();
        this->output.poiCloudMsg.header.frame_id = this->output.poiCloudLink;
        this->output.poiCloudPublisher.publish(this->output.poiCloudMsg);

		//	Wait
		r.sleep();
	}
}

void RedlineExtractorNode::cameraImageCallback(const sensor_msgs::Image::ConstPtr& data)
{
	//	Convert to Open CV image format
	this->input.cvImage = cv_bridge::toCvCopy(data, imageEncoding::BGR8);
}

void RedlineExtractorNode::processImage(void)
{
    cv::Vec3b color;
	double excessRedValue;
	
	//  Reset points of interest point cloud and image
	this->poiCloud.clear();
	this->poiImage = cv::Mat::zeros(this->input.cvImage->image.rows, this->input.cvImage->image.cols, CV_8UC3);
	
    //  Iterate through pixels of image (TODO: Faster method exists)
	for (int i = 0; i < this->input.cvImage->image.rows; i++)
	{
		for (int j = 0; j < this->input.cvImage->image.cols; j++)
		{
		    
            color = this->input.cvImage->image.at<cv::Vec3b>(i,j);                          //  Get color of current pixel
            excessRedValue =    this->params.redWeight      * color.val[2] - 
                                this->params.greenWeight    * color.val[1] - 
                                this->params.blueWeight     * color.val[0];                 //  Calculate excess red value

            if (excessRedValue > this->params.threshold)
            {
                this->poiImage.at<cv::Vec3b>(i,j) = cv::Vec3b(255, 255, 255);               //  Assign white to the pixel of interest
                this->poiCloud.push_back(PointT((double)j,(double)i,0.0f));                 //  Add interest point to the point cloud
            }
		}
	}

//    ROS_INFO("Number of points in cloud (%d) ...", (int)this->poiCloud.size());
    
    //  Convert data
    pcl::toROSMsg(this->poiCloud, this->output.poiCloudMsg);
}

void RedlineExtractorNode::extractLines(void)
{
    //  Temporary cloud
    pcl::PointCloud<PointT> tempPoints;

    //  Create vector for interesting point indices
    std::vector<int> inLiersLineOne;
    std::vector<int> inLiersLineTwo;
    pcl::PointIndices::Ptr indices;

    //  Clear point clouds containing lines
    this->lineOnePoints.clear();
    this->lineTwoPoints.clear();

    //  Setup ransac (Line 1)   
    pcl::SampleConsensusModelLine<PointT>::Ptr modelLineOne(new pcl::SampleConsensusModelLine<PointT> (this->poiCloud.makeShared()));
    pcl::RandomSampleConsensus<PointT> ransacLineOne(modelLineOne);
    
    //  Find line 1
    ransacLineOne.setDistanceThreshold(this->params.ransacDistance);
    ransacLineOne.computeModel();
    ransacLineOne.getInliers(inLiersLineOne);
    
    //  Filter outliers from poiCloud
    ROS_INFO(" Size: %d", (int)this->poiCloud.size());
    pcl::FilterIndices<PointT> indicesFilter;
    indicesFilter.setInputCloud (this->poiCloud.makeShared());
    indicesFilter.filter(inLiersLineOne);
    ROS_INFO(" Size: %d", (int)this->poiCloud.size());
    
    
//    pcl::copyPointCloud<pcl::PointXYZ>(this->poiCloud, inLiers, this->modelCloud);
    pcl::toROSMsg(this->lineOnePoints, this->output.poiCloudMsg);
}
