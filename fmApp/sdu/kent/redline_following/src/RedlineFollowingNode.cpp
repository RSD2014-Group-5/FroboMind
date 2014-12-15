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
    
    this->nodeHandler.param<std::string>("line_topic", this->output.lineTopic, "line");
    this->nodeHandler.param<std::string>("line_link", this->output.lineLink, "/line_link");
    this->output.linePublisher = this->nodeHandler.advertise<redline_following::Line>(this->output.lineTopic, 10);

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
    		this->makeLineModels();
    		this->generateMsg();
    		
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
        
        //  Lines
        this->output.lineMsg.header.stamp = ros::Time::now();
        this->output.lineMsg.header.frame_id = this->output.lineLink;
        this->output.linePublisher.publish(this->output.lineMsg);
        
		//	Wait
		r.sleep();
	}
}

void RedlineExtractorNode::cameraImageCallback(const sensor_msgs::Image::ConstPtr& data)
{
	//	Convert to Open CV image format
	this->input.cvImage = cv_bridge::toCvCopy(data, imageEncoding::BGR8);
	
	//  Perspective warping
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
                
                //  TODO: Homography
                this->poiCloud.push_back(PointT((double)j,(double)i,0.0f));                 //  Add interest point to the point cloud
            }
		}
	}
}

void RedlineExtractorNode::makeLineModels (void)
{
    std::vector<int> ransacIndices;
    pcl::PointCloud<PointT> ransacPoints;

    //  Setup ransac
    pcl::SampleConsensusModelLine<PointT>::Ptr lineModel(new pcl::SampleConsensusModelLine<PointT> (this->poiCloud.makeShared()));
    pcl::RandomSampleConsensus<PointT> ransac(lineModel);
    
    //  Extract line one if possible
    ransac.setDistanceThreshold(this->params.ransacDistance);
    this->models.lineOneFound = ransac.computeModel();
    ransac.getInliers(ransacIndices);
    ransac.getModelCoefficients(this->models.lineOne);
    
    //  Extract points from poiCloud to ransacLineOne
    pcl::copyPointCloud<pcl::PointXYZ>(this->poiCloud, ransacIndices, this->ransacLineOne);
    
    //  Remove points from poiCloud according to indices
    pcl::PointCloud<PointT>::iterator it;
    for (int i = ransacIndices.size() - 1; i >= 0; i--)
    {
        it = this->poiCloud.begin() + ransacIndices.at(i);
        this->poiCloud.erase(it);
    }
    
    //  Extract line two if possible
    pcl::SampleConsensusModelLine<PointT>::Ptr lineModel1(new pcl::SampleConsensusModelLine<PointT> (this->poiCloud.makeShared()));
    pcl::RandomSampleConsensus<PointT> ransac1(lineModel1);
    ransacIndices.clear();
    ransac1.setDistanceThreshold(this->params.ransacDistance);
    this->models.lineTwoFound = ransac1.computeModel();
    ransac1.getInliers(ransacIndices);
    ransac1.getModelCoefficients(this->models.lineTwo);
    
    //  Extract points from poiCloud to ransacLineTwo
    pcl::copyPointCloud<pcl::PointXYZ>(this->poiCloud, ransacIndices, this->ransacLineTwo);
}

void RedlineExtractorNode::generateMsg (void)
{
    double step = 100;
    double x1, x2, a1, a2, y1, y2, b1, b2;
    
    this->output.lineMsg.lineOneFound = false;
    this->output.lineMsg.lineTwoFound = false;
    this->output.lineMsg.angle = .0;
    this->output.lineMsg.lineOne.clear();
    this->output.lineMsg.lineOne.push_back(.0);
    this->output.lineMsg.lineOne.push_back(.0);
    this->output.lineMsg.lineOne.push_back(.0);
    this->output.lineMsg.lineOne.push_back(.0);
    this->output.lineMsg.lineTwo.clear();
    this->output.lineMsg.lineTwo.push_back(.0);
    this->output.lineMsg.lineTwo.push_back(.0);
    this->output.lineMsg.lineTwo.push_back(.0);
    this->output.lineMsg.lineTwo.push_back(.0);
    
//    this->output.lineMsg.lineTwo = [.0,.0,.0,.0];
    
    if (this->models.lineOneFound)
    {
        x1 = this->models.lineOne.coeff(0),
        x2 = this->models.lineOne.coeff(1),
        a1 = this->models.lineOne.coeff(3),
        a2 = this->models.lineOne.coeff(4);
    
        cv::Point p1(x1 - step * a1, x2 - step * a2), p2(x1 + step * a1, x2 + step * a2);
        cv::line(this->poiImage, p1, p2, cv::Scalar(0,0,255));
        
        this->output.lineMsg.lineOneFound = true;
        this->output.lineMsg.lineOne.clear();
        this->output.lineMsg.lineOne.push_back(x1);
        this->output.lineMsg.lineOne.push_back(x2);
        this->output.lineMsg.lineOne.push_back(a1);
        this->output.lineMsg.lineOne.push_back(a2);
    }

    if (this->models.lineTwoFound)
    {
        y1 = this->models.lineTwo.coeff(0),
        y2 = this->models.lineTwo.coeff(1),
        b1 = this->models.lineTwo.coeff(3),
        b2 = this->models.lineTwo.coeff(4);

        cv::Point p3(y1 - step * b1, y2 - step * b2), p4(y1 + step * b1, y2 + step * b2);
        cv::line(this->poiImage, p3, p4, cv::Scalar(0,255,0));
        
        this->output.lineMsg.lineTwoFound = true;
        this->output.lineMsg.lineTwo.clear();
        this->output.lineMsg.lineTwo.push_back(y1);
        this->output.lineMsg.lineTwo.push_back(y2);
        this->output.lineMsg.lineTwo.push_back(b1);
        this->output.lineMsg.lineTwo.push_back(b2);
    }
   
    if (this->models.lineOneFound && this->models.lineTwoFound)
        this->output.lineMsg.angle = std::acos(a1 * b1 + a2 * b2) * 180.0 / M_PI;
   
    //  Convert data
    pcl::toROSMsg(this->poiCloud, this->output.poiCloudMsg);
}
