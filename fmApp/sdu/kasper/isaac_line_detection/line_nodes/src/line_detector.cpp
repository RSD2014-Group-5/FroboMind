#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//#include <opencv/cvwimage.h>
#include <opencv/cv.h>


// ojo!!!
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#include <std_msgs/Bool.h>

#include "cv_functions.h"

// INclude msg class
#include <line_nodes/line_points.h>

#include <sstream>

using namespace cv;

// Define names of subscriber and publisher
//#define SUBSCRIBER "rsd_camera/bar_camera"

//#define SUBSCRIBER "/usb_cam/image_raw"
#define SUBSCRIBER "/line_action/image_raw"


//#define PUBLISHER "camera/angle"
#define PUBLISHER "line_node/line_points"
#define IMAGE_PUBLISHER "line_node/image_lines"
#define CROSS_DETECTED_PUBLISHER "line_node/cross_detected"

class SubscribeAndPublish
{
	public:
	SubscribeAndPublish()
	{
		//Topic to publish
        pub_ = n_.advertise<line_nodes::line_points>(PUBLISHER, 1);
		im_pub_ = n_.advertise<sensor_msgs::Image>(IMAGE_PUBLISHER, 1);
        cross_pub_=n_.advertise<std_msgs::Bool>(CROSS_DETECTED_PUBLISHER,1);


		//Topic to subscribe
		sub_ = n_.subscribe(SUBSCRIBER, 1, &SubscribeAndPublish::callback, this);
	}

	// Define Callback function called by the subscriber
	void callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// To select wheter to show or not the output image
        bool show_image=TRUE;
		
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Start the line detection
		
		// Resize the image
		 Mat image;
	   	// resize(cv_ptr->image,image,cvSize(cv_ptr->image.size[1]/4,cv_ptr->image.size[0]/4));
		 image=cv_ptr->image;

	   	 Mat image_hsv;
	    	 cvtColor(image,image_hsv,CV_RGB2HSV );	

		 // Filter by color
		 Mat mask;
		 inRange(image_hsv,Scalar(100,50,50), Scalar(130, 255, 255),mask);

		// Open to delete small areas 
		 Mat mask2;
		 morphologyEx(mask,mask2,cv::MORPH_OPEN,getStructuringElement(MORPH_RECT, Size(10,10)));
		
		 // Detect lines
		 vector<Vec2f> lines;

		 // Find lines of preprocesed image
		 HoughLines( mask2, lines, 1, CV_PI/360, 350);

		// Merge lines
		vector<Vec2f> lines_merged=merge_lines(lines);
	



		// Show result image



		Mat image_lines;
		image_lines=display_lines(image,lines_merged,"Lines merged");

/*
        if (show_image)
        {
            imshow("lines_detected",image_lines);
            cv::waitKey(3);
        }
*/

        if(lines_merged.size()>1)
        {
            // there is a cross comming

            std_msgs::Bool pushed_msg;
            pushed_msg.data=TRUE;

            cross_pub_.publish(pushed_msg);

        }

        int index=0;
        double max_difference=90;


      // cout<<"New"<<endl;
        // Find the line with less angle with the angle closser to 90 degrees
        for (uint i=0;i<lines_merged.size();i++)
        {
            double difference;

            if(lines_merged[i][1]>1.57)
            {
                difference=abs(lines_merged[i][1]-3.14);

            }else{
                difference=abs(lines_merged[i][1]);
            }

            cout<<"angle_line: "<<lines_merged[i][1]<<" difference "<<difference<<endl;

            if(difference<max_difference)
            {

                max_difference=difference;
                index=i;
                cout<<"final index= "<<index<<endl;

            }
        }

        vector<Vec2f> line_followed;

        if(lines_merged.size()>0)
        {
            line_followed.push_back(lines_merged.at(index));
        }
        vector<Vec2f> result=transform_point(line_followed,image.cols,image.rows);

		// Print the points of the lines
	/*
		for (int i=0;i<result.size();i++)
		{
			cout<<"Point "<<i<<": "<<result[i]<<endl;	
		}
	*/

		// Publish output image
	     cv_bridge::CvImage cvi;
	     cvi.header.stamp = ros::Time::now();
	     cvi.header.frame_id = "image";
	     cvi.encoding = "bgr8";
	
             cvi.image=image_lines;

	     sensor_msgs::Image im;
	     cvi.toImageMsg(im);		
	
	     im_pub_.publish(im);


		

        line_nodes::line_points to_publish;
		if(result.size()!=0)
		{		
			to_publish.x1=result[0][0];
			to_publish.y1=result[0][1];
			to_publish.x2=result[1][0];
			to_publish.y2=result[1][1];	
			pub_.publish(to_publish);

		}		

		// Publish the string
		//pub_.publish(str);
		
	}

	private:
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Publisher im_pub_;
    	ros::Publisher cross_pub_;
	ros::Subscriber sub_;


};


//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "rsd_line_detector");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
