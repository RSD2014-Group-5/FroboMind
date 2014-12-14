#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//#include <opencv/cvwimage.h>
#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <zbar.h>

#include "std_msgs/String.h"

#include <sstream>

using namespace cv;
using namespace zbar;


//Find the correct name for the kinect topic
//#define PUBLISHER "camera/image"
//#define SUBSCRIBER "rsd_camera/image"
//#define SUBSCRIBER "rsd_camera/bar_camera"
#define SUBSCRIBER "line_action/image_raw"
//#define SUBSCRIBER "camera/bar_camera"
#define PUBLISHER "line_node/barcode"


class SubscribeAndPublish
{
	public:
	SubscribeAndPublish()
	{
		//Topic to publish
		pub_ = n_.advertise<std_msgs::String>(PUBLISHER, 1000);
		//Topic you want to subscribe
		sub_ = n_.subscribe(SUBSCRIBER, 1, &SubscribeAndPublish::callback, this);
	}

	// Define Callback function called by the subscriber
	void callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// To select wheter to show or not the output image
		bool show_image=TRUE;

		std_msgs::String str;
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

	   // Bar_code detection
		// Image out is used to show the input image with the barcode with a rectangle
		Mat imgout; 
		//if (show_image){imgout=cv_ptr->image;} 
		// Input image hav to be in grayscale. Otherwise barcode detection does not work
		Mat img;
		cvtColor(cv_ptr->image,img,CV_BGR2GRAY);
		


		// Create detector object
		ImageScanner scanner;
		scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
		int width = img.cols;
		int height = img.rows;
		uchar *raw = (uchar *)img.data;
		// wrap image data
		Image image(width, height, "Y800", raw, width * height);
		// scan the image for barcodes
		scanner.scan(image);

		// Publish all detected barcodes
		for(Image::SymbolIterator symbol = image.symbol_begin();symbol != image.symbol_end();++symbol) 
		{
			// Add string with barcode data to the message
			str.data=symbol->get_data();
			
			/*
			// Draw rectangles on barcodes
			if (show_image)
			{
				int n = symbol->get_location_size();
				vector<Point> vp;
				for(int i=0;i<n;i++)
				{
				   	vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
				}
				RotatedRect r = minAreaRect(vp);
				Point2f pts[4];
				r.points(pts);
				for(int i=0;i<4;i++)
				{
					line(imgout,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
				}
			}
			*/
		}
		// Show result image

		/*
		if (show_image)	
		{
			imshow("Barcode detection output", imgout);
			cv::waitKey(3);
		}
		*/

		// Publish the last of the barcodes detected

		if(str.data!=""){
		pub_.publish(str);
		}
	}

	private:
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;

};


//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "rsd_barcode_detect");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
