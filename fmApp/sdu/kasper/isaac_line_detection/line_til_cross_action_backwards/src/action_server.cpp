#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <line_til_cross_action_backwards/GocellAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <msgs/IntStamped.h>

#define IMAGE_SUBSCRIBER "usb_cam/image_raw"
#define IMAGE_PUBLISHER "line_action/image_raw"
#define PID_ENABLE_PUBLISHER "line_action/pid_enable_back"
#define CROSS_DETECTED_SUBSCRIBER "line_node/cross_detected"
#define ENCODER_LEFT_SUBSCRIBER "fmInformation/encoder_left"

class GocellAction
{
	protected:
		ros::NodeHandle nh_;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<line_til_cross_action_backwards::GocellAction> as_;
		std::string action_name_;
		// create messages that are used to published feedback/result
		line_til_cross_action_backwards::GocellFeedback feedback_;
		line_til_cross_action_backwards::GocellResult result_;
		line_til_cross_action_backwards::GocellGoal goal_;
		ros::Publisher image_pub_;
		ros::Publisher pid_pub_;
		ros::Subscriber image_sub_;
		ros::Subscriber cross_sub_;
        ros::Subscriber encoder_left_sub_;
		cv::Mat image_;
		bool success_;
		int counter_;
        int encoder_val_;

        int cross_counter_;
        bool prev_cross_;
        int prev_cross_counter_;
        int encoder_offset_;


	public:
		GocellAction(std::string name) :
		// action server
		as_(nh_, name, boost::bind(&GocellAction::executeCB, this, _1), false),
		action_name_(name)
		{
			// Sibscribers and publishers
			image_pub_ = nh_.advertise<sensor_msgs::Image>(IMAGE_PUBLISHER, 1);
			image_sub_ = nh_.subscribe(IMAGE_SUBSCRIBER, 1, &GocellAction::callback_image, this);
			pid_pub_=nh_.advertise<std_msgs::Bool>(PID_ENABLE_PUBLISHER,1);
			cross_sub_ = nh_.subscribe(CROSS_DETECTED_SUBSCRIBER, 1, &GocellAction::callback_cross_detected, this);
            encoder_left_sub_ = nh_.subscribe(ENCODER_LEFT_SUBSCRIBER, 1, &GocellAction::callback_encoder, this);


			counter_=0;
			success_=FALSE;
			as_.start();
		}

		~GocellAction(void)
		{
		}


        void callback_encoder(const msgs::IntStampedPtr& msg)
        {
            // Store the value of the encoder
            encoder_val_=msg->data;


        }


		
		void callback_cross_detected(const std_msgs::BoolConstPtr& msg)
		{
			ROS_INFO("cross detected");


            if(!prev_cross_)
            {
                if(cross_counter_==0)
                {    // this is the cross to stop
                    success_=msg->data;
                }else{
                    cross_counter_--;
                }
            }
            prev_cross_=true;
            prev_cross_counter_=0;


		}
		void callback_image(const sensor_msgs::ImageConstPtr& msg)
		{
			// Copy the message
			// image_message_=msg;
			// ROS_INFO("Image received");
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
			image_=cv_ptr->image;
			// counter_=counter_+1;
		}
		void executeCB(const line_til_cross_action_backwards::GocellGoalConstPtr &goal)
		{
			// helper variables
			goal_.cell_name=goal->cell_name;
			success_ = false;
			// Publish the image to the line_detector
			// Publish output image
			//If the image is not empty
			counter_=0;

            // counter of crosses left

            prev_cross_= false;
            cross_counter_=0;
            encoder_offset_=0;

            if(goal->cell_name=="")
            {
                cross_counter_=1;
            }
            if(goal->cell_name=="Robot 1")
            {
                nh_.getParam("Robot_1_offset",encoder_offset_);
            }
            if(goal->cell_name=="Robot 2")
            {
                nh_.getParam("Robot_2_offset",encoder_offset_);
            }
            if(goal->cell_name=="Robot 3")
            {
                nh_.getParam("Robot_3_offset",encoder_offset_);
            }



			std_msgs::Bool pid_message;
			pid_message.data=TRUE;
			pid_pub_.publish(pid_message);
			ros::Rate r(25);


            prev_cross_counter_=0;

			while (ros::ok())
			{


                if(prev_cross_)
                {
                    prev_cross_counter_++;

                    if(prev_cross_counter_>30)
                    {
                        prev_cross_=false;
                        prev_cross_counter_=0;
                    }

                }



                if(counter_>1000)
				{
					break;
				}
				if(success_)
				{
					break;
				}
				if(image_.data)
				{
					// ROS_INFO("publishing image");
					cv_bridge::CvImage cvi;
					cvi.header.stamp = ros::Time::now();
					cvi.header.frame_id = "image";
					cvi.encoding = "bgr8";
					cvi.image=image_;
					sensor_msgs::Image im;
					cvi.toImageMsg(im);
					image_pub_.publish(im);
				}
				counter_++;
				r.sleep();
			}
			//result_.sequence = feedback_.sequence;
			result_.cell_final="final_robot";

			// set the action state to succeeded

            if(success_)
            {
                // Continue navigating the offset distance

                // Continue till the cross
                int init_encoder=encoder_val_;
                ros::Rate s(25);
                while (ros::ok())
                {
                    if(image_.data)
                    {
                        // ROS_INFO("publishing image");
                        cv_bridge::CvImage cvi;
                        cvi.header.stamp = ros::Time::now();
                        cvi.header.frame_id = "image";
                        cvi.encoding = "bgr8";
                        cvi.image=image_;
                        sensor_msgs::Image im;
                        cvi.toImageMsg(im);
                        image_pub_.publish(im);
                    }


                    //Continue navigating unless advanced all the desired distance
                    if(encoder_val_<(init_encoder-encoder_offset_))
                    {
                      break;
                    }
                    if(encoder_val_>init_encoder)
                    {
                        ROS_ERROR("Overflow on the encoder");
                        std::cout<<"Last value of encoder_val= "<<init_encoder<<std::endl;
                        break;
                    }

                    s.sleep();
                }


				as_.setSucceeded(result_);
				pid_message.data=FALSE;
				pid_pub_.publish(pid_message);

            }else{
				std_msgs::Bool pid_message;
				// abort and stop publishing
				pid_message.data=FALSE;
				pid_pub_.publish(pid_message);
				as_.setAborted(result_);
			}
		}
};
	int main(int argc, char** argv)
	{
		ros::init(argc, argv, "Gocell_back");
		GocellAction gocell(ros::this_node::getName());
		ros::spin();
		return 0;
	}

