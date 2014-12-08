#include "tipper.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "Tipper");

    Tipper tipperNode;
    tipperNode.spinItDJ();

    return 0;
}

Tipper::Tipper()
{
    ros::NodeHandle n;
    ros::NodeHandle local_nh = ros::NodeHandle("~");

    local_nh.param<std::string>("tipper_nmea_sub", parameters.tipper_nmea_sub, "/fmData/nmea_from_tipper");
    tipperMsg_sub = n.subscribe<msgs::nmea>(parameters.tipper_nmea_sub, 2, &Tipper::msgFromTipper, this);

    local_nh.param<std::string>("tipper_nmea_pub", parameters.tipper_nmea_pub, "/fmData/nmea_to_tipper");
    tipperMsg_pub = n.advertise<msgs::nmea>(parameters.tipper_nmea_pub, 5);

    local_nh.param<std::string>("tipper_done", parameters.tipper_done_pub, "/tip_done");
    tipperDone_pub = n.advertise<std_msgs::Bool>(parameters.tipper_done_pub, 5);

    local_nh.param<std::string>("tipper_start", parameters.tipper_start_sub, "/tip_start");
    tipperStart_sub = n.subscribe<std_msgs::Bool>(parameters.tipper_start_sub, 5, &Tipper::tipStart, this);

    //Global variables
    tip_command = false;
    moving_up = false;
    moving_down = false;

    //Move down the tipper to initial state in the bottom.
    tipInit();

    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
}

void Tipper::tipInit()
{
    //Moves the tipper down to start position.
    ros::Rate r(30);
    while ( ros::ok() )
    {
        //ROS_DEBUG("Moving up: %d Moving down: %d End button: %d Tip command: %d", moving_up, moving_down, end_button, tip_command);
        if(end_button)
            break;
        else
           commandTip( false, true );

        ros::spinOnce();
        r.sleep();
    }

    tipper_initialized = true;

}

void Tipper::publishProgress(bool done)
{
    //Publish if the tipper is done tipping, to let the other nodes know.
    std_msgs::Bool done_temp = std_msgs::Bool();
    done_temp.data = done;
    tipperDone_pub.publish( done_temp );
}

void Tipper::tipStart(const std_msgs::Bool::ConstPtr &msg)
{
    tip_command = msg->data;
}

void Tipper::msgFromTipper(const msgs::nmea::ConstPtr &msg)
{
    if (msg->data.size() >= 2)
    {
        std::string top = msg->data[0];
        top_button = boost::lexical_cast<bool>(top);

        std::string end = msg->data[1];
        end_button = boost::lexical_cast<bool>(end);

        if( end_button || top_button ) //If any of the switches are pressed, then there is no moving of the tipper.
        {
            moving_down = false;
            moving_up = false;
        }
        //ROS_ERROR("DATA RECIEVED FROM TIPPER! Top endstop: %s Bottom endstop: %s ", top.c_str(), end.c_str());
    }
}

void Tipper::commandTip(bool up, bool down)
{
    //Send command to tipper firmware to move up or down.
    msgs::nmea msgToTipper;
    msgToTipper.header.stamp = ros::Time::now();
    msgToTipper.type = "TPRSD";
    msgToTipper.data.clear();
    msgToTipper.data.push_back(boost::lexical_cast<std::string>((int)up));      //Move tipper up
    msgToTipper.data.push_back(boost::lexical_cast<std::string>((int)down));    //Move tipper down

    tipperMsg_pub.publish( msgToTipper );

    //Publish status.
    publishProgress( false );

    //Sets the global variables in order to keep track of the tippers current task.
    moving_down = down;
    moving_up = up;
}

void Tipper::spinItDJ()
{
    ros::Rate r(30);
    while ( ros::ok() )
    {
        //ROS_DEBUG("Moving up: %d Moving down: %d End button: %d Tip command: %d", moving_up, moving_down, end_button, tip_command);

        if( tipper_initialized )
        {
            //Checks if a new tip is arriving, and none is in progress.
            if( moving_up == false && moving_down == false && end_button == true && tip_command == true ) //Move up if a tip is commanded and and the end button is pressed.
                commandTip( true, false );
            else if ( moving_up == false && moving_down == false && top_button == true && tip_command == true) //Move down if the tipper reached the top and a tip is commanded/requested.
            {
                commandTip( false, true );
                tip_command = false;
            } else if ( moving_up == false && moving_down == false && end_button == true && tip_command == false ) {
                //Publish to let ROS nodes known that the tipper is not doing any work anymore.
                publishProgress( true );
            } else if ( moving_up == true || moving_down == true || (end_button == false && top_button == false) )
                publishProgress( false );
        }
        ros::spinOnce();
        r.sleep();
    }
}
