#ifndef RANSAC_H
#define RANSAC_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <sys/time.h>

#include <msgs/nmea.h>

class Tipper
{
public:
    Tipper();
    void spinItDJ();

    //Publishers
    ros::Publisher tipperMsg_pub;
    ros::Publisher tipperDone_pub;

    //Subscribers
    ros::Subscriber tipperMsg_sub;
    ros::Subscriber tipperStart_sub;

    bool tip_command;
    bool end_button;
    bool top_button;

    bool moving_up;
    bool moving_down;

    bool tipper_initialized;

    void msgFromTipper(const msgs::nmea::ConstPtr &msg);
    void commandTip(bool up, bool down);
    void publishProgress(bool done);
    void tipStart(const std_msgs::Bool::ConstPtr &msg);
    void tipInit();

    struct
    {
        std::string tipper_nmea_sub;
        std::string tipper_nmea_pub;
        std::string tipper_done_pub;
        std::string tipper_start_sub;
    } parameters;
};

#endif // RANSAC_H
