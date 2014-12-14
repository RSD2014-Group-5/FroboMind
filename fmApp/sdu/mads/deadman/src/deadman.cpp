#include "deadman.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "Deadman");

    Deadman deadmanNode;
    deadmanNode.spinItDJ();

    return 0;
}

Deadman::Deadman()
{
    ros::NodeHandle n;
    ros::NodeHandle local_nh = ros::NodeHandle("~");

    local_nh.param<std::string>("deadman_pub", parameters.deadman_pub, "/fmCommand/deadman");
    deadman_pub = n.advertise<std_msgs::Bool>(parameters.deadman_pub, 10);

    local_nh.param<std::string>("active_sub", parameters.active_sub, "/fmCommand/active");
    active_sub = n.subscribe<std_msgs::Bool>(parameters.active_sub, 10, &Deadman::activeCallback, this);

    local_nh.param<std::string>("laserscan_frame", parameters.laserscan_frame, "/laser");
    local_nh.param<std::string>("laserscan_sub", parameters.laserscan_sub, "/LaserScanner/scan");
    laserScan = n.subscribe<sensor_msgs::LaserScan>(parameters.laserscan_sub, 10, &Deadman::laserScanCallback, this);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));


    /* Loads parameters from launch file */
    local_nh.param<int>("max_points", parameters.max_points, 1);
    local_nh.param<double>("max_distance", parameters.max_distance, 1);
    local_nh.param<int>("angle", parameters.angle, 1);

    closest_count = 0;
    active = false;

    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }

}

void Deadman::activeCallback(const std_msgs::Bool::ConstPtr &data)
{
    //Sets if the obstacle are active or not.
    active = data->data;
}

void Deadman::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
    //Transform laser scan to point cloud in the scanners own frame
    sensor_msgs::PointCloud2 pointCloud;
    projector_.transformLaserScanToPointCloud(parameters.laserscan_frame, *data, pointCloud, tfListener_);
    int temp_closest_index_count = 0;

    //Finds the index that are closest to an obstacle
    int remove_index = (270 - parameters.angle) / 2; //Finds the index to remove between in order to obtain the angle
    for(int i = remove_index; i < data->ranges.size() - remove_index - 1; i++) //270 - 48  //By looping from 25 to (270 - 24) we only take the points into account that lies in front of the robot.
    {
        //Skip empty measurements.
        if (data->ranges[i] == 0.0)
            continue;

        if(data->ranges[i] < parameters.max_distance)
            temp_closest_index_count++;
    }

    closest_count = temp_closest_index_count;

    //ROS_DEBUG("TOO CLOSE POINTS: %d CLOSEST: %f ", closest_index_count, closest);
    pcl::fromROSMsg(pointCloud, laserScanCloud);
}

bool Deadman::obstacleDetection()
{
    if(!active) //If the obstacle detection is not active, then just return true as we dont want to stop if anything comes in front.
        return true;

    if(closest_count >= parameters.max_points) //Checks if too many points are too close to the robot. If so, then force a deadman.
        return false;

    return true;
}

void Deadman::spinItDJ()
{
    ros::Rate r(30);
    while (ros::ok())
    {
        bool deadman_var = obstacleDetection(); //Checks if anything is in front of the robot

        //Publishes the deadman
        std_msgs::Bool deadman = std_msgs::Bool();
        deadman.data = deadman_var;
        deadman_pub.publish(deadman);

        //ROS_DEBUG("Deadman: %d", deadman_var);

        ros::spinOnce();
        r.sleep();
     }
}
