#include "waypoint.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "WayPoint");

    WayPoint findLinesNode;
    findLinesNode.spinItDJ();

    return 0;
}

WayPoint::WayPoint()
{
    ros::NodeHandle n;
    ros::NodeHandle local_nh = ros::NodeHandle("~");

    marker_pub = n.advertise<visualization_msgs::Marker>("waypoint_visualization", 10);

    local_nh.param<std::string>("cmd_vel_pub", parameters.cmd_vel_pub, "/fmCommand/cmd_vel");
    velocity_pub = n.advertise<geometry_msgs::TwistStamped>(parameters.cmd_vel_pub, 10);

    local_nh.param<std::string>("deadman_pub", parameters.deadman_pub, "/fmCommand/deadman");
    deadman_pub = n.advertise<std_msgs::Bool>(parameters.deadman_pub, 10);

    local_nh.param<std::string>("waypoint_sub", parameters.waypoint_sub, "/fmCommand/waypoint");
    waypoint_sub = n.subscribe<geometry_msgs::Point>(parameters.waypoint_sub, 10, &WayPoint::waypointCallback, this);

    local_nh.param<std::string>("odometry_sub", parameters.odometry_sub, "/odom");
    odometry_sub = n.subscribe<nav_msgs::Odometry>(parameters.odometry_sub, 10, &WayPoint::odometryCallback, this);

    local_nh.param<std::string>("laserscan_frame", parameters.laserscan_frame, "/laser");
    local_nh.param<std::string>("laserscan_sub", parameters.laserscan_sub, "/LaserScanner/scan");
    laserScan = n.subscribe<sensor_msgs::LaserScan>(parameters.laserscan_sub, 10, &WayPoint::laserScanCallback, this);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));


    /* Loads parameters from launch file */
    local_nh.param<double>("kp_angle", parameters.kp_angle, 1);
    local_nh.param<double>("ki_angle", parameters.ki_angle, 1);
    local_nh.param<double>("kd_angle", parameters.kd_angle, 1);

    local_nh.param<double>("kp_distance", parameters.kp_distance, 1);
    local_nh.param<double>("ki_distance", parameters.ki_distance, 1);
    local_nh.param<double>("kd_distance", parameters.kd_distance, 1);


    local_nh.param<double>("kp_obstacle", parameters.kp_obstacle, 0.2);
    local_nh.param<double>("kd_obstacle", parameters.kd_obstacle, 0.1);
    local_nh.param<double>("speed_obstacle", parameters.speed_obstacle, 0.2);

    local_nh.param<double>("max_turn_output", parameters.max_turn_output, 1);
    local_nh.param<double>("max_distance_obstacle", parameters.max_distance_obstacle, 1);
    local_nh.param<double>("waypoint_reached_threshold", parameters.waypoint_reached_threshold, 0.1);

    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }

}

void WayPoint::odometryCallback(const nav_msgs::Odometry::ConstPtr &data)
{
    this->odometryPose = data->pose;
}

void WayPoint::waypointCallback(const geometry_msgs::Point::ConstPtr &data)
{
    geometry_msgs::Point waypoint;
    waypoint.x = data->x;
    waypoint.y = data->y;
    waypoint.z = data->z;

    waypoints.push_back(waypoint);
}

int closest_index = 0;
double closest_distance = 0;
void WayPoint::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
    //	Transform laser scan to point cloud in the scanners own frame
    sensor_msgs::PointCloud2 pointCloud;
    projector_.transformLaserScanToPointCloud(parameters.laserscan_frame, *data, pointCloud, tfListener_);

    //Finds the index that are closest to an obstacle
    double closest = 99999;
    for(int i = 45; i < data->ranges.size() - 44; i++) //By looping from 45 to (270 - 45) we only take the points into account that lies in front of the robot.
    {
        if(closest > data->ranges[i])
        {
            closest = data->ranges[i];
            closest_index = i;
        }
    }

    closest_distance = closest;

    //ROS_DEBUG("CLOSEST INDEX: %d", closest_index);
    pcl::fromROSMsg(pointCloud, laserScanCloud);
}


double lastErrObstacleAngle = 0;
double lastErrObstacleDistance = 0;
bool WayPoint::obstacleDetection()
{
   /* double closest_distance = 999999;

    //Finds the closest obstacle. (This might cause problem when it is running outside simulation as the LIDAR detecs stuff on the robot itself)
    for(int i = 0; i < this->laserScanCloud.size(); i++)
    {
        PointT point = laserScanCloud[i];
        //This distance calculation does not count in the offset of the lidar position on the robot.
        double distance = sqrt( pow(point.x, 2) + pow(point.y, 2) );
        if(closest_distance > distance)
        {
            closest_distance = distance;
        }
    }*/


    //Publish deadman status depending on the distance to the obstacle.
    bool ret = false;
    std_msgs::Bool deadman = std_msgs::Bool();
    if(closest_distance < parameters.max_distance_obstacle)
    {
        //The laser range scanner has a angle increment of 1deg.
        int angle = 135 - closest_index;
        //ROS_DEBUG("Closest point: %f Max allowed: %f", closest_distance, parameters.max_distance_obstacle);
        //ROS_DEBUG("Angle: %d", angle);

        //Create some simple PD controller that avoid the obstacle.
        double errorObstacleDistance = parameters.max_distance_obstacle - closest_distance;
        double EdObstacleDistance = errorObstacleDistance - lastErrObstacleDistance;
        lastErrObstacleDistance = errorObstacleDistance;

        //Make sure to have a 90deg angle on the obstacle. That way we can drive around it.
        double errorObstacleAngle;
        if(angle < 0)
            errorObstacleAngle = fabs(angle) - 90;
        else
            errorObstacleAngle = 90 - fabs(angle);

        ROS_DEBUG("Angle to obstacle: %d Error: %f", angle, errorObstacleAngle);

        double EdObstacleAngle = errorObstacleAngle - lastErrObstacleAngle;
        lastErrObstacleAngle = errorObstacleAngle;

        double outputObstacleAngle = (parameters.kp_obstacle * errorObstacleAngle) + (parameters.kd_obstacle * EdObstacleAngle);
        double outputObstacleDistance = (parameters.kp_obstacle * errorObstacleDistance) + (parameters.kd_obstacle * EdObstacleDistance);

        //Make the robot turn away from the obstacle.
        //if(angle < 0)
        //    outputObstacleDistance = outputObstacleDistance * -1;

        //Send out velocity commands that avoid  the obstacle.
        geometry_msgs::TwistStamped cmd_velocity;
        cmd_velocity.header.stamp = ros::Time::now();
        cmd_velocity.twist.angular.z = outputObstacleAngle; //angle; // * -1; //Negative = turning right, Positive = turning left
        cmd_velocity.twist.linear.x = parameters.speed_obstacle; //1 - outputObstacleAngle;

        velocity_pub.publish(cmd_velocity);
        ret = true;
        deadman.data = true;

        //Add a safety that stops if something comes way too close!
        //deadman.data = false;
    }
    else
    {
        lastErrObstacleAngle = 0;
        lastErrObstacleDistance = 0;
        deadman.data = true;
        ret = false;
    }

    deadman_pub.publish(deadman);
    return ret;
}

void WayPoint::visualizeWaypoint(geometry_msgs::Point waypoint)
{
    visualization_msgs::Marker points;
    points.header.frame_id = parameters.laserscan_frame;
    points.header.stamp = ros::Time::now();
    points.ns = "wapoints";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::SPHERE;

    ros::Duration lifetime(0.5);
    points.lifetime = lifetime;

    points.pose.position.x = waypoint.x;
    points.pose.position.y = waypoint.y;
    points.pose.position.z = 1;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 1.0;
    points.color.b = 0.0;

    marker_pub.publish(points);
}


void WayPoint::spinItDJ()
{
    //Adds test waypoints.
    geometry_msgs::Point waypoint;
    waypoint.x = -3;
    waypoint.y = -2;
    waypoint.z = 0;
    this->waypoints.push_back(waypoint);

    waypoint.x = 1;
    waypoint.y = 2;
    waypoint.z = 0;
    this->waypoints.push_back(waypoint);

    waypoint.x = -1;
    waypoint.y = 2;
    waypoint.z = 0;
    this->waypoints.push_back(waypoint);

    ros::Rate r(30);
    while (ros::ok())
    {
        this->visualizeWaypoint(waypoint);
        this->gotoWaypoint();

        ros::spinOnce();
        r.sleep();
     }
}

unsigned long lastTime = 0;
double Input, Output, Setpoint;
double errSum = 0;
double lastErrAngle = 0;
double lastErrDistance = 0;

void WayPoint::gotoWaypoint()
{
    /*Compute all the working error variables*/
    if(this->waypoints.size() <= 0)
        return;

    //Defines the local variables used for calculations in this function.
    geometry_msgs::Point wayPoint = this->waypoints[0];

    double curX = odometryPose.pose.position.x;
    double curY = odometryPose.pose.position.y;

    double wayPointX = wayPoint.x;
    double wayPointY = wayPoint.y;

    //Calculates the error in the distance
    double currentDistance = sqrt(pow(odometryPose.pose.position.x - wayPoint.x,2) + pow(odometryPose.pose.position.y - wayPoint.y,2));
    double errorDistance = currentDistance;
    double EdDistance = errorDistance - lastErrDistance;
    lastErrDistance = errorDistance;

    double outputDistance = (parameters.kp_distance * errorDistance) + (parameters.kd_distance * EdDistance);

    //Extract the yaw from the quaternion from the odometry.
    double currentAngle = (tf::getYaw(odometryPose.pose.orientation) * 180) / M_PI;

    //Calculates the angle to the point from the robot.
    //http://robotics.stackexchange.com/questions/550/how-to-determine-heading-without-compass
    double a = wayPointX - curX;
    double b = wayPointY - curY;
    double angleToWaypoint = atan2 (b,a) * 180 / M_PI;

    //Calculates the real angle on the robot, as the one determined from the laserscan gives from 0 to 180 and 0 to -180
    double robotRealAngle;
    if(currentAngle < 0)
        robotRealAngle = 180 + (180 - fabs(currentAngle));
    else
        robotRealAngle = currentAngle;
    //Same goes for the waypoint angle
    double waypointRealAngle;
    if(angleToWaypoint < 0)
        waypointRealAngle = 180 + (180 - fabs(angleToWaypoint));
    else
        waypointRealAngle = angleToWaypoint;

    //Calculates the error in the angle to the waypoint and in the robots angle.
    double errorAngle = waypointRealAngle - robotRealAngle;

    //Uses PD controller to correct the error in the angle.
    double EdAngle = errorAngle - lastErrAngle;
    double outputAngle = (parameters.kp_angle * errorAngle) + (parameters.kd_angle * EdAngle);
    lastErrAngle = errorAngle;

    //ROS_DEBUG("Angle to waypoint: %f Current Angle: %f Error: %f Yaw: %f", angleToWaypoint, currentAngle, errorAngle, tf::getYaw(odometryPose.pose.orientation));

    //If the angle output are high, it might be necessary to turn on the spot, as turning slowly might increase the probability of hitting and obstacle in a crowded environment.
    if(fabs(outputAngle) > parameters.max_turn_output)
    {
        //Find the shortest way to turn in order to reach the desirec angle.
        double turnRight = fabs(errorAngle);
        double turnLeft = fabs(errorAngle);
        double diffTo360 = fabs(360 - robotRealAngle); //Difference from current angle of robot to 0degress
        double diffTo0 = fabs(0 - robotRealAngle); //Difference from current robot angle to 360degrees.

        //If the angle of the robot is bigger than waypoint angle, the robot first has to turn to 360 if turning to the left.
        if(robotRealAngle > waypointRealAngle)
            turnLeft += diffTo360;

        //If the robot angle is smaller than the waypoint angle it first has to turn to 0 if turning right.
        if(robotRealAngle < waypointRealAngle)
            turnRight += diffTo0;

        //Determines if we are going to turn right or left to reach the waypoint in the shortest turn.
        if(turnLeft < turnRight)
            outputAngle = fabs(outputAngle); //Turn left
        else
            outputAngle = fabs(outputAngle) * -1; //Turn right

        outputDistance = 0; //Dont drive forward when turning.
    }


    //TODO: CHECK IF WAYPOINT IS IN SIGHT OR IN COLLISION.
    //Check if the waypoint is reached.
    ROS_DEBUG("Distance from goal: %f", errorDistance);
    if(parameters.waypoint_reached_threshold > errorDistance)
    {
        ROS_DEBUG("Waypoint reached at %f, %f Robot(%f, %f)", wayPointX, wayPointY, curX, curY);

        //Removes the waypoint.
        this->waypoints.erase(waypoints.begin());
        return;
    }

    //Publish motor commands.
    if(obstacleDetection()) //If an obstacle is too close, then return;
        return;

    //Test publish a twist control message.
    geometry_msgs::TwistStamped cmd_velocity;
    //Assign the data to the message for the FroBit motorcontroller
    //ROS_DEBUG("Forward: %f  Turn: %f", outputDistance, outputAngle);
    cmd_velocity.header.stamp = ros::Time::now();
    cmd_velocity.twist.angular.z = outputAngle; //angle; // * -1; //Negative = turning right, Positive = turning left
    cmd_velocity.twist.linear.x = outputDistance;


    velocity_pub.publish(cmd_velocity);
}


