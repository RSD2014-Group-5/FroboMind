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

    local_nh.param<std::string>("waypointreached_pub", parameters.waypointreached_pub, "/fmKnowledge/waypoint_reached");
    waypointreached_pub = n.advertise<std_msgs::Bool>(parameters.waypointreached_pub, 10);

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
    local_nh.param<int>("min_closepoints", parameters.min_closepoints, 5);
    local_nh.param<double>("clearance_distance", parameters.clearance_distance, 0.5);



    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }

}

bool pose_received = false;
void WayPoint::odometryCallback(const nav_msgs::Odometry::ConstPtr &data)
{
    //Only for testing when using kaspers node to publish waypoints
    if(pose_received == false)
    {
        //Inform the other nodes that the waypoint have been reached.
        std_msgs::Bool waypoint = std_msgs::Bool();
        waypoint.data = true;
        waypointreached_pub.publish(waypoint);
    }

    this->odometryPose = data->pose;
    pose_received = true;
}

void WayPoint::waypointCallback(const geometry_msgs::Point::ConstPtr &data)
{
    geometry_msgs::Point waypoint;
    waypoint.x = data->x;
    waypoint.y = data->y;
    waypoint.z = data->z;

    waypoints.push_back(waypoint);
    ROS_DEBUG("Waypoint recieved: %f %f", waypoint.x, waypoint.y);
}

int closest_index = 0;
int closest_index_count = 0;
double closest_distance = 0;
void WayPoint::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
    //	Transform laser scan to point cloud in the scanners own frame
    sensor_msgs::PointCloud2 pointCloud;
    projector_.transformLaserScanToPointCloud(parameters.laserscan_frame, *data, pointCloud, tfListener_);
    int temp_closest_index_count = 0;

    //Finds the index that are closest to an obstacle
    double closest = 99999;
    for(int i = 25; i < data->ranges.size() - 24; i++) //270 - 48  //By looping from 25 to (270 - 24) we only take the points into account that lies in front of the robot.
    {
        //Skip empty measurements.
        if(data->ranges[i] == 0.0)
            continue;

        if(data->ranges[i] < parameters.max_distance_obstacle)
            temp_closest_index_count++;

        if(closest > data->ranges[i])
        {
            closest = data->ranges[i];
            closest_index = i;
        }
    }

    closest_index_count = temp_closest_index_count;
    closest_distance = closest;

    //ROS_DEBUG("TOO CLOSE POINTS: %d CLOSEST: %f ", closest_index_count, closest);
    pcl::fromROSMsg(pointCloud, laserScanCloud);
}


double lastErrObstacleAngle = 0;
double lastErrObstacleDistance = 0;
double moved_since_90turn = 0;
bool move_forward = false;
double x_start = 0;
double y_start = 0;
bool obstacle_lasttry = false;

bool WayPoint::obstacleDetection()
{
    double max_turn = 0.3;

    //Publish deadman status depending on the distance to the obstacle.
    std_msgs::Bool deadman = std_msgs::Bool();
    deadman.data = true;
    deadman_pub.publish(deadman);

    double errorObstacleAngle = 1;
    int angle;
    if(parameters.min_closepoints < closest_index_count)
    {
        obstacle_lasttry = true;
        //The laser range scanner has a angle increment of 1deg.
        angle = 135 - closest_index;
        //ROS_DEBUG("Closest point: %f Max allowed: %f", closest_distance, parameters.max_distance_obstacle);
        //ROS_DEBUG("Angle: %d", angle);

        //Create some simple PD controller that avoid the obstacle.
        double errorObstacleDistance = parameters.max_distance_obstacle - closest_distance;
        if(errorObstacleDistance < 0) //If the closest distance is bigger than the maximum allowed, then do nothing!
            errorObstacleDistance = 0;

        double EdObstacleDistance = errorObstacleDistance - lastErrObstacleDistance;
        lastErrObstacleDistance = errorObstacleDistance;
        double outputObstacleDistance = (2 * errorObstacleDistance);
        //ROS_DEBUG("Error distance: %f Output: %f Distance: %f", errorObstacleDistance, outputObstacleDistance, closest_distance);

        //Make sure to have a 90deg angle on the obstacle. That way we can drive around it.
        if(angle < 0)
        {
            errorObstacleAngle = fabs(angle) - 90;
            if(errorObstacleAngle > 0) //Ignore if the angle is bigger than 90 and the error become in opposite sign of the desired.
                errorObstacleAngle = 0;
        }
        else
        {
            errorObstacleAngle = 90 - fabs(angle);
            if(errorObstacleAngle < 0) //Ignore if the angle is bigger than 90 and the error become in opposite sign of the desired.
                errorObstacleAngle = 0;
        }

        //PD controlling of the angle
        double EdObstacleAngle = errorObstacleAngle - lastErrObstacleAngle;
        lastErrObstacleAngle = errorObstacleAngle;
        double outputObstacleAngle = (parameters.kp_obstacle * errorObstacleAngle) + (parameters.kd_obstacle * EdObstacleAngle);

        //Count the distance into the amount of angles to turn so turn harder if the robot is too close to the obstacle.
        /*if(outputObstacleAngle > 0)
            outputObstacleAngle += outputObstacleDistance * 4;
        else
            outputObstacleAngle -= outputObstacleDistance * 4;*/

        //Move forward if the angle is bigger than the 90deg
        if(errorObstacleAngle == 0)
        {
            //Sets the current X and Y from where the robot started to move forward so the distance it has travelled can be measured.
            if(!move_forward)
            {
                x_start = this->odometryPose.pose.position.x;
                y_start = this->odometryPose.pose.position.y;
                move_forward = true;
            }
        } else {
            ROS_DEBUG("Angle to obstacle: %d Error: %f Output: %f", angle, errorObstacleAngle, outputObstacleAngle);

            //Resets the move forward variables.
            move_forward = false;

            //Limits the turning speed
            if(outputObstacleAngle > max_turn)
                outputObstacleAngle = max_turn;
            else if(fabs(outputObstacleAngle) > max_turn)
                outputObstacleAngle = -max_turn;

            //Send out velocity commands that avoid  the obstacle.
            geometry_msgs::TwistStamped cmd_velocity;
            cmd_velocity.header.stamp = ros::Time::now();
            cmd_velocity.twist.angular.z = outputObstacleAngle; //angle; // * -1; //Negative = turning right, Positive = turning left
            cmd_velocity.twist.linear.x = 0; //parameters.speed_obstacle; //1 - outputObstacleAngle;

            velocity_pub.publish(cmd_velocity);
            //Add a safety that stops if something comes way too close!
            //deadman.data = false;
        }
    }
    else if(!move_forward)
    {
        if(obstacle_lasttry == true)
        {
            ROS_DEBUG("An obstacle was discovered in the last scan. Moving a bit forward to get clearance!");
            x_start = this->odometryPose.pose.position.x;
            y_start = this->odometryPose.pose.position.y;
            move_forward = true;

            //If an obstacle was detected in the last try, then move forward just a bit to not "oscillate" between the states of no obstacle and obstacle
            move_forward = true;
            obstacle_lasttry = false;
        }
        else
        {
            //This and the last movement did not detect any obstacles! Move forward!
            lastErrObstacleAngle = 0;
            lastErrObstacleDistance = 0;
            deadman.data = true;

            return false;
        }
    }

    if(move_forward)
    {
        //Moveforward to get a clearance from the obstacle.

        //Calculate the distance we have already moved.
        double x_cur = this->odometryPose.pose.position.x;
        double y_cur = this->odometryPose.pose.position.y;
        double distance = sqrt(pow(x_cur - x_start, 2) + pow(y_cur - y_start, 2));

        //Send out velocity commands that avoid  the obstacle.
        geometry_msgs::TwistStamped cmd_velocity;
        cmd_velocity.header.stamp = ros::Time::now();
        cmd_velocity.twist.angular.z = 0; //angle; // * -1; //Negative = turning right, Positive = turning left
        cmd_velocity.twist.linear.x = parameters.speed_obstacle; //1 - outputObstacleAngle;

        velocity_pub.publish(cmd_velocity);

        if(distance > parameters.clearance_distance)
            move_forward = false;

        //ROS_DEBUG("DISTANCE MOVED TO GET CLEARANCE: %f Angle: %d Error output: %f Points: %d Closest: %d", distance, angle, errorObstacleAngle, closest_index_count, closest_index);
    }

    return true;
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
    ros::Rate r(30);
    while (ros::ok())
    {
        //Only for testing with static waypoints
        /*if(pose_received == true && this->waypoints.size() == 0)
        {
            //Adds test waypoints.
            geometry_msgs::Point waypoint;
            waypoint.y = this->odometryPose.pose.position.y - 1.5;
            waypoint.x = this->odometryPose.pose.position.x + 1.0299;
            waypoint.z = 0;
            this->waypoints.push_back(waypoint);

            waypoint.y = this->odometryPose.pose.position.y;
            waypoint.x = this->odometryPose.pose.position.x;
            waypoint.z = 0;
            this->waypoints.push_back(waypoint);
        }*/

        //this->visualizeWaypoint(waypoint);
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
    double max_forwardspeed = 1;

    /*Compute all the working error variables*/
    if(this->waypoints.size() <= 0 || pose_received == false)
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
    double currentAngle = ((tf::getYaw(odometryPose.pose.orientation) * 180) / M_PI) + 0.0001; // Adds a very small value in the angle in order to handle the rare case of a robot angle of 0degrees

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

    if(obstacleDetection()) //If an obstacle is too close, then return;
        return;

    //If the angle output are high, it might be necessary to turn on the spot, as turning slowly might increase the probability of hitting and obstacle in a crowded environment.
    if(fabs(errorAngle) > parameters.max_turn_output)
    {
        //Find the shortest way to turn in order to reach the desirec angle.
        double turnRight = fabs(errorAngle);
        double turnLeft = fabs(errorAngle);
        double diffTo360 = fabs(360 - robotRealAngle); //Difference from current angle of robot to 0degress
        double diffTo0 = fabs(0 - robotRealAngle); //Difference from current robot angle to 360degrees.

        //If the angle of the robot is bigger than waypoint angle, the robot first has to turn to 360 if turning to the left.
        if(robotRealAngle > waypointRealAngle)
        {
            turnLeft = waypointRealAngle;
            turnLeft += diffTo360;
        }

        //If the robot angle is smaller than the waypoint angle it first has to turn to 0 if turning right.
        if(robotRealAngle < waypointRealAngle)
        {
            turnRight = 360 - waypointRealAngle;
            turnRight += diffTo0;
        }

        //ROS_DEBUG("Diff to 0: %f  Dif to 360: %f", diffTo0, diffTo360);

        //Determines if we are going to turn right or left to reach the waypoint in the shortest turn.
        if(turnLeft < turnRight)
        {
            outputAngle = fabs(outputAngle); //Turn left
            if(fabs(outputAngle) > 0.5)  //Limits the speed
                outputAngle = 0.5;
        }
        else
        {
            outputAngle = fabs(outputAngle) * -1; //Turn right
            if(fabs(outputAngle) > 0.5) //Limits the speed
                outputAngle = -0.5;
        }

        ROS_DEBUG("Angle to waypoint: %f Current Angle: %f Error: %f To turn: %f TurnRight: %f TurnLeft: %f", waypointRealAngle, robotRealAngle, errorAngle, outputAngle, turnRight, turnLeft);
        outputDistance = 0; //Dont drive forward when turning.
    }

    //Limits forward speed.
    if(outputDistance > max_forwardspeed)
        outputDistance = max_forwardspeed;


    //TODO: CHECK IF WAYPOINT IS IN SIGHT OR IN COLLISION.
    //Check if the waypoint is reached.
    if(parameters.waypoint_reached_threshold > errorDistance)
    {
        ROS_DEBUG("Waypoint reached at %f, %f Robot(%f, %f)", wayPointX, wayPointY, curX, curY);

        //Inform the other nodes that the waypoint have been reached.
        std_msgs::Bool waypoint = std_msgs::Bool();
        waypoint.data = true;
        waypointreached_pub.publish(waypoint);

        //Removes the waypoint.
        this->waypoints.erase(waypoints.begin());
        return;
    }

    //ROS_DEBUG("Distance from goal: %f", errorDistance);

    //Publish motor control message that moves the robot towards the waypoint
    //ROS_DEBUG("Motor commands!! Turn: %f  Forward: %f", outputAngle, outputDistance);
    geometry_msgs::TwistStamped cmd_velocity;
    cmd_velocity.header.stamp = ros::Time::now();
    cmd_velocity.twist.angular.z = outputAngle; //angle; // * -1; //Negative = turning right, Positive = turning left
    cmd_velocity.twist.linear.x = outputDistance;

    velocity_pub.publish(cmd_velocity);
}


