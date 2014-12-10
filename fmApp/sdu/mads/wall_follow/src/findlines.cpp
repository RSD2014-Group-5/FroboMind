#include "findlines.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "FindLines");

    FindLines findLinesNode;
    findLinesNode.spinItDJ();

    return 0;
}

FindLines::FindLines()
{
    ros::NodeHandle n;
    ros::NodeHandle local_nh = ros::NodeHandle("~");

    marker_pub = n.advertise<visualization_msgs::Marker>("lines_visualization", 10);

    local_nh.param<std::string>("cmd_vel_pub", parameters.cmd_vel_pub, "/fmCommand/cmd_vel");
    velocity_pub = n.advertise<geometry_msgs::TwistStamped>(parameters.cmd_vel_pub, 10);

    local_nh.param<std::string>("deadman_pub", parameters.deadman_pub, "/fmCommand/deadman");
    deadman_pub = n.advertise<std_msgs::Bool>(parameters.deadman_pub, 10);

    local_nh.param<std::string>("active_sub", parameters.active_sub, "/fmCommand/wallfollow");
    active_sub = n.subscribe<std_msgs::Bool>(parameters.active_sub, 10, &FindLines::activeCallback, this);
    parameters.active = false;

    local_nh.param<std::string>("laserscan_frame", parameters.laserscan_frame, "/laser");
    local_nh.param<std::string>("laserscan_sub", parameters.laserscan_sub, "/LaserScanner/scan");
    laserScan = n.subscribe<sensor_msgs::LaserScan>(parameters.laserscan_sub, 10, &FindLines::laserScanCallback, this);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));


    // Loads parameters from launch file
    local_nh.param<double>("wall_distance", parameters.wall_distance, 0.3);

    local_nh.param<double>("ransac_inliers", parameters.ransac_inliers, 20);
    local_nh.param<double>("ransac_threshold", parameters.ransac_threshold, 0.1);
    local_nh.param<bool>("show_lines", parameters.show_lines, true);

    local_nh.param<double>("forward_velocity", parameters.forward_velocity, 0.0);
    local_nh.param<double>("turn_velocity", parameters.turn_velocity, 0.1);
    local_nh.param<double>("turn_factor", parameters.turn_factor, 1);

    local_nh.param<double>("lidar_offset_x", parameters.lidar_offset_x, 0.21);
    local_nh.param<double>("lidar_offset_y", parameters.lidar_offset_y, 0.1);

    local_nh.param<double>("kp", parameters.kp, 100);
    local_nh.param<double>("ki", parameters.ki, 1);
    local_nh.param<double>("kd", parameters.kd, 1);


    //Enables debug logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
}

std::vector<LineStruct> FindLines::ransac(pcl::PointCloud<PointT>::Ptr laserRangePointCloud, int min_inliers, double threshold)
{
    std::vector<LineStruct> lines;

    if(laserRangePointCloud->points.size() <= 0)
        return lines;

    // initialize PointClouds
    pcl::PointCloud<PointT>::Ptr cloud = laserRangePointCloud;
    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

    /*std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelLine<PointT>::Ptr model_p (new pcl::SampleConsensusModelLine<PointT> (cloud));

    pcl::RandomSampleConsensus<PointT> ransac (model_p);
    ransac.setDistanceThreshold (threshold);
    ransac.computeModel();
    ransac.getInliers(inliers);

    if(min_inliers > inliers.size())
        return;
    */

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (threshold);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    int marker_id = 0;

    while(1)
    {
        //Apply ransac.
        //ROS_DEBUG("CLOUD SIZE: %d", cloud->points.size());
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        //If not enough inliers were found, then break the loop.
        if(inliers->indices.size() < min_inliers || cloud->points.size() <= parameters.ransac_inliers)
            break;

        //ROS_DEBUG("Inliers %d", inliers->indices.size());
        //ROS_DEBUG("We are %d", cloud->points.size());

        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = parameters.laserscan_frame;
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        ros::Duration lifetime(0.01);
        points.lifetime = line_strip.lifetime = line_list.lifetime = lifetime;

        marker_id++;
        line_list.id = marker_id;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.1;
        points.scale.y = 0.1;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        if(marker_id == 1)
        {
            line_list.color.a = 1.0;
            line_list.color.b = 1.0;
            line_list.color.g = 1.0;
        } else if(marker_id == 2)
        {
            line_list.color.a = 1.0;
            line_list.color.g = 1.0;
        }else if(marker_id == 3)
        {
            line_list.color.a = 1.0;
            line_list.color.b = 1.0;
        }else if(marker_id == 4)
        {
            line_list.color.r = 1.0;
            line_list.color.b = 1.0;
            line_list.color.a = 1.0;
        }else if(marker_id == 5)
        {
            line_list.color.r = 1.0;
            line_list.color.g = 1.0;
            line_list.color.a = 1.0;
        }else if(marker_id == 6)
        {
            line_list.color.b = 1.0;
            line_list.color.g = 1.0;
            line_list.color.a = 1.0;
        }else if(marker_id == 7)
        {

        }

        //Creates linear line that indicates the line found by ransac
        PointT first = cloud->points[inliers->indices[0]];
        PointT last = cloud->points[inliers->indices[inliers->indices.size()-1]];
        double a = (double)((double)last.y - (double)first.y) / (double)((double)last.x - (double)first.x);
        double b = (double)first.y - (a*(double)first.x);
        double c = -((a * (double)first.x) + (b * (double)first.y));

        //TODO PARAMETRIC REPRESENTATION OF LINE

        //Finds the closest inlier distance from robot.
        double d_cur = 99;
        for(int j = 0; j < inliers->indices.size(); j++)
        {
            PointT closest = cloud->points[inliers->indices[j]];
            double distance = 0;

            //Adds the offset of the LIDAR. As we wants the distance from the robot and not from the LIDAR.
            if(b < 0)
                distance = sqrt( pow(closest.x + parameters.lidar_offset_x - 0, 2) + pow(closest.y + parameters.lidar_offset_y - 0, 2) );
            else
                distance = sqrt( pow(closest.x + parameters.lidar_offset_x - 0, 2) + pow(closest.y - parameters.lidar_offset_y - 0, 2) );

            if(distance < d_cur) d_cur = distance;

        }

        //Creates a object from the given line parameters and adds it to the return list.
        LineStruct lineStruct;
        lineStruct.a = a;
        lineStruct.b = b;
        lineStruct.c = c;
        lineStruct.distance = d_cur;

        lines.push_back(lineStruct);

        ROS_DEBUG("A: %f B: %f C: %f D: %f", a,b,c, d_cur);

        //Draws the line that was just found for visualization in RViZ
        std::vector<double> vector_representation;
        vector_representation.push_back((double)last.x - (double)first.x);
        vector_representation.push_back((double)last.y - (double)first.y);

        if(parameters.show_lines)
        {
            geometry_msgs::Point p_start;
            p_start.y = a*first.x + b;
            p_start.z = 0;
            p_start.x = first.x;
            line_list.points.push_back(p_start);

            geometry_msgs::Point p_end;
            p_end.y = a*last.x + b;
            p_end.z = 0;
            p_end.x = last.x;//;
            line_list.points.push_back(p_end);

            marker_pub.publish(line_list);
        }
        //Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);

    }

    //Sorts the vector of lines in distance, with lowest distance first.
    std::sort(lines.begin(), lines.end());

    for(int i = 0; i < lines.size(); i++)
        ROS_DEBUG("Distance %d - %f", i, lines.at(i).distance);

    return lines;

    /*visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_laser_link";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 92;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    //Determine which side on the robot the wall is on. Calculates the X and determine if its negativ or positive.
    double y = 0;
    double x = (y - vector_representation[1]) / vector_representation[0];

    geometry_msgs::Point p;
    p.x = x;
    p.y = 0;
    p.z = 0; //laserScanCloud.points[inliers[i]].z;

    points.points.push_back(p);
    //line_strip.points.push_back(p);

    // The line list needs two points for each line

    marker_pub.publish(points);
    // Create the vertices for the points and lines
    for (int i = 0; i < cloud->size(); i++)
    {
      geometry_msgs::Point p;
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = 0; //laserScanCloud.points[inliers[i]].z;

      points.points.push_back(p);
      //line_strip.points.push_back(p);

      // The line list needs two points for each line

      marker_pub.publish(points);

    }*/
}


unsigned long lastTime = 0;
double Input, Output, Setpoint;
double errSum = 0;
double lastErr = 0;
double kp, ki, kd;

void FindLines::followWall(std::vector<LineStruct> lines, double desiredDistance)
{
    /* The parameter vector should be like as the following: 0 = a, 1 = b, 2 = c
     * This function will always follow the line that is closest to the robot
     */

    LineStruct closestLine = lines[0];

    double a = closestLine.a;
    double b = closestLine.b;
    double c = closestLine.c;
    double currentDistance = closestLine.distance;

    //Calculates the error.
    /*How long since we last calculated*/
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int now = tp.tv_sec * 1000 + tp.tv_usec / 1000;

    if(lastTime == 0)
        lastTime = now;

    double timeChange = (double)(now - lastTime);

    /*Compute all the working error variables*/
    double error = desiredDistance - currentDistance;
    ROS_DEBUG("Desired: %f Current: %f Error: %f", desiredDistance, currentDistance, error);

    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;
    double Ed = error - lastErr;
    /*Compute PID Output*/
    //Output = (parameters.kp * error) + (parameters.ki * errSum); // + (parameters.kd * dErr);
    Output = (parameters.kp * error) + (parameters.kd * Ed) + (parameters.ki * errSum);

    /*if(b < 0)
        Output = Output * -1;*/

    //Determines speed and angle to turn.
    double velocity, angle;
    angle = -Output;

    //Determines if we are turning, then slow down.
    if(fabs(Output) > parameters.turn_factor)
        velocity = parameters.turn_velocity;
    else
        velocity = parameters.forward_velocity;

    ROS_DEBUG("Vel: %f Output: %f", velocity, Output);
    //Checks if a wall in front of the robot is coming closer.
    /*if(lines.size() > 1)
    {
        LineStruct secondClosest = lines[1];

        double threshold = 0.1;
        if(secondClosest.distance - threshold < parameters.wall_distance)
        {
            velocity = 0.0;
            angle = -0.8;
            ROS_DEBUG("OUTPUT %f", angle);
        }

    }*/
    //ROS_DEBUG("Error: %f D-Error: %f ErrorSum: %f Output: %f", error, dErr, errSum, Output);

    //Remember some variables for next time
    lastErr = error;
    lastTime = now;

    //Publish deadman message
    std_msgs::Bool deadman = std_msgs::Bool();
    deadman.data = true;
    deadman_pub.publish(deadman);

    //Test publish a twist control message.
    geometry_msgs::TwistStamped cmd_velocity;
    //Assign the data to the message for the FroBit motorcontroller
    cmd_velocity.header.stamp = ros::Time::now();
    cmd_velocity.twist.angular.z = angle; //angle; // * -1; //Negative = turning right, Positive = turning left
    cmd_velocity.twist.linear.x = velocity;

    velocity_pub.publish(cmd_velocity);
}

void FindLines::spinItDJ()
{
    ros::Rate r(30);
    while (ros::ok())
    {
        if(parameters.active) //Only go into the loop of the wall following is active
        {
            std::vector<LineStruct> lines = ransac(laserScanCloud.makeShared(), parameters.ransac_inliers, parameters.ransac_threshold);

            //Call the line following algorithm if a line was represented.
            if(lines.size() > 0)
                followWall(lines, parameters.wall_distance);
        }

        ros::spinOnce();
        r.sleep();
     }
}

void FindLines::activeCallback(const std_msgs::Bool::ConstPtr& data)
{
    parameters.active = data->data;
    ROS_DEBUG("CALLBACK ACTIVE RECEIEVED! FOLLOWING LINES: %d", parameters.active);
}

void FindLines::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
    //	Transform laser scan to point cloud in the scanners own frame
    sensor_msgs::PointCloud2 pointCloud;
    projector_.transformLaserScanToPointCloud(parameters.laserscan_frame, *data, pointCloud, tfListener_);

    pcl::fromROSMsg(pointCloud, laserScanCloud);

    //ROS_DEBUG("Laser callback");
}
