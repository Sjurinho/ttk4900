#include "graph.hpp"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

//constructor method
Graph::Graph(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{   
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    isam                            = new gtsam::ISAM2(parameters);

    nh_ = nh;
    subOdometry = nh.subscribe<nav_msgs::Odometry>("/lidarOdom", 32, &Graph::odometryHandler, this);
    subMap = nh.subscribe<sensor_msgs::PointCloud2>("/map", 32, &Graph::mapHandler, this);

}

// Destructor method
Graph::~Graph()
{

}

void Graph::odometryHandler(const nav_msgs::OdometryConstPtr &odomMsg)
{
    timeOdometry = odomMsg->header.stamp.toSec();
    double r, p, y;
    geometry_msgs::Quaternion geoQuat = odomMsg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(r, p, y);
    disp[0] = odomMsg->pose.pose.position.x;
    disp[1] = odomMsg->pose.pose.position.y;
    disp[2] = odomMsg->pose.pose.position.z;
    disp[3] = r;
    disp[4] = -p;
    disp[5] = -y;
    newLaserOdometry=true;
}

void Graph::mapHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{
    timeMap = pointCloud2Msg->header.stamp.toSec();
    newMap = true;
}

void Graph::runOnce()
{
    if (newLaserOdometry && newMap){
        newLaserOdometry, newMap = false;
        std::lock_guard<std::mutex> lock(mtx);
    }
}