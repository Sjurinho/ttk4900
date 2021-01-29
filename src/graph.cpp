#include "graph.hpp"

#include <pcl/common/transforms.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

//constructor method
Graph::Graph(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{   
    nh_ = nh;
    ROS_INFO("Initializing Graph Node");

    //Subscribers and publishers
    subOdometry = nh.subscribe<nav_msgs::Odometry>("/lidarOdom", 32, &Graph::odometryHandler, this);
    subMap = nh.subscribe<sensor_msgs::PointCloud2>("/featurePointCloud", 32, &Graph::mapHandler, this);
    pubTransformedMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    pubTransformedPose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
    
    //Initializing and allocation of memory
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;;
    isam = new gtsam::ISAM2(parameters);
    cloudKeyPositions.reset(new pcl::PointCloud<pcl::PointXYZ>());
    currentCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloudKeyPoses.reset(new pcl::PointCloud<PointXYZRPY>());

}

// Destructor method
Graph::~Graph()
{

}

void Graph::_incrementPosition()
{   
    gtsam::Vector3 rotVec(disp[0], disp[1], disp[2]);
    gtsam::Point3 trans(-disp[3], -disp[4], -disp[5]);
    gtsam::Rot3 oriLocal = gtsam::Rot3::RzRyRx(rotVec);
    gtsam::Pose3 localPose(oriLocal, trans);

    //localPose.compose(poseInWorld);
    poseInWorld = poseInWorld * localPose;
    currentPosPoint = pcl::PointXYZ(poseInWorld.translation().x(), poseInWorld.translation().y(), poseInWorld.translation().z());
}

void Graph::_performIsam()
{
    currentPosPoint.x = disp[3];
    currentPosPoint.y = disp[4];
    currentPosPoint.z = disp[5];

    bool saveThisKeyFrame = true;
    if (sqrt((previousPosPoint.x-previousPosPoint.x)*(previousPosPoint.x-currentPosPoint.x)
            +(previousPosPoint.y-currentPosPoint.y)*(previousPosPoint.y-currentPosPoint.y)
            +(previousPosPoint.z-currentPosPoint.z)*(previousPosPoint.z-currentPosPoint.z)) < 0.3){
        saveThisKeyFrame = false;
    }

    if (saveThisKeyFrame == false && !cloudKeyPositions->points.empty())
        	return;

    previousPosPoint = currentPosPoint;

    if (cloudKeyPositions->points.empty()){
        // #TODO: INITIALIZE GRAPH
        //_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Rot3::RzRyRx()))
    }
    else{
        // #TODO: ADD BETWEEN
    }

    // #TODO: ADD TO ISAM
}

void Graph::odometryHandler(const nav_msgs::OdometryConstPtr &odomMsg)
{
    timeOdometry = odomMsg->header.stamp.toSec();
    double r, p, y;
    geometry_msgs::Quaternion geoQuat = odomMsg->pose.pose.orientation;
    //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(r, p, y);

    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(r, p, y);

    disp[3] = odomMsg->pose.pose.position.x;
    disp[4] = odomMsg->pose.pose.position.y;
    disp[5] = odomMsg->pose.pose.position.z;
    disp[0] = r;
    disp[1] = p;
    disp[2] = y;
    newLaserOdometry=true;
}

void Graph::mapHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{
    timeMap = pointCloud2Msg->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloud2Msg, *currentCloud);
    newMap = true;
}

void Graph::runOnce()
{
    if (newLaserOdometry && newMap){
        std::lock_guard<std::mutex> lock(mtx);
        newLaserOdometry, newMap = false;

        _incrementPosition();
        _publishTransformed();

    }
}

// FOR VISUALIZATION
void Graph::_publishTransformed(){
    if (pubTransformedMap.getNumSubscribers() > 0){
        pcl::PointCloud<pcl::PointXYZ> currentInWorld;
        pcl::transformPointCloud(*currentCloud, currentInWorld, poseInWorld.matrix());
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(currentInWorld, msg);
        msg.header.frame_id = "map";
        pubTransformedMap.publish(msg);
    }
    if (pubTransformedPose.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x   = poseInWorld.x();
        pose.pose.position.y   = poseInWorld.y();
        pose.pose.position.z   = poseInWorld.z();
        pose.pose.orientation.w  = poseInWorld.rotation().toQuaternion().w();
        pose.pose.orientation.x  = poseInWorld.rotation().toQuaternion().x();
        pose.pose.orientation.y  = poseInWorld.rotation().toQuaternion().y();
        pose.pose.orientation.z  = poseInWorld.rotation().toQuaternion().z();

        pubTransformedPose.publish(pose);

    }
}