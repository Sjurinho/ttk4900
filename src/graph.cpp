#include "graph.hpp"

#include <pcl/common/transforms.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <opencv/cv.h>

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
    cloudKeyFramesMap.reset(new pcl::PointCloud<pcl::PointXYZ>());


}

// Destructor method
Graph::~Graph()
{

}

void Graph::_incrementPosition()
{   
    gtsam::Vector3 rotVec(disp[0], disp[1], disp[2]);
    gtsam::Point3 trans(disp[3], disp[4], disp[5]);
    gtsam::Rot3 oriLocal = gtsam::Rot3::RzRyRx(rotVec);
    displacement = gtsam::Pose3(oriLocal, trans);

    //displacement.compose(poseInWorld);
    currentPoseInWorld = currentPoseInWorld * displacement;
    currentPosPoint = pcl::PointXYZ(currentPoseInWorld.x(), currentPoseInWorld.y(), currentPoseInWorld.z());
}

void Graph::_performIsam()
{
    bool saveThisKeyFrame = true;

    double squaredDistance = 
    (previousPosPoint.x-currentPosPoint.x)*(previousPosPoint.x-currentPosPoint.x)
    +(previousPosPoint.y-currentPosPoint.y)*(previousPosPoint.y-currentPosPoint.y)
    +(previousPosPoint.z-currentPosPoint.z)*(previousPosPoint.z-currentPosPoint.z);

    if (sqrt(squaredDistance) < 0.3){
        saveThisKeyFrame = false;
    }

    if (saveThisKeyFrame == false && !cloudKeyPositions->points.empty())
        	return;

    ROS_INFO("SAVING NEW KEY FRAME");
    previousPosPoint = currentPosPoint;

    if (cloudKeyPositions->points.empty()){
        // #TODO: INITIALIZE GRAPH
        _graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, displacement, priorNoise));
        initialEstimate.insert(0, displacement);
        //lastPoseInWorld = currentPoseInWorld;
    }
    else{
        _graph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPositions->points.size()-1, cloudKeyPositions->points.size(), lastPoseInWorld.between(currentPoseInWorld), odometryNoise));
        initialEstimate.insert(cloudKeyPositions->points.size(), currentPoseInWorld);
        // #TODO: ADD BETWEEN
    }

    PointXYZRPY currentPose;
    // #TODO: ADD TO ISAM
    isam->update(_graph, initialEstimate);
    isam->update();

    _graph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();

    currentPoseInWorld = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size()-1);

    cloudKeyPositions->push_back(pcl::PointXYZ(currentPoseInWorld.x(), currentPoseInWorld.y(), currentPoseInWorld.z()));

    currentPose.x = currentPoseInWorld.x(); currentPose.y = currentPoseInWorld.y(); currentPose.z = currentPoseInWorld.z();
    currentPose.roll = currentPoseInWorld.rotation().roll();
    currentPose.pitch = currentPoseInWorld.rotation().pitch();
    currentPose.yaw = currentPoseInWorld.rotation().yaw();

    cloudKeyPoses->push_back(currentPose);

    lastPoseInWorld = currentPoseInWorld;

    pcl::PointCloud<pcl::PointXYZ>::Ptr thisKeyFrame(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*currentCloud, *thisKeyFrame);
    cloudKeyFrames.push_back(thisKeyFrame);
}

void Graph::_smoothPoses(){
    int smoothingFrames = 10;
    if (cloudKeyFrames.size() < smoothingFrames) return;
    int latestFrame = cloudKeyFrames.size()-1;
    for (int i = 10; i > 0; --i){
        int frameID = latestFrame - i;
        pcl::PointCloud<pcl::PointXYZ> frameFrom = *cloudKeyFrames[frameID];
        pcl::PointCloud<pcl::PointXYZ> frameTo = *cloudKeyFrames[frameID+1];
        PointXYZRPY poseFrom = cloudKeyPoses->at(frameID);
        PointXYZRPY poseTo = cloudKeyPoses->at(frameID+1);


    } 
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
        _performIsam();
        _publishTransformed();

    }
}

// FOR VISUALIZATION
void Graph::_publishTransformed(){
    if (pubTransformedMap.getNumSubscribers() > 0){
        pcl::PointCloud<pcl::PointXYZ> currentInWorld;
        pcl::transformPointCloud(*currentCloud, currentInWorld, currentPoseInWorld.matrix());
        *cloudKeyFramesMap += currentInWorld;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloudKeyFramesMap, msg);
        msg.header.frame_id = "map";
        pubTransformedMap.publish(msg);
    }
    if (pubTransformedPose.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x   = currentPoseInWorld.x();
        pose.pose.position.y   = currentPoseInWorld.y();
        pose.pose.position.z   = currentPoseInWorld.z();
        pose.pose.orientation.w  = currentPoseInWorld.rotation().toQuaternion().w();
        pose.pose.orientation.x  = currentPoseInWorld.rotation().toQuaternion().x();
        pose.pose.orientation.y  = currentPoseInWorld.rotation().toQuaternion().y();
        pose.pose.orientation.z  = currentPoseInWorld.rotation().toQuaternion().z();

        pubTransformedPose.publish(pose);

    }
}