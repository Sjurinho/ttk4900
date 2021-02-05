#include "graph.hpp"

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

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
    pubPoseArray = nh.advertise<geometry_msgs::PoseArray>("/poseArray", 1);
    
    //Initializing and allocation of memory
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
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
        _graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, currentPoseInWorld, priorNoise));
        initialEstimate.insert(0, currentPoseInWorld);
        lastPoseInWorld = currentPoseInWorld;
    }
    else{
        _graph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPositions->points.size()-1, cloudKeyPositions->points.size(), lastPoseInWorld.between(currentPoseInWorld), odometryNoise));
        initialEstimate.insert(cloudKeyPositions->points.size(), currentPoseInWorld);
    }

    PointXYZRPY currentPose;
    isam->update(_graph, initialEstimate);
    isam->update();

    _graph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();

    currentPoseInWorld = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size()-1);

    cloudKeyPositions->push_back(pcl::PointXYZ(currentPoseInWorld.x(), currentPoseInWorld.y(), currentPoseInWorld.z()));

    currentPose.x = currentPoseInWorld.translation().x(); currentPose.y = currentPoseInWorld.translation().y(); currentPose.z = currentPoseInWorld.translation().z();
    currentPose.roll = currentPoseInWorld.rotation().roll();
    currentPose.pitch = currentPoseInWorld.rotation().pitch();
    currentPose.yaw = currentPoseInWorld.rotation().yaw();

    cloudKeyPoses->push_back(currentPose);

    lastPoseInWorld = currentPoseInWorld;

    pcl::PointCloud<pcl::PointXYZ>::Ptr thisKeyFrame(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*currentCloud, *thisKeyFrame);
    cloudKeyFrames.push_back(thisKeyFrame);
}

float Graph::_smoothPoses(){
    auto skewSymmetric = [](float a, float b, float c){return gtsam::skewSymmetric(a, b, c);};
    int smoothingFrames = 10;
    if (cloudKeyFrames.size() < smoothingFrames) return 0;
    int latestFrame = cloudKeyFrames.size();
    int minNrOfPoints = std::numeric_limits<int>::max();
    for (int i = 0; i < smoothingFrames; ++i){
        int frameID = latestFrame - smoothingFrames + i;
        int size = cloudKeyFrames[frameID]->points.size();
        if (size < minNrOfPoints){
            minNrOfPoints = size > 1000 ? 1000 : size;
        }
    }
    int pointD = 3; int poseD = 6;
    int ARows = minNrOfPoints*smoothingFrames*pointD;
    int BRows = ARows;
    int ACols = poseD*smoothingFrames; //+ pointD*minNrOfPoints;
    int XCols = ACols;
    cv::Mat matA = cv::Mat::zeros(ARows, ACols, CV_32FC1);
    cv::Mat matAt(ACols, ARows, CV_32FC1, cv::Scalar::all(0.0));
    cv::Mat matAtA(ACols, ACols, CV_32FC1, cv::Scalar::all(0));
    cv::Mat matB(BRows, 1, CV_32FC1, cv::Scalar::all(0));
    cv::Mat matAtB(ACols, 1, CV_32FC1, cv::Scalar::all(0));
    cv::Mat matX(XCols, 1, CV_32FC1, cv::Scalar::all(0));
    for (int k = 0; k < smoothingFrames; ++k){
        int frameID = latestFrame - smoothingFrames + k;
        pcl::PointCloud<pcl::PointXYZ> framePoints = *cloudKeyFrames[frameID];
        pcl::PointCloud<pcl::PointXYZ> mapLocal;
        PointXYZRPY keyPose = cloudKeyPoses->at(frameID);

        gtsam::Vector3 rotVec(keyPose.roll, keyPose.pitch, keyPose.yaw);
        gtsam::Point3 trans(keyPose.x, keyPose.y, keyPose.z);
        gtsam::Rot3 oriLocal = gtsam::Rot3::RzRyRx(rotVec);
        gtsam::Pose3 poseInWorld(oriLocal, trans);

        pcl::transformPointCloud(*cloudKeyFramesMap, mapLocal, poseInWorld.inverse().matrix()); //Need to use map instead

        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(mapLocal.makeShared());
        std::vector<int> indices;
        std::vector<float> distances;
        for (int j = 0; j < minNrOfPoints; j++){
            auto pointInFrame = framePoints.at(j);

            if (kdTree.nearestKSearch(pointInFrame, 1, indices, distances) > 0) {
                auto x_wjPCL = cloudKeyFramesMap->at(indices[0]);
                auto x_wj = gtsam::Point3(x_wjPCL.x, x_wjPCL.y, x_wjPCL.z);
                auto x_Lj = gtsam::Point3(pointInFrame.x, pointInFrame.y, pointInFrame.z);
                auto R_wLi = poseInWorld.rotation();
                auto t_wi  = poseInWorld.translation();

                /*auto J_hij_xwj = cv::Mat(pointD, pointD, CV_32F, cv::Scalar::all(0));
                J_hij_xwj.at<float>(0, 0) = R_wLi.transpose()(0, 0);
                J_hij_xwj.at<float>(0, 1) = R_wLi.transpose()(0, 1);
                J_hij_xwj.at<float>(0, 2) = R_wLi.transpose()(0, 2);
                J_hij_xwj.at<float>(1, 0) = R_wLi.transpose()(1, 0);
                J_hij_xwj.at<float>(1, 1) = R_wLi.transpose()(1, 1);
                J_hij_xwj.at<float>(1, 2) = R_wLi.transpose()(1, 2);
                J_hij_xwj.at<float>(2, 0) = R_wLi.transpose()(2, 0);
                J_hij_xwj.at<float>(2, 1) = R_wLi.transpose()(2, 1);
                J_hij_xwj.at<float>(2, 2) = R_wLi.transpose()(2, 2);*/

                auto x_Li = R_wLi.transpose() * (x_wj - t_wi);
                auto tmp = skewSymmetric(x_Li.x(), x_Li.y(), x_Li.z());

                auto J_hij_TLiw = cv::Mat(pointD, poseD, CV_32F, cv::Scalar::all(0));
                J_hij_TLiw.at<float>(0, 0) = -1.0;
                J_hij_TLiw.at<float>(1, 1) = -1.0;
                J_hij_TLiw.at<float>(2, 2) = -1.0;
                J_hij_TLiw.at<float>(0, 3) = tmp(0, 0);
                J_hij_TLiw.at<float>(0, 4) = tmp(0, 1);
                J_hij_TLiw.at<float>(0, 5) = tmp(0, 2);
                J_hij_TLiw.at<float>(1, 3) = tmp(1, 0);
                J_hij_TLiw.at<float>(1, 4) = tmp(1, 1);
                J_hij_TLiw.at<float>(1, 5) = tmp(1, 2);
                J_hij_TLiw.at<float>(2, 3) = tmp(2, 0);
                J_hij_TLiw.at<float>(2, 4) = tmp(2, 1);
                J_hij_TLiw.at<float>(2, 5) = tmp(2, 2);

                auto e = (poseInWorld.inverse()*x_wj) - x_Lj;
                auto b_ij = cv::Mat(pointD,1,CV_32F,cv::Scalar::all(0));
                b_ij.at<float>(0,0) = -e.x();
                b_ij.at<float>(1,0) = -e.y();
                b_ij.at<float>(2,0) = -e.z();

                auto ProwRange = cv::Range(k*minNrOfPoints*pointD + pointD*j, k*minNrOfPoints*pointD + pointD*j + pointD);
                auto PcolRange = cv::Range(k*poseD, k*poseD + poseD);
                //auto SrowRange = ProwRange;
                //auto ScolRange = cv::Range(smoothingFrames*poseD + pointD*j, smoothingFrames*poseD + pointD*j + pointD);

                auto bColRange = cv::Range::all();
                auto bRowRange = cv::Range(k*minNrOfPoints*pointD + j*pointD, k*minNrOfPoints*pointD + j*pointD + pointD);

                cv::Mat PsubMatA = matA.rowRange(ProwRange).colRange(PcolRange);

                //cv::Mat SsubMatA = matA.colRange(ScolRange).rowRange(SrowRange);
                cv::Mat bsubMatB = matB.colRange(bColRange).rowRange(bRowRange);
                
                J_hij_TLiw.copyTo(PsubMatA);
                //J_hij_xwj.copyTo(SsubMatA);
                b_ij.copyTo(bsubMatB);
            }
        }
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
    for (int k = 0; k < smoothingFrames; k++){
        int frameID = latestFrame - smoothingFrames + k;
        pcl::PointXYZ keyPos = cloudKeyPositions->at(frameID);
        PointXYZRPY keyPose = cloudKeyPoses->at(frameID);
        
        keyPose.x       += matX.at<float>(k*poseD + 0, 0); 
        keyPose.y       += matX.at<float>(k*poseD + 1, 0); 
        keyPose.z       += matX.at<float>(k*poseD + 2, 0);

        keyPose.roll    += matX.at<float>(k*poseD + 3, 0); 
        keyPose.pitch   += matX.at<float>(k*poseD + 4, 0); 
        keyPose.yaw     += matX.at<float>(k*poseD + 5, 0); 
        cloudKeyPoses->at(frameID) = keyPose;

        keyPos.x        += matX.at<float>(k*poseD + 0, 0); 
        keyPos.y        += matX.at<float>(k*poseD + 1, 0); 
        keyPos.z        += matX.at<float>(k*poseD + 2, 0); 
        cloudKeyPositions->at(frameID) = keyPos;
    }
    /*for (int i = 0; i < minNrOfPoints; i++){ #TODO: decide whether or not to optimize points also

    }*/
    PointXYZRPY lastKeyPose = cloudKeyPoses->back();
    gtsam::Vector3 rotVec(lastKeyPose.roll, lastKeyPose.pitch, lastKeyPose.yaw);
    gtsam::Point3 trans(lastKeyPose.x, lastKeyPose.y, lastKeyPose.z);
    gtsam::Rot3 orientation = gtsam::Rot3::RzRyRx(rotVec);
    currentPoseInWorld = gtsam::Pose3(orientation, trans);
    //std::cout << "norm: " << cv::norm(matX, cv::NORM_L2) << std::endl;
    return cv::norm(matX, cv::NORM_L2);
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
        _transformMapToWorld();
        /*if (!cloudKeyPoses->empty()){
            PointXYZRPY lastKeyPose = cloudKeyPoses->back();
            gtsam::Vector3 rotVec(lastKeyPose.roll, lastKeyPose.pitch, lastKeyPose.yaw);
            gtsam::Point3 trans(lastKeyPose.x, lastKeyPose.y, lastKeyPose.z);
            gtsam::Rot3 orientation = gtsam::Rot3::RzRyRx(rotVec);
            currentPoseInWorld = gtsam::Pose3(orientation, trans);
        }*/
        _performIsam();
        while (_smoothPoses() > 0.02); // #TODO: Rearrange so can be called before ISAM2
        _publishTrajectory();
        _publishTransformed();

    }
}

void Graph::_transformMapToWorld(){
    pcl::PointCloud<pcl::PointXYZ> currentInWorld;
    pcl::transformPointCloud(*currentCloud, currentInWorld, currentPoseInWorld.matrix());
    if (cloudKeyFramesMap->empty()){
        *cloudKeyFramesMap += currentInWorld;
        return;
    }
    //pcl::PointCloud<pcl::PointXYZ>::Ptr currentMap(new pcl::PointCloud<pcl::PointXYZ>(*cloudKeyFramesMap));
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(cloudKeyFramesMap);
    std::vector<int> indices;
    std::vector<float> distances;
    for (auto &it : currentInWorld.points){

        if (kdTree.nearestKSearch(it, 1, indices, distances) > 0) {
            if (sqrt(distances[0]) > 0.3){
                cloudKeyFramesMap->push_back(it);
            }
        }
        else{
            cloudKeyFramesMap->push_back(it);
        }
    }
    
}
// FOR VISUALIZATION
void Graph::_publishTransformed(){
    if (pubTransformedMap.getNumSubscribers() > 0){
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

void Graph::_publishTrajectory(){
    if (!(pubPoseArray.getNumSubscribers() > 0)) return;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "map";
    for (auto &it : *cloudKeyPoses){
        geometry_msgs::Pose pose;
        pose.position.x = it.x;
        pose.position.y = it.y;
        pose.position.z = it.z;
        tf::Quaternion quat = tf::createQuaternionFromRPY(it.roll, it.pitch, it.yaw);
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        poseArray.poses.push_back(pose);
    }
    pubPoseArray.publish(poseArray);
}