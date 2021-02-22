#include "graph.hpp"

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/registration/correspondence_estimation.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

void matrix_square_root( const cv::Mat& A, cv::Mat& sqrtA ) {
    cv::Mat U, V, Vi, E;
    cv::eigen( A, E, U );
    V = U.t();
    cv::transpose( V, Vi ); // inverse of the orthogonal V
    cv::sqrt(E, E);         // assume that A is positively
                            // defined, otherwise its
                            // square root will be
                            // complex-valued
    sqrtA = V * cv::Mat::diag(E) * Vi;
}

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

    gtsam::Vector6 Sigmas(6);
    Sigmas << 1, 1, 1e-1, 1e-1, 1e-1, 1; 

    cloudKeyPositions.reset(new pcl::PointCloud<pcl::PointXYZ>());
    currentFeatureCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    currentGroundPlaneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloudKeyPoses.reset(new pcl::PointCloud<PointXYZRPY>());
    localKeyFramesMap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    fullMap.reset(new pcl::PointCloud<pcl::PointXYZ>());

    cloudMap.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(voxelRes));
    cloudMap->setInputCloud(fullMap);

    priorNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);
    constraintNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);

}
// Destructor method
Graph::~Graph()
{

}

void Graph::_transformToGlobalMap()
{   

    pcl::PointCloud<pcl::PointXYZ> currentInWorld;
    pcl::transformPointCloud(*currentFeatureCloud, currentInWorld, currentPoseInWorld.matrix());
    std::vector<int> newPointIdxVector;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> changeDetector(voxelRes);
    changeDetector.setInputCloud(fullMap);
    changeDetector.addPointsFromInputCloud();
    changeDetector.switchBuffers();
    changeDetector.setInputCloud(currentInWorld.makeShared());
    changeDetector.addPointsFromInputCloud();
    changeDetector.getPointIndicesFromNewVoxels(newPointIdxVector);

    for (std::size_t i = 0; i<newPointIdxVector.size(); ++i){
        cloudMap->addPointToCloud(currentInWorld[newPointIdxVector[i]],fullMap);
    }
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

    //ROS_INFO("SAVING NEW KEY FRAME");
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
    pcl::copyPointCloud(*currentFeatureCloud, *thisKeyFrame);
    cloudKeyFrames.push_back(thisKeyFrame);
}

void Graph::_createKeyFramesMap(){
    if (cloudKeyFrames.size() < smoothingFrames) return;
    localKeyFramesMap->clear();
    for (int i = 0; i<smoothingFrames; i++){
        int frameID = cloudKeyFrames.size() - smoothingFrames + i;
        PointXYZRPY poseInFrame = cloudKeyPoses->at(frameID);
        gtsam::Vector3 rotVec(poseInFrame.roll, poseInFrame.pitch, poseInFrame.yaw);
        gtsam::Point3 trans(poseInFrame.x, poseInFrame.y, poseInFrame.z);
        gtsam::Rot3 orientation = gtsam::Rot3::RzRyRx(rotVec);
        gtsam::Pose3 pose = gtsam::Pose3(orientation, trans);

        pcl::PointCloud<pcl::PointXYZ> keyFrameInWorld;
        pcl::transformPointCloud(*cloudKeyFrames[i], keyFrameInWorld, pose.matrix());
        *localKeyFramesMap += keyFrameInWorld;
    }
}

void Graph::_smoothPoses(){
    auto skewSymmetric = [](double a, double b, double c){return gtsam::skewSymmetric(a, b, c);};

    if (cloudKeyFrames.size() < smoothingFrames) return;
    //ROS_INFO("RUNNING SMOOTHING");
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
    //int ARows = minNrOfPoints*smoothingFrames*poseD;
    int BRows = ARows;
    int ACols = poseD*smoothingFrames; //+ pointD*minNrOfPoints;
    int XCols = ACols;
    //int iter = 0;
    double lambda = 1e-4;
    std::vector<gtsam::Point3> worldPoints; 
    worldPoints.reserve(minNrOfPoints*smoothingFrames);
    std::vector<gtsam::Point3> localPoints; 
    localPoints.reserve(minNrOfPoints*smoothingFrames); //reserve space
    for (int iter = 0; iter<maxIterSmoothing; iter++){
        worldPoints.clear(); localPoints.clear();
        cv::Mat matA = cv::Mat::zeros(ARows, ACols, CV_64FC1);
        cv::Mat matAt(ACols, ARows, CV_64FC1, cv::Scalar::all(0.0));
        cv::Mat matAtA(ACols, ACols, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matB(BRows, 1, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matAtB(ACols, 1, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matX(XCols, 1, CV_64FC1, cv::Scalar::all(0));
        for (int k = 0; k < smoothingFrames; ++k){
            //#TODO: CALCULATE CORRESPONDENCES DIFFERENTLY
            // Extract frame
            int frameID = latestFrame - smoothingFrames + k;
            pcl::PointCloud<pcl::PointXYZ> framePoints = *cloudKeyFrames[frameID];
            pcl::PointCloud<pcl::PointXYZ> frameInWorld;
            PointXYZRPY keyPose = cloudKeyPoses->at(frameID);
            
            gtsam::Pose3 poseInWorld;
            _fromPointXYZRPYToPose3(keyPose, poseInWorld);

            pcl::transformPointCloud(framePoints, frameInWorld, poseInWorld.matrix());
            auto R_wLi = poseInWorld.rotation();
            auto t_wi  = poseInWorld.translation();

            std::vector<int> indices;
            std::vector<float> distances;
            for (int j = 0; j < minNrOfPoints; j++){
                auto pointInWorld = frameInWorld.at(j);
                auto pointInLocalFrame = framePoints.at(j);
                if (cloudMap->nearestKSearch(pointInWorld, 1, indices, distances) > 0) {
                    // Extract points
                    auto q_wjPCL = fullMap->at(indices[0]);
                    auto q_wj = gtsam::Point3(q_wjPCL.x, q_wjPCL.y, q_wjPCL.z);
                    auto p_wj = gtsam::Point3(pointInWorld.x, pointInWorld.y, pointInWorld.z);
                    auto p_Lij = gtsam::Point3(pointInLocalFrame.x, pointInLocalFrame.y, pointInLocalFrame.z);
                    worldPoints[k*minNrOfPoints + j] = q_wj;
                    localPoints[k*minNrOfPoints + j] = p_Lij;

                    gtsam::Matrix3 tmp = - (R_wLi.matrix() * skewSymmetric(p_Lij.x(), p_Lij.y(), p_Lij.z()));

                    // Calculate Jacobians
                    auto J_hij_TwLi = cv::Mat(pointD, poseD, CV_64F, cv::Scalar::all(0));
                    J_hij_TwLi.at<double>(0, 0) = R_wLi.matrix()(0, 0);
                    J_hij_TwLi.at<double>(0, 1) = R_wLi.matrix()(0, 1);
                    J_hij_TwLi.at<double>(0, 2) = R_wLi.matrix()(0, 2);
                    J_hij_TwLi.at<double>(1, 0) = R_wLi.matrix()(1, 0);
                    J_hij_TwLi.at<double>(1, 1) = R_wLi.matrix()(1, 1);
                    J_hij_TwLi.at<double>(1, 2) = R_wLi.matrix()(1, 2);
                    J_hij_TwLi.at<double>(2, 0) = R_wLi.matrix()(2, 0);
                    J_hij_TwLi.at<double>(2, 1) = R_wLi.matrix()(2, 1);
                    J_hij_TwLi.at<double>(2, 2) = R_wLi.matrix()(2, 2);
                    J_hij_TwLi.at<double>(0, 3) = tmp(0, 0);
                    J_hij_TwLi.at<double>(0, 4) = tmp(0, 1);
                    J_hij_TwLi.at<double>(0, 5) = tmp(0, 2);
                    J_hij_TwLi.at<double>(1, 3) = tmp(1, 0);
                    J_hij_TwLi.at<double>(1, 4) = tmp(1, 1);
                    J_hij_TwLi.at<double>(1, 5) = tmp(1, 2);
                    J_hij_TwLi.at<double>(2, 3) = tmp(2, 0);
                    J_hij_TwLi.at<double>(2, 4) = tmp(2, 1);
                    J_hij_TwLi.at<double>(2, 5) = tmp(2, 2);

                    auto e = p_wj - q_wj;
                    auto b_ij = cv::Mat(pointD,1,CV_64F,cv::Scalar::all(0));
                    b_ij.at<double>(0,0) = -e.x();
                    b_ij.at<double>(1,0) = -e.y();
                    b_ij.at<double>(2,0) = -e.z();

                    // Extract submatrice to insert into
                    auto ProwRange = cv::Range(k*minNrOfPoints*pointD + pointD*j, k*minNrOfPoints*pointD + pointD*j + pointD);
                    auto PcolRange = cv::Range(k*poseD, k*poseD + poseD);
                    //auto SrowRange = ProwRange;
                    //auto ScolRange = cv::Range(smoothingFrames*poseD + pointD*j, smoothingFrames*poseD + pointD*j + pointD);
                    auto bColRange = cv::Range::all();
                    auto bRowRange = cv::Range(k*minNrOfPoints*pointD + j*pointD, k*minNrOfPoints*pointD + j*pointD + pointD);
                    //auto bRowRange = cv::Range(k*minNrOfPoints*poseD + j*poseD, k*minNrOfPoints*poseD + j*poseD + poseD);

                    cv::Mat PsubMatA = matA.rowRange(ProwRange).colRange(PcolRange);
                    //cv::Mat SsubMatA = matA.colRange(ScolRange).rowRange(SrowRange);
                    cv::Mat bsubMatB = matB.colRange(bColRange).rowRange(bRowRange);

                    // Propagate uncertainty
                    cv::Mat sigmas(6, 6, CV_64F, cv::Scalar::all(0));
                    cv::eigen2cv(odometryNoise->covariance(), sigmas);
                    cv::Mat sigmasFloat;
                    sigmas.convertTo(sigmasFloat, CV_64F);

                    cv::Mat whitener = J_hij_TwLi * sigmasFloat * J_hij_TwLi.t();
                    cv::Mat whitenerInv;
                    cv::invert(whitener, whitenerInv, cv::DECOMP_SVD);
                    cv::Mat whitenerSqrtInv;
                    matrix_square_root(whitenerInv, whitenerSqrtInv);

                    // Copy into submatrices
                    cv::Mat Ai = whitenerSqrtInv * J_hij_TwLi;
                    Ai.copyTo(PsubMatA);
                    cv::Mat bi = whitenerSqrtInv * b_ij;
                    //J_hij_xwj.copyTo(SsubMatA);
                    bi.copyTo(bsubMatB);
                }
            }
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        auto matAtAdiag = cv::Mat::diag(matAtA.diag());
        matAtB = matAt * matB;
        cv::solve(matAtA + (lambda * matAtAdiag), matAtB, matX, cv::DECOMP_QR);
        std::vector<PointXYZRPY> keyPosesBefore;
        std::vector<PointXYZRPY> keyPosesAfter;
        std::vector<PointXYZRPY> keyPosesResult;
        std::cout << cv::norm(matX, 2) << std::endl;
        // Check Update
        for (int k = 0; k < smoothingFrames; k++){
            int frameID = latestFrame - smoothingFrames + k;
            pcl::PointXYZ keyPos = cloudKeyPositions->at(frameID);
            PointXYZRPY keyPose = cloudKeyPoses->at(frameID);
            keyPosesBefore.push_back(keyPose);

            gtsam::Pose3 gtsamKeyPose;
            _fromPointXYZRPYToPose3(keyPose, gtsamKeyPose);

            gtsam::Vector6 xi;
            xi << matX.at<double>(k*poseD + 3, 0), matX.at<double>(k*poseD + 4, 0), matX.at<double>(k*poseD + 5, 0), matX.at<double>(k*poseD + 0, 0), matX.at<double>(k*poseD + 1, 0), matX.at<double>(k*poseD + 2, 0);

            gtsam::Pose3 tau = gtsam::Pose3::Expmap(xi);

            _fromPose3ToPointXYZRPY(gtsamKeyPose * tau, keyPose);
            keyPosesAfter.push_back(keyPose);
        }
        //_applyUpdate(keyPosesAfter, latestFrame);
        double fxBefore = 0;
        double fxAfter = 0;
        double fxResult = 0;
        _evaluate_transformation(minNrOfPoints, latestFrame, keyPosesBefore, keyPosesAfter, worldPoints, localPoints, fxBefore, fxAfter);
        auto streng = "BEFORE: " + std::to_string(fxBefore) + ", AFTER: " + std::to_string(fxAfter);
        std::cout << streng << std::endl;
        if (fxAfter < fxBefore){
            ROS_INFO("GOOD STEP");

            //std::lock_guard<std::mutex> lock(mtx);
            _applyUpdate(keyPosesAfter, latestFrame);
            lambda /= 10;
            fxResult = fxAfter;
        }
        else{
            //ROS_INFO("BAD STEP");
            lambda *= 10;
            fxResult = fxBefore;
        }

        if (fxResult < fxTol || cv::norm(matX, 2) < stepTol) {
            break;
        }
    }
    PointXYZRPY lastKeyPose = cloudKeyPoses->back();
    gtsam::Vector3 rotVec(lastKeyPose.roll, lastKeyPose.pitch, lastKeyPose.yaw);
    gtsam::Point3 trans(lastKeyPose.x, lastKeyPose.y, lastKeyPose.z);
    gtsam::Rot3 orientation = gtsam::Rot3::RzRyRx(rotVec);
    currentPoseInWorld = gtsam::Pose3(orientation, trans) * displacement;
    /*for (int i = 0; i < minNrOfPoints; i++){ #TODO: decide whether or not to optimize points also

    }*/
    //std::cout << "norm: " << cv::norm(matX, cv::NORM_L2) << std::endl;
}

void Graph::_applyUpdate(std::vector<PointXYZRPY> keyPoses, int latestFrame)
{
    for (int k = 0; k < smoothingFrames; k++){
        int frameID = latestFrame - smoothingFrames + k;
        cloudKeyPoses->at(frameID) = keyPoses.at(k);

    }
}

void Graph::_updateIsam()
{

}

void Graph::_evaluate_transformation(int minNrOfPoints, int latestFrame, const std::vector<PointXYZRPY>& posesBefore, const std::vector<PointXYZRPY>& posesAfter, const std::vector<gtsam::Point3> &pointsWorld, const std::vector<gtsam::Point3> &pointsLocal, double &resultBefore, double &resultAfter)
{   
    resultBefore = 0; resultAfter = 0;
    for (int k = 0; k < smoothingFrames; ++k){
        PointXYZRPY keyPoseBefore = posesBefore.at(k);
        PointXYZRPY keyPoseAfter  = posesAfter.at(k);

        gtsam::Pose3 poseInWorldBefore;
        gtsam::Pose3 poseInWorldAfter;
        _fromPointXYZRPYToPose3(keyPoseBefore, poseInWorldBefore);
        _fromPointXYZRPYToPose3(keyPoseAfter, poseInWorldAfter);
        for (int j = 0; j < minNrOfPoints; j++){

            auto q_wj = pointsWorld[k*minNrOfPoints + j];
            auto p_Lij = pointsLocal[k*minNrOfPoints + j];

            resultBefore += pow(gtsam::norm3(poseInWorldBefore * p_Lij - q_wj), 2);
            resultAfter += pow(gtsam::norm3(poseInWorldAfter * p_Lij - q_wj), 2);
        }
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
    pcl::fromROSMsg(*pointCloud2Msg, *currentFeatureCloud);
    newMap = true;
}

void Graph::runOnce()
{
    if (newLaserOdometry && newMap){
        //std::lock_guard<std::mutex> lock(mtx);
        newLaserOdometry, newMap = false;
        int iterCount = 0;

        _incrementPosition();
        //_transformMapToWorld();
        _transformToGlobalMap();
        _createKeyFramesMap();
        _smoothPoses();// #TODO: Rearrange so can be called before ISAM2
        _performIsam();
        _publishTrajectory();
        _publishTransformed();

    }
}

void Graph::runSmoothing()
{
    if (smoothingEnabledFlag == false) return;
    ros::Rate rate(1);
    while (ros::ok){
        rate.sleep();
        _smoothPoses();
        _updateIsam();
    }
}

void Graph::_transformMapToWorld()
{
    pcl::PointCloud<pcl::PointXYZ> currentInWorld;
    pcl::transformPointCloud(*currentFeatureCloud, currentInWorld, currentPoseInWorld.matrix());
    /*for (auto &it : currentInWorld.points){
        cloudMap->addPointToCloud(it, cloudKeyFramesMap);
    }*/
    
    
    if (fullMap->empty()){
        *fullMap += currentInWorld;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentMap(new pcl::PointCloud<pcl::PointXYZ>(*fullMap));
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(fullMap);
    std::vector<int> indices;
    std::vector<float> distances;
    for (auto &it : currentInWorld.points){

        if (kdTree.nearestKSearch(it, 1, indices, distances) > 0) {
            if (sqrt(distances[0]) > 0.5){
                fullMap->push_back(it);
            }
        }
        else{
            fullMap->push_back(it);
        }
    }
    
}
// FOR VISUALIZATION
void Graph::_publishTransformed()
{
    if (pubTransformedMap.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*fullMap, msg);
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

void Graph::_publishTrajectory()
{
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

void Graph::_fromPointXYZRPYToPose3(const PointXYZRPY &poseIn, gtsam::Pose3 &poseOut)
{
        gtsam::Vector6 xi;
        xi << poseIn.roll, poseIn.pitch, poseIn.yaw, poseIn.x, poseIn.y, poseIn.z;
        poseOut = gtsam::Pose3::Expmap(xi);
        /*gtsam::Vector3 rotVec(poseIn.roll, poseIn.pitch, poseIn.yaw);
        gtsam::Point3 trans(poseIn.x, poseIn.y, poseIn.z);
        gtsam::Rot3 oriLocal = gtsam::Rot3::RzRyRx(rotVec);
        poseOut = gtsam::Pose3(oriLocal, trans);*/
}

void Graph::_fromPose3ToPointXYZRPY(const gtsam::Pose3 &poseIn, PointXYZRPY &poseOut)
{
    poseOut.x = poseIn.translation().x(); 
    poseOut.y = poseIn.translation().y(); 
    poseOut.z = poseIn.translation().z();
    poseOut.roll = poseIn.rotation().roll();
    poseOut.pitch = poseIn.rotation().pitch();
    poseOut.yaw = poseIn.rotation().yaw();
}