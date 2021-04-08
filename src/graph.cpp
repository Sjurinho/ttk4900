#include "graph.hpp"

#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/navigation/GPSFactor.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (x,y,z)

typedef gtsam::BearingRange<gtsam::Pose3, gtsam::Point3> BearingRange3D;

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

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 1e-4;//8e-05;
  double accel_bias_rw_sigma = 0.1;//0.02;
  double gyro_bias_rw_sigma = 0.002;//0.0001454441043;
  gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(accel_noise_sigma, 2);
  gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(gyro_noise_sigma, 2);
  gtsam::Matrix33 integration_error_cov =
      gtsam::I_3x3 * 1e-5;  // error committed in integrating position from velocities
  gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(accel_bias_rw_sigma, 2);
  gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(gyro_bias_rw_sigma, 2);
  gtsam::Matrix66 bias_acc_omega_int =
      gtsam::I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  return p;
}


//constructor method
Graph::Graph(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{   
    nh_ = nh;
    ROS_INFO("Initializing Graph Node");

    //Subscribers and publishers
    subOdometry = nh.subscribe<nav_msgs::Odometry>("/lidarOdom", 32, &Graph::odometryHandler, this);
    subMap = nh.subscribe<sensor_msgs::PointCloud2>("/featurePointCloud", 32, &Graph::mapHandler, this);
    subGroundPlane = nh.subscribe<sensor_msgs::PointCloud2>("/groundPlanePointCloud", 32, &Graph::groundPlaneHandler, this);
    subImu = nh.subscribe<sensor_msgs::Imu>("/imu", 32, &Graph::imuHandler, this);
    subGnss = nh.subscribe<geometry_msgs::PoseStamped>("/gnss", 32, &Graph::gnssHandler, this);
    pubTransformedMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    pubTransformedPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1);
    pubPoseArray = nh.advertise<geometry_msgs::PoseArray>("/poseArray", 1);
    pubReworkedMap = nh.advertise<sensor_msgs::PointCloud2>("/reworkedMap", 1);
    
    //Initializing and allocation of memory
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    isam = new gtsam::ISAM2(parameters);

    gtsam::Vector6 Sigmas(6);
    Sigmas << 0.05, 0.05, 1e-3, 0.05, 0.05, 0.1; // rad, rad, rad, m, m, m
    gtsam::Vector6 imuSigmas(6);
    imuSigmas << 5e-4, 5e-4, 5e-4, 0.001, 0.001, 0.02; // rad, rad, rad, m, m, m
    gtsam::Vector3 structureSigmas(3);
    structureSigmas << 0.3, 0.3, 0.3; //m, m, m
    gtsam::Vector3 gnssSigmas(3);
    gnssSigmas << 0.2, 0.2, 0.3;

    downSizeFilterMap.setLeafSize(voxelRes, voxelRes, voxelRes);

    cloudKeyPositions.reset(new pcl::PointCloud<pcl::PointXYZ>());
    currentFeatureCloud.reset(new pcl::PointCloud<pointT>());
    currentGroundPlaneCloud.reset(new pcl::PointCloud<pointT>());
    cloudKeyPoses.reset(new pcl::PointCloud<PointXYZRPY>());
    localKeyFramesMap.reset(new pcl::PointCloud<pointT>());
    cloudMapFull.reset(new pcl::PointCloud<pointT>());

    octreeMap.reset(new pcl::octree::OctreePointCloudSearch<pointT>(voxelRes));
    octreeMap->setInputCloud(cloudMapFull);

    priorNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);
    constraintNoise = gtsam::noiseModel::Diagonal::Variances(Sigmas);
    imuPoseNoise = gtsam::noiseModel::Diagonal::Variances(imuSigmas);
    imuVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); // m/s
    imuBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
    structureNoise = gtsam::noiseModel::Diagonal::Variances(structureSigmas);
    gnssNoise = gtsam::noiseModel::Diagonal::Variances(gnssSigmas);


    gtsam::imuBias::ConstantBias priorImuBias; //assumed zero

    auto p = imuParams();

    //preintegrated = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, priorImuBias);
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p, priorImuBias);

}
// Destructor method
Graph::~Graph()
{

}

void Graph::_transformToGlobalMap()
{   

    pcl::PointCloud<pointT> currentInWorld;

    pcl::transformPointCloud(*currentFeatureCloud, currentInWorld, currentPoseInWorld.matrix());
    
    std::vector<int> newPointIdxVector;
    pcl::octree::OctreePointCloudChangeDetector<pointT> changeDetector(voxelRes);
    changeDetector.setInputCloud(cloudMapFull);
    changeDetector.addPointsFromInputCloud();
    changeDetector.switchBuffers();
    changeDetector.setInputCloud(currentInWorld.makeShared());
    changeDetector.addPointsFromInputCloud();
    changeDetector.getPointIndicesFromNewVoxels(newPointIdxVector);

    for (std::size_t i = 0; i<newPointIdxVector.size(); ++i){
        octreeMap->addPointToCloud(currentInWorld[newPointIdxVector[i]],cloudMapFull);
    }
}

void Graph::_incrementPosition()
{   

    currentPoseInWorld = currentPoseInWorld * displacement;
    currentPosPoint = pcl::PointXYZ(currentPoseInWorld.x(), currentPoseInWorld.y(), currentPoseInWorld.z());

}

void Graph::_initializePreintegration(){
    ROS_INFO("IMU DETECTED - INITIALIZING");
    gtsam::Vector3 priorVelocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias priorBias; //Assumed 0 initial bias
    _graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), priorVelocity, imuVelocityNoise));
    _graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), priorBias, imuBiasNoise));

    initialEstimate.insert(V(0), priorVelocity);
    initialEstimate.insert(B(0), priorBias);

    prevImuState = gtsam::NavState(currentPoseInWorld, priorVelocity);
    predImuState = prevImuState;
    updateImu = true;
}

void Graph::_performIsam()
{
    bool saveThisKeyFrame = true;

    double squaredDistance = 
    (previousPosPoint.x-currentPosPoint.x)*(previousPosPoint.x-currentPosPoint.x)
    +(previousPosPoint.y-currentPosPoint.y)*(previousPosPoint.y-currentPosPoint.y)
    +(previousPosPoint.z-currentPosPoint.z)*(previousPosPoint.z-currentPosPoint.z);

    if (sqrt(squaredDistance) < keyFrameSaveDistance){
        saveThisKeyFrame = false;
    }

    if (saveThisKeyFrame == false && !cloudKeyPositions->points.empty()) return;

    //ROS_INFO("SAVING NEW KEY FRAME");
    previousPosPoint = currentPosPoint;
    uint64_t index = cloudKeyPositions->points.size();
    //std::cout << "INDEX: " << index << std::endl;
    if (cloudKeyPositions->points.empty()){
        _graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(index), currentPoseInWorld, priorNoise));
        
        initialEstimate.insert(X(index), currentPoseInWorld);

        lastPoseInWorld = currentPoseInWorld;

        //_initializePreintegration();
    }
    else{
        _graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(index-1), X(index), lastPoseInWorld.between(currentPoseInWorld), odometryNoise));
        initialEstimate.insert(X(index), currentPoseInWorld);

        if (updateImu){
            auto preintImuCombined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*preintegrated);

            gtsam::CombinedImuFactor combinedImuFactor(X(index-1), V(index-1), X(index), V(index), B(index-1), B(index), preintImuCombined);

            _graph.add(combinedImuFactor);
            initialEstimate.insert(V(index), predImuState.v());
            initialEstimate.insert(B(index), prevImuBias);
        }
        
        for(auto it = newKeyGnssMeasurementPairs.begin(); it != newKeyGnssMeasurementPairs.end();){
            if (isamCurrentEstimate.exists(it->first)){
                std::cout << "GPS: " << it->second << std::endl;
                gtsam::GPSFactor gpsFactor(it->first, it->second.translation(), gnssNoise);
                keyGnssMeasurementPairs.push_back(*it);
                newGnssInGraph = true;
                _graph.add(gpsFactor);
                it = newKeyGnssMeasurementPairs.erase(it);
            }
            else ++it;
        }
    }

    //std::cout << currentPoseInWorld << std::endl;

    PointXYZRPY currentPose;
    isam->update(_graph, initialEstimate);
    isam->update();

    _graph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();

    if (updateImu){
        prevImuState = gtsam::NavState(isamCurrentEstimate.at<gtsam::Pose3>(X(index)), isamCurrentEstimate.at<gtsam::Vector3>(V(index)));
        prevImuBias = isamCurrentEstimate.at<gtsam::imuBias::ConstantBias>(B(index));
        preintegrated->resetIntegrationAndSetBias(prevImuBias);
        updateImu=false;
        imuFactorAdded = true;
    }

    currentPoseInWorld = isamCurrentEstimate.at<gtsam::Pose3>(X(index));

    cloudKeyPositions->push_back(pcl::PointXYZ(currentPoseInWorld.x(), currentPoseInWorld.y(), currentPoseInWorld.z()));
    
    _fromPose3ToPointXYZRPY(currentPoseInWorld, currentPose);

    cloudKeyPoses->push_back(currentPose);
    timeKeyPosePairs.push_back(std::pair<double, gtsam::Pose3>(timeOdometry, currentPoseInWorld));

    lastPoseInWorld = currentPoseInWorld;

    pcl::PointCloud<pointT>::Ptr thisKeyFrame(new pcl::PointCloud<pointT>());
    downSizeFilterMap.setInputCloud(currentFeatureCloud);
    downSizeFilterMap.filter(*thisKeyFrame);
    cloudKeyFrames.push_back(thisKeyFrame);
    cloudsInQueue += 1;
}

/*void Graph::_lateralEstimation()
{
    double normalMean[3] = { 0.0 };
    for (auto gpPoint : currentGroundPlaneCloud->points){
        if (isnan(gpPoint.normal_x) || isnan(gpPoint.normal_y) || isnan(gpPoint.normal_z)) {
            continue;
        }
        normalMean[0] += gpPoint.normal_x;
        normalMean[1] += gpPoint.normal_y;
        normalMean[2] += gpPoint.normal_z;
    }
    normalMean[0] /= currentGroundPlaneCloud->points.size();
    normalMean[1] /= currentGroundPlaneCloud->points.size();
    normalMean[2] /= currentGroundPlaneCloud->points.size();

    double len = sqrt(normalMean[0] * normalMean[0] + normalMean[1] * normalMean[1] + normalMean[2] * normalMean[2]);
    normalMean[0] /= len;
    normalMean[1] /= len;
    normalMean[2] /= len;
    float deltaPitch = acos(1-normalMean[2]) - M_PI/2;
    float deltaRoll = acos(normalMean[0]) - M_PI/2;
    //#TODO: FINISH/DECIDE WHETHER OR NOT TO USE THIS
}*/

void Graph::_mapToGraph(){
    mtx.lock();
    if (cloudsInQueue==0){
        mtx.unlock();
        return;
    }
    int startIdx = cloudKeyFrames.size() - cloudsInQueue;
    int cloudsInQueueAtRunTime = cloudsInQueue;

    std::vector<gtsam::Pose3> framePoses;
    for (int i = startIdx; i<startIdx + cloudsInQueueAtRunTime; i++)
    {
        framePoses.push_back(isamCurrentEstimate.at<gtsam::Pose3>(X(i)));
    }
    pcl::registration::CorrespondenceEstimation<pointT, pointT> matcher;
    matcher.setInputTarget(cloudMapFull);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pointT> trimmer;
    trimmer.setInputTarget(cloudMapFull);
    //pcl::registration::CorrespondenceRejectorTrimmed trimmer;
    mtx.unlock();

    //trimmer.setOverlapRatio(0.4);
    trimmer.setMaximumIterations(500);
    trimmer.setRefineModel(true);

    gtsam::ExpressionFactorGraph graph;
    gtsam::Values initial;
    for (int cloudnr = startIdx; cloudnr < startIdx + cloudsInQueueAtRunTime; cloudnr++){
        gtsam::Pose3 pose = framePoses[cloudnr-startIdx];

        pcl::PointCloud<pointT> cloudInWorld, filteredCloud;
        pcl::PointCloud<pointT> cloud = *cloudKeyFrames.at(cloudnr);

        pcl::transformPointCloud(cloud, cloudInWorld, pose.matrix());

        pcl::CorrespondencesPtr allCorrespondences(new pcl::Correspondences);
        matcher.setInputSource(cloudInWorld.makeShared());
        matcher.determineReciprocalCorrespondences(*allCorrespondences, 0.5); 

        pcl::CorrespondencesPtr trimmedCorrespondences(new pcl::Correspondences);
        trimmer.setInputSource(cloudInWorld.makeShared());
        trimmer.setInputCorrespondences(allCorrespondences);
        
        trimmer.getCorrespondences(*trimmedCorrespondences);
        if (trimmedCorrespondences->size() < minCorresponendencesStructure) continue;
        std::cout << "MAP CORRESPONDENCES: " << trimmedCorrespondences->size() << std::endl;
        for (int j = 0; j<trimmedCorrespondences->size(); j++){

            int pointIdx = trimmedCorrespondences->at(j).index_match;
            pointT pclPoint = cloudMapFull->at(pointIdx);
            gtsam::Point3 pointWorld = gtsam::Point3(pclPoint.x, pclPoint.y, pclPoint.y);
            pointT pclPointFrame = cloudInWorld.at(trimmedCorrespondences->at(j).index_query);
            gtsam::Point3 pointMeasured = gtsam::Point3(pclPointFrame.x, pclPointFrame.y, pclPointFrame.z);
            /*auto prediction = gtsam::Expression<BearingRange3D>
            (BearingRange3D::Measure, gtsam::Pose3_(X(cloudnr)), gtsam::Point3_(L(pointIdx)));*/
            auto prediction = gtsam::Expression<BearingRange3D>
            (BearingRange3D::Measure, gtsam::Pose3_(X(cloudnr)), gtsam::Point3_(L(pointIdx)));
            auto measurement = BearingRange3D(pose.bearing(pointMeasured), pose.range(pointMeasured));

            graph.addExpressionFactor(prediction, measurement, structureNoise);
            if (!isamCurrentEstimate.exists(L(pointIdx)) && !initial.exists(L(pointIdx))) {
                initial.insert(L(pointIdx), pointMeasured);
                mapKeys.push_back(std::make_pair(L(pointIdx), pointIdx));
            }
            /*if (!smoothMapEstimate.exists(L(pointIdx))){
                initial.insert(L(pointIdx), pointMeasured);
                mapKeys.push_back(std::make_pair(L(pointIdx), pointIdx));
            }*/
        }
        /*
        if (cloudnr > startIdx){
            graph.addExpressionFactor(gtsam::between(gtsam::Pose3_(X(cloudnr-1)), gtsam::Pose3_(X(cloudnr))), framePoses[cloudnr-1].between(framePoses[cloudnr]), imuPoseNoise);
        }
        initial.insert(X(cloudnr), framePoses[cloudnr]);*/
    }
    /*std::cout << "initial error: " << graph.error(initial) << std::endl;
    isamMap->update(graph, initial);
    isamMap->update();
    smoothMapEstimate = isamMap->calculateEstimate();*/
    mtx.lock();
    cloudsInQueue = 0;
    isam->update(graph, initial);
    isam->update();
    isamCurrentEstimate = isam->calculateEstimate();
    /*for (auto key : mapKeys){
        gtsam::Point3 point = isamCurrentEstimate.at<gtsam::Point3>(key.first);
        pointT pclpoint;
        pclpoint.x = point.x(); pclpoint.y = point.y(); pclpoint.z = point.z();
        cloudMapFull->at(key.second) = pclpoint;
    }*/
    mtx.unlock();
    //smoothMapEstimate = isam->calculateEstimate();
    //graph.keys().print();
    //_publishReworkedMap(keys);
}

void Graph::_investigateLoopClosure()
{
    mtx.lock();
    if (!newGnssInGraph) mtx.unlock(); return;
    gtsam::NonlinearFactorGraph graph;
    newGnssInGraph = false;
    auto newGnssMeasurement = keyGnssMeasurementPairs.back();
    for (int i = 0; i<keyGnssMeasurementPairs.size()-1; i++){
        auto keyGnssMeasurementPair = keyGnssMeasurementPairs[i];
        auto lcFactor = gtsam::BetweenFactor<gtsam::Pose3>(newGnssMeasurement.first, keyGnssMeasurementPair.first, newGnssMeasurement.second.between(keyGnssMeasurementPair.second), loopClosureNoise);
        graph.add(lcFactor);
    }
    isam->update(graph);
    isam->update();
    isamCurrentEstimate = isam->calculateEstimate();
    mtx.unlock();
}

void Graph::odometryHandler(const nav_msgs::OdometryConstPtr &odomMsg)
{
    double time = odomMsg->header.stamp.toSec();

    gtsam::Point3 pos(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(odomMsg->pose.pose.orientation.w, odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z);
    gtsam::Pose3 pose(rot, pos);

    mtx.lock();
    odometryMeasurements.push_back(std::pair<double, gtsam::Pose3>(time, pose));
    timeOdometry = time;
    displacement = pose;
    newLaserOdometry=true;
    mtx.unlock();
}

void Graph::mapHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{   
    pcl::PointCloud<pointT> tmp;
    pcl::fromROSMsg(*pointCloud2Msg, tmp);
    mtx.lock();
    timeMap = pointCloud2Msg->header.stamp.toSec();
    *currentFeatureCloud = tmp;
    newMap = true;
    mtx.unlock();
}

void Graph::groundPlaneHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{   
    pcl::PointCloud<pointT> tmp;
    pcl::fromROSMsg(*pointCloud2Msg, tmp);
    mtx.lock();
    *currentGroundPlaneCloud = tmp;
    timeMap = pointCloud2Msg->header.stamp.toSec();
    newGroundPlane = true;
    mtx.unlock();
}

void Graph::imuHandler(const sensor_msgs::ImuConstPtr &imuMsg){
    if (!imuEnabledFlag) return;
    double time = imuMsg->header.stamp.toSec();
    gtsam::Vector6 measurement;
    measurement << imuMsg->linear_acceleration.x, -imuMsg->linear_acceleration.y, -imuMsg->linear_acceleration.z, imuMsg->angular_velocity.x, -imuMsg->angular_velocity.y, -imuMsg->angular_velocity.z; //IMU measurement in Lidar frame
    //measurement << imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z, imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z;
    mtx.lock();
    imuMeasurements.push_back(std::pair<double, gtsam::Vector6>(time, measurement));
    newImu = true;
    mtx.unlock();
}

void Graph::gnssHandler(const geometry_msgs::PoseStampedConstPtr &gnssMsg)
{
    if (!gnssEnabledFlag) return;
    double time = gnssMsg->header.stamp.toSec();
    gtsam::Point3 pos(gnssMsg->pose.position.x, gnssMsg->pose.position.y, gnssMsg->pose.position.z);
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(gnssMsg->pose.orientation.w, gnssMsg->pose.orientation.x, gnssMsg->pose.orientation.y, gnssMsg->pose.orientation.z);
    gtsam::Pose3 pose(rot, pos);

    mtx.lock();
    gnssMeasurements.push_back(std::pair<double, gtsam::Pose3>(time, pose));
    newGnss = true;
    mtx.unlock();
}

void Graph::_cloud2Map(){
    auto skewSymmetric = [](double a, double b, double c){return gtsam::skewSymmetric(a, b, c);};

    if (cloudKeyFrames.size() < 1) return;

    pcl::CorrespondencesPtr allCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pointT, pointT> matcher;

    pcl::CorrespondencesPtr partialOverlapCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorTrimmed trimmer;
    trimmer.setInputCorrespondences(allCorrespondences);
    trimmer.setOverlapRatio(0.6);


    pcl::PointCloud<pointT> framePoints = *currentFeatureCloud;
    pcl::PointCloud<pointT> frameInWorld;

    if (updateImu){
        pcl::transformPointCloud(framePoints, frameInWorld, predImuState.pose().matrix());
    }
    else{
        pcl::transformPointCloud(framePoints, frameInWorld, currentPoseInWorld.matrix());
    }

    matcher.setInputSource(frameInWorld.makeShared());
    matcher.setInputTarget(cloudMapFull);
    matcher.determineReciprocalCorrespondences(*allCorrespondences); 
    trimmer.getCorrespondences(*partialOverlapCorrespondences);

    //std::cout << "Correspondences: " << partialOverlapCorrespondences->size() << std::endl;

    int nPoints = partialOverlapCorrespondences->size();
    int pointD = 3; int poseD = 6; int priorD = updateImu ? 6:0;
    int ARows = nPoints*pointD + priorD;
    int BRows = ARows;
    int ACols = poseD;// + pointD*nPoints;
    int XCols = ACols;
    double lambda = 1e-4;
    std::vector<gtsam::Point3> worldPoints; 
    worldPoints.reserve(nPoints);
    std::vector<gtsam::Point3> localPoints; 
    localPoints.reserve(nPoints); //reserve space
    for (int iter = 0; iter<maxIterSmoothing; iter++){
        worldPoints.clear(); localPoints.clear();
        cv::Mat matA = cv::Mat::zeros(ARows, ACols, CV_64FC1);
        cv::Mat matAt(ACols, ARows, CV_64FC1, cv::Scalar::all(0.0));
        cv::Mat matAtA(ACols, ACols, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matB(BRows, 1, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matAtB(ACols, 1, CV_64FC1, cv::Scalar::all(0));
        cv::Mat matX(XCols, 1, CV_64FC1, cv::Scalar::all(0));

        auto R_wLi = currentPoseInWorld.rotation();
        auto t_wi  = currentPoseInWorld.translation();

        for (int j = 0; j < nPoints; j++){
            int sourceIndex = partialOverlapCorrespondences->at(j).index_query;
            int targetIndex = partialOverlapCorrespondences->at(j).index_match;
            pointT pointInWorld = frameInWorld.at(sourceIndex);
            pointT pointInLocalFrame = framePoints.at(sourceIndex);
            pointT matchedPointMap = cloudMapFull->at(targetIndex);
            //#TODO: Extract points first, then do optimization?

            // Extract points
            auto q_wj = gtsam::Point3(matchedPointMap.x, matchedPointMap.y, matchedPointMap.z);
            auto p_wj = gtsam::Point3(pointInWorld.x, pointInWorld.y, pointInWorld.z);
            auto p_Lij = gtsam::Point3(pointInLocalFrame.x, pointInLocalFrame.y, pointInLocalFrame.z);

            worldPoints[j] = q_wj;
            localPoints[j] = p_Lij;

            gtsam::Matrix3 tmp = - (R_wLi.matrix() * skewSymmetric(p_Lij.x(), p_Lij.y(), p_Lij.z()));

            // Calculate Jacobians
            auto J_hij_TwLi = cv::Mat(pointD, poseD, CV_64F, cv::Scalar::all(0));
            J_hij_TwLi.at<double>(0, 3) = R_wLi.matrix()(0, 0);
            J_hij_TwLi.at<double>(0, 4) = R_wLi.matrix()(0, 1);
            J_hij_TwLi.at<double>(0, 5) = R_wLi.matrix()(0, 2);
            J_hij_TwLi.at<double>(1, 3) = R_wLi.matrix()(1, 0);
            J_hij_TwLi.at<double>(1, 4) = R_wLi.matrix()(1, 1);
            J_hij_TwLi.at<double>(1, 5) = R_wLi.matrix()(1, 2);
            J_hij_TwLi.at<double>(2, 3) = R_wLi.matrix()(2, 0);
            J_hij_TwLi.at<double>(2, 4) = R_wLi.matrix()(2, 1);
            J_hij_TwLi.at<double>(2, 5) = R_wLi.matrix()(2, 2);
            /*J_hij_TwLi.at<double>(0, 0) = 1;
            J_hij_TwLi.at<double>(1, 1) = 1;
            J_hij_TwLi.at<double>(2, 2) = 1;*/
            J_hij_TwLi.at<double>(0, 0) = tmp(0, 0);
            J_hij_TwLi.at<double>(0, 1) = tmp(0, 1);
            J_hij_TwLi.at<double>(0, 2) = tmp(0, 2);
            J_hij_TwLi.at<double>(1, 0) = tmp(1, 0);
            J_hij_TwLi.at<double>(1, 1) = tmp(1, 1);
            J_hij_TwLi.at<double>(1, 2) = tmp(1, 2);
            J_hij_TwLi.at<double>(2, 0) = tmp(2, 0);
            J_hij_TwLi.at<double>(2, 1) = tmp(2, 1);
            J_hij_TwLi.at<double>(2, 2) = tmp(2, 2);

            /*auto J_hij_xwj = cv::Mat(pointD, pointD, CV_64F, cv::Scalar::all(0));
            J_hij_xwj.at<double>(0, 0) = R_wLi.matrix()(0, 0);
            J_hij_xwj.at<double>(0, 1) = R_wLi.matrix()(0, 1);
            J_hij_xwj.at<double>(0, 2) = R_wLi.matrix()(0, 2);
            J_hij_xwj.at<double>(1, 0) = R_wLi.matrix()(1, 0);
            J_hij_xwj.at<double>(1, 1) = R_wLi.matrix()(1, 1);
            J_hij_xwj.at<double>(1, 2) = R_wLi.matrix()(1, 2);
            J_hij_xwj.at<double>(2, 0) = R_wLi.matrix()(2, 0);
            J_hij_xwj.at<double>(2, 1) = R_wLi.matrix()(2, 1);
            J_hij_xwj.at<double>(2, 2) = R_wLi.matrix()(2, 2);*/

            auto e = p_wj - q_wj;
            auto b_ij = cv::Mat(pointD,1,CV_64F,cv::Scalar::all(0));
            b_ij.at<double>(0,0) = -e.x();
            b_ij.at<double>(1,0) = -e.y();
            b_ij.at<double>(2,0) = -e.z();

            // Extract submatrice to insert into
            auto ProwRange = cv::Range(pointD*j, pointD*j + pointD);
            auto PcolRange = cv::Range(0, poseD);
            //auto SrowRange = ProwRange; 
            //auto ScolRange = cv::Range(poseD + pointD*j, poseD + pointD*j + pointD);
            auto bColRange = cv::Range::all();
            auto bRowRange = cv::Range(j*pointD, j*pointD + pointD);
            //auto bRowRange = cv::Range(k*nPoints*poseD + j*poseD, k*nPoints*poseD + j*poseD + poseD);

            cv::Mat PsubMatA = matA.rowRange(ProwRange).colRange(PcolRange);
            //cv::Mat SsubMatA = matA.colRange(ScolRange).rowRange(SrowRange);
            cv::Mat bsubMatB = matB.colRange(bColRange).rowRange(bRowRange);

            // Propagate uncertainty
            cv::Mat sigmasPose(6, 6, CV_64F, cv::Scalar::all(0));
            
            cv::eigen2cv(odometryNoise->covariance(), sigmasPose);
            cv::Mat sigmasPoseFloat;
            sigmasPose.convertTo(sigmasPoseFloat, CV_64F);

            /*cv::Mat sigmasPoints(3, 3, CV_64F, cv::Scalar::all(0));
            cv::eigen2cv(structureNoise->covariance(), sigmasPoints);
            cv::Mat sigmasPointsFloat;
            sigmasPoints.convertTo(sigmasPointsFloat, CV_64F);*/

            cv::Mat whitener = J_hij_TwLi * sigmasPoseFloat * J_hij_TwLi.t();
            //                    + J_hij_xwj * sigmasPoints * J_hij_xwj.t();
            cv::Mat whitenerInv;
            cv::invert(whitener, whitenerInv, cv::DECOMP_SVD);
            cv::Mat whitenerSqrtInv;
            matrix_square_root(whitenerInv, whitenerSqrtInv);

            // Copy into submatrices
            cv::Mat Ai = whitenerSqrtInv * J_hij_TwLi;
            Ai.copyTo(PsubMatA);

            cv::Mat bi = whitenerSqrtInv * b_ij;
            bi.copyTo(bsubMatB);

            //cv::Mat Si = whitenerSqrtInv * J_hij_xwj;
            //Si.copyTo(SsubMatA);
        }
        // Add prior if imu data is available
        if (updateImu){
            cv::Mat priorMatA = matA.rowRange(cv::Range(nPoints*pointD, nPoints*pointD + poseD)).colRange(cv::Range(0, poseD));
            cv::setIdentity(priorMatA);
            cv::Mat priorMatB = matB.rowRange(cv::Range(nPoints*pointD, nPoints*pointD + poseD));
            gtsam::Vector6 prior = - gtsam::Pose3::Logmap(predImuState.pose().inverse() * currentPoseInWorld);
            //auto preintImuCombined = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*preintegrated);
            auto preintImuCombined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*preintegrated);
            //std::cout << "prior before whitening: " << prior << std::endl;
            gtsam::Matrix6 cov = preintImuCombined.preintMeasCov().block<6,6>(0, 0, 6, 6);
            gtsam::Matrix6 whitener = gtsam::inverse_square_root(cov);
            gtsam::Vector6 whitenedPrior = whitener * prior;

            //std::cout << cov << std::endl;

            priorMatB.at<double>(0, 0) = whitenedPrior(0);
            priorMatB.at<double>(1, 0) = whitenedPrior(1);
            priorMatB.at<double>(2, 0) = whitenedPrior(2);
            priorMatB.at<double>(3, 0) = whitenedPrior(3);
            priorMatB.at<double>(4, 0) = whitenedPrior(4);
            priorMatB.at<double>(5, 0) = whitenedPrior(5);
        }

        // Solve using Levenberg Marquardt
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        auto matAtAdiag = cv::Mat::diag(matAtA.diag());
        matAtB = matAt * matB;
        cv::solve(matAtA + (lambda * matAtAdiag), matAtB, matX, cv::DECOMP_QR);

        // Check Update
        gtsam::Pose3 keyPoseBefore = currentPoseInWorld;

        gtsam::Vector6 xi;
        xi << matX.at<double>(0, 0), matX.at<double>(1, 0), matX.at<double>(2, 0), matX.at<double>(3, 0), matX.at<double>(4, 0), matX.at<double>(5, 0);

        gtsam::Pose3 tau = gtsam::Pose3::Expmap(xi);

        gtsam::Pose3 keyPoseAfter = currentPoseInWorld * tau;

        double fxBefore = 0;
        double fxAfter = 0;
        double fxResult = 0;

        for (int i = 0; i < nPoints; i++){
            gtsam::Point3 q_wj = worldPoints[i];
            gtsam::Point3 p_Lij = localPoints[i];

            fxBefore += pow(gtsam::norm3(keyPoseBefore * p_Lij - q_wj), 2);
            fxAfter += pow(gtsam::norm3(keyPoseAfter * p_Lij - q_wj), 2);
        }
        if (fxAfter < fxBefore){
            currentPoseInWorld = keyPoseAfter;
            lambda /= 10;
            fxResult = fxAfter;
        }
        else{
            //ROS_INFO("BAD STEP");
            lambda *= 10;
            fxResult = fxBefore;
        }

        if (fxResult < fxTol || cv::norm(matX) < stepTol) {
            break;
        }
    }
}

void Graph::runOnce(int &runsWithoutUpdate)
{
    if (gnssEnabledFlag && newGnss){
        mtx.lock();
        newGnss = false;
        _preProcessGNSS();
        mtx.unlock();
    }

    if (imuEnabledFlag && newImu && imuFactorAdded && imuInitialized){
        mtx.lock();
        newImu = false;
        _preProcessIMU();
        mtx.unlock();
    }

    if (newLaserOdometry && newMap && newGroundPlane){
        mtx.lock();
        newLaserOdometry, newMap, newGroundPlane = false;

        _incrementPosition();
        // #TODO: PROCESS IMU

        _postProcessIMU(); 
        //_transformMapToWorld();
        //_lateralEstimation();

        _cloud2Map();
        
        _transformToGlobalMap();

        _performIsam();

        _publishTransformed();

        _publishTrajectory();
        
        mtx.unlock();        
        runsWithoutUpdate=0;
    }
    else runsWithoutUpdate++;
}

void Graph::_preProcessGNSS(){
    for (auto it = gnssMeasurements.begin(); it != gnssMeasurements.end(); ++it){
        std::pair<double, gtsam::Pose3> measurementWithStamp = *it;
        if (measurementWithStamp.first > timeOdometry) break;
        for (int j = timeKeyPosePairs.size()-1; j >= 0; --j){
            auto timeKeyPose = timeKeyPosePairs[j];
            std::cout << "Measurement stamp: " << measurementWithStamp.first << " KeyPose stamp: " << timeKeyPose.first << "Time odometry: " << timeOdometry << std::endl;
            if (measurementWithStamp.first > (timeKeyPose.first - delayTol)){
                std::cout << "Stamp: " << it->first << " Pos: " << it->second.translation() << " Keypose #: " << j << std::endl;
                newKeyGnssMeasurementPairs.push_back(std::pair<gtsam::Key, gtsam::Pose3>(X(j), measurementWithStamp.second));
                gnssMeasurements.erase(it);
                break;
            }
        }
    }
}

void Graph::_preProcessIMU(){
    int measurements = imuMeasurements.size();
    //int i = 0;
    for (auto it = imuMeasurements.begin(); it != imuMeasurements.end(); it++){
        std::pair<double, gtsam::Vector6> measurementWithStamp = *it;
        //std::cout << "measurementWithStamp: " << measurementWithStamp.first << " timePrev:" << timePrevPreintegratedImu << " TimeOdometry: "<< timeOdometry << std::endl;
        if (measurementWithStamp.first <= timeOdometry){
            double dt = measurementWithStamp.first - timePrevPreintegratedImu;
            preintegrated->integrateMeasurement(measurementWithStamp.second.head<3>(), measurementWithStamp.second.tail<3>(), dt);
            timePrevPreintegratedImu = measurementWithStamp.first;
            //imuMeasurements.pop_front();
            //i--;
            it = imuMeasurements.erase(it);
            updateImu=true;
        }
        else break;
    }
}

void Graph::_postProcessIMU(){
    if (imuEnabledFlag && !imuInitialized){
        _initializePreintegration();
        newImu = false;
        imuInitialized=true;
        return;
    }

    if(updateImu){
        predImuState = preintegrated->predict(prevImuState, prevImuBias);

        currentPosPoint = pcl::PointXYZ(predImuState.pose().x(), predImuState.pose().y(), predImuState.pose().z());
        /*
        Lowpassfilter trial
        auto currentPoseOnManifold = gtsam::Pose3::Logmap(currentPoseInWorld);
        auto predImuStateOnManifold = gtsam::Pose3::Logmap(predImuState.pose());
        currentPoseInWorld = gtsam::Pose3::Expmap(currentPoseOnManifold + 0.3*(-currentPoseOnManifold + predImuStateOnManifold));
        */
       currentPoseInWorld = predImuState.pose();
    }
}

void Graph::runRefine()
{
    if (smoothingEnabledFlag == false) return;
    ros::Rate rate(0.5);
    while (ros::ok()){
        _mapToGraph();
        //_investigateLoopClosures()
        _publishReworkedMap();
        rate.sleep();
    }
}

void Graph::_transformMapToWorld()
{
    pcl::PointCloud<pointT> currentInWorld;
    pcl::transformPointCloud(*currentFeatureCloud, currentInWorld, currentPoseInWorld.matrix());
    /*for (auto &it : currentInWorld.points){
        octreeMap->addPointToCloud(it, cloudKeyFramesMap);
    }*/
    
    
    if (cloudMapFull->empty()){
        *cloudMapFull += currentInWorld;
        return;
    }
    pcl::PointCloud<pointT>::Ptr currentMap(new pcl::PointCloud<pointT>(*cloudMapFull));
    pcl::KdTreeFLANN<pointT> kdTree;
    kdTree.setInputCloud(cloudMapFull);
    std::vector<int> indices;
    std::vector<float> distances;
    for (auto &it : currentInWorld.points){
        if (!pcl::isFinite<pointT>(it)) continue;

        if (kdTree.nearestKSearch(it, 1, indices, distances) > 0) {
            if (sqrt(distances[0]) > 0.5){
                cloudMapFull->push_back(it);
            }
        }
        else{
            cloudMapFull->push_back(it);
        }
    }
    
}

void Graph::_publishTransformed()
{
    // Publish the entire map
    if (pubTransformedMap.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloudMapFull, msg);
        msg.header.frame_id = "map";
        pubTransformedMap.publish(msg);
    }

    // Publish the newest pose from the ISAM2 estimate
    if (pubTransformedPose.getNumSubscribers() > 0){
        geometry_msgs::PoseWithCovarianceStamped poseWCov;
        poseWCov.header.frame_id = "map";
        poseWCov.header.stamp = timer.fromSec(timeOdometry);

        auto estimate = isamCurrentEstimate.at<gtsam::Pose3>(X(cloudKeyPoses->points.size()-1));
        auto cov = isam->marginalCovariance(X(cloudKeyPoses->points.size()-1));

        poseWCov.pose.pose.position.z   = estimate.z();
        poseWCov.pose.pose.position.y   = estimate.y();
        poseWCov.pose.pose.position.x   = estimate.x();
        poseWCov.pose.pose.orientation.w  = estimate.rotation().toQuaternion().w();
        poseWCov.pose.pose.orientation.x  = estimate.rotation().toQuaternion().x();
        poseWCov.pose.pose.orientation.y  = estimate.rotation().toQuaternion().y();
        poseWCov.pose.pose.orientation.z  = estimate.rotation().toQuaternion().z();
        int row = 0;
        int col = 0;
        std::map<int, int> map = {{0, 3}, {1,4}, {2, 5}, {3, 0}, {4, 1}, {5, 2}};
        for (int i=0; i<36; i++){
            row = i / 6;
            col = i % 6;
            poseWCov.pose.covariance.at(i)= (double) cov(map[row], map[col]);
        }
        pubTransformedPose.publish(poseWCov);
    }
}

void Graph::_publishReworkedMap()
{
    if (pubReworkedMap.getNumSubscribers() > 0){
        pcl::PointCloud<pointT> reworkedMap;
        sensor_msgs::PointCloud2 msg;
        for (auto key : mapKeys){
            auto gtsampoint = isamCurrentEstimate.at<gtsam::Point3>(key.first);
            auto pclpoint = pcl::PointXYZ(gtsampoint.x(), gtsampoint.y(), gtsampoint.z());
            reworkedMap.push_back(pclpoint);
        }
        pcl::toROSMsg(reworkedMap, msg);
        msg.header.frame_id = "map";
        pubReworkedMap.publish(msg);
    }
}

void Graph::_publishTrajectory()
{
    if (!(pubPoseArray.getNumSubscribers() > 0) || cloudKeyPoses->points.size() % 10 != 0) return;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = timer.now();
    poseArray.header.frame_id = "map";
    int poses = cloudKeyPoses->points.size();
    for (int i = 0; i<poses; i++){
        PointXYZRPY it;
        _fromPose3ToPointXYZRPY(isamCurrentEstimate.at<gtsam::Pose3>(X(i)), it);
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

void Graph::writeToFile()
{   
    std::ofstream csvFile("/home/sjurinho/master_ws/src/tunnel_slam/data/LatestRun.csv");
    std::cout << "Failed to open file: " << csvFile.fail() << std::endl;
    csvFile << "key,landmark(x;y;z),pose(x;y;z;r;p;y;cov[36]),velocity(u;v;w),bias(bu;bv;bw;br;bp;by)\n";
    for (auto it : isamCurrentEstimate){
        std::string key = gtsam::DefaultKeyFormatter(it.key);
        std::string row;
        switch (key.front()){
            case 'l':
            {
                auto l = it.value.cast<gtsam::Point3>();
                row = key + "," + std::to_string(l.x()) + ";" + std::to_string(l.y()) + ";" + std::to_string(l.z()) + ",,,";
                break;
            }
            case 'x':
            {
                auto x = it.value.cast<gtsam::Pose3>();
                auto cov = isam->marginalCovariance(it.key);
                std::map<int, int> map = {{0, 3}, {1,4}, {2, 5}, {3, 0}, {4, 1}, {5, 2}};
                
                row = key + ",," + std::to_string(x.translation().x()) + ";" + std::to_string(x.translation().y()) + ";" + std::to_string(x.translation().z()) + ";" + std::to_string(x.rotation().roll()) + ";" + std::to_string(x.rotation().pitch()) + ";" + std::to_string(x.rotation().yaw()) + ";";
                for (int i = 0; i<36; i++){
                    int r = i / 6;
                    int c = i % 6;
                    std::string s;
                    if (i<35) {
                        s = std::to_string((double) cov(map[r], map[c])) + ";";
                    }
                    else{
                        s = std::to_string((double) cov(map[r], map[c]));
                    }
                    row += s;
                }
                row += ",,";
                break;
            }
            case 'v':
            {
                auto vel = it.value.cast<gtsam::Velocity3>();
                row = key + ",,," + std::to_string(vel(0)) + ";" + std::to_string(vel(1)) + ";" + std::to_string(vel(2)) + ",";
                break;
            }
            case 'b':
            {
                auto bias = it.value.cast<gtsam::imuBias::ConstantBias>();
                row = key + ",,,," + std::to_string(bias.accelerometer()(0)) + ";" + std::to_string(bias.accelerometer()(1)) + ";" + std::to_string(bias.accelerometer()(2)) + ";" + std::to_string(bias.gyroscope()(0)) + ";" + std::to_string(bias.gyroscope()(1)) + ";" + std::to_string(bias.gyroscope()(2));
                break;
            }
            default:
                row = "ERROR,,,,";
                break;
        }
        csvFile << row + "\n";
    }
    isam->saveGraph("IsamGraph");
    csvFile.close();
}