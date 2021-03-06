// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef GRAPH //usd for conditional compiling.
#define GRAPH

#include <mutex>
#include <vector>
#include <deque>

#include <ros/ros.h> // including the ros header file

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/NoiseModel.h>
//#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>


// POINT TYPE FOR REGISTERING ENTIRE POSE
typedef pcl::PointXYZ pointT;

struct PointXYZRPY{
    PCL_ADD_POINT4D;
    float roll;
    float pitch;
    float yaw;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRPY, 
                                (float, x, x) (float, y, y) (float, z, z) 
                                (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw));

class Graph
{
    public:
        Graph(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~Graph(); // destructor method
        void odometryHandler(const nav_msgs::OdometryConstPtr &odomMsg);
        void mapHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);
        void groundPlaneHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);
        void imuHandler(const sensor_msgs::ImuConstPtr &imuMsg);
        void gnssHandler(const geometry_msgs::PoseStampedConstPtr &gnssMsg);

        double getCurrentTimeOdometry(void) const { return timeOdometry; }

        void runOnce(int &runsWithoutUpdate);
        void runRefine();
        void runLoopClosure();
        void writeToFile();
    private:
        void _mapToGraph();
        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subOdometry;
        ros::Subscriber subMap, subGroundPlane;
        ros::Subscriber subImu;
        ros::Subscriber subGnss;
        ros::Publisher pubTransformedMap;
        ros::Publisher pubTransformedPose;
        ros::Publisher pubPoseArray;
        ros::Publisher pubReworkedMap;
        ros::Publisher pubCurrentCloudInWorld;
        ros::Publisher pubPotentialLoopCloud;
        ros::Publisher pubLatestKeyFrameCloud;
        ros::Publisher pubICPResultCloud;       
        ros::Time timer;

        // Optimization parameters
        bool smoothingEnabledFlag=true, imuEnabledFlag=true, gnssEnabledFlag=true, loopClosureEnabledFlag=true;
        double voxelRes = 0.1;
        double keyFrameSaveDistance = 3;
        double minCorresponendencesStructure = 30;
        int cloudsInQueue = 0;

        int historyKeyFrameSearchRadius = 20;
        int closestHistoryFrameID = -1;
        int latestFrameIDLoopClosure = 0;
        int historyKeyFrameSearchNum = 3;
        float historyKeyframeFitnessScore = 0.8; // the smaller the better alignment
        bool aLoopIsClosed = false;
        bool potentialLoopFlag = false;

        int maxIterSmoothing = 100;
        float fxTol = 0.05;
        double stepTol = 1e-6;
        double delayTol = 1;

        double* imuComparisonTimerPtr;

        bool imuInitialized=false, newKeyPose = false;
        std::mutex mtx;

        double timeOdometry, timeMap, timePrevPreintegratedImu, timeKeyPose = 0;
        bool newLaserOdometry=false, newMap=false, newGroundPlane=false, newImu=false, updateImu=false, newGnss=false, reinitialize=false;
        // gtsam estimation members
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values initialEstimate, isamCurrentEstimate;
        gtsam::ISAM2 *isam;

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise, odometryNoise, constraintNoise, structureNoise, gnssNoise, loopClosureNoise;

        gtsam::noiseModel::Isotropic::shared_ptr imuVelocityNoise, imuBiasNoise;


        pcl::VoxelGrid<pointT> downSizeFilterMap;
        pcl::PointXYZ previousPosPoint, currentPosPoint;
        pcl::PointCloud<pointT>::Ptr currentFeatureCloud, currentGroundPlaneCloud, latestKeyFrameCloud, nearHistoryKeyFrameCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPositions; // Contains key positions
        pcl::PointCloud<PointXYZRPY>::Ptr cloudKeyPoses; // Contains key poses
        pcl::KdTreeFLANN<pointT>::Ptr kdtreeHistoryKeyPositions;

        std::vector<pcl::PointCloud<pointT>::Ptr> cloudKeyFrames;
        pcl::PointCloud<pointT>::Ptr localKeyFramesMap, cloudMapFull, cloudMapRefined; //For publishing only
        pcl::PointCloud<pcl::PointXYZ>::Ptr reworkedMap;
        pcl::octree::OctreePointCloudSearch<pointT>::Ptr octreeMap;
        std::vector<std::pair<gtsam::Key, int>> mapKeys;

        gtsam::Pose3 currentPoseInWorld, lastPoseInWorld = gtsam::Pose3::identity();
        gtsam::Pose3 displacement;

        std::shared_ptr<gtsam::PreintegrationType> preintegrated;
        gtsam::NavState prevImuState, predImuState;
        gtsam::imuBias::ConstantBias prevImuBias;
        std::deque<std::pair<double, gtsam::Pose3>> odometryMeasurements, timeKeyPosePairs; // [time, measurement]
        std::deque<std::pair<double, gtsam::Vector6>> imuMeasurements; // [time, measurement]
        std::pair<double, gtsam::Point3> gnssMeasurement; // [time, measurement]
        std::deque<std::pair<gtsam::Key, gtsam::Point3>> newKeyGnssMeasurementPairs, keyGnssMeasurementPairs;


        
        void _incrementPosition();
        void _transformMapToWorld();
        void _transformToGlobalMap(); // Adds to the octree structure and fullmap simultaneously
        void _performIsam();
        void _publishTrajectory();
        void _publishTransformed();
        void _fromPointXYZRPYToPose3(const PointXYZRPY &poseIn, gtsam::Pose3 &poseOut);
        void _fromPose3ToPointXYZRPY(const gtsam::Pose3 &poseIn, PointXYZRPY &poseOut);
        void _cloud2Map();
        void _initializePreintegration();
        void _preProcessIMU();
        void _postProcessIMU();
        void _publishReworkedMap();
        void _preProcessGNSS();
        bool _detectLoopClosure();
        bool _performLoopClosure();
        void _performIsamTimedOut();
        void _postProcessImuTimedOut();
};
#endif