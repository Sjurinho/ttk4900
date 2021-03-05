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
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

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

        void runOnce();
        void runSmoothing();
    private:
        void _smoothPoses();
        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subOdometry;
        ros::Subscriber subMap, subGroundPlane;
        ros::Subscriber subImu;
        ros::Publisher pubTransformedMap;
        ros::Publisher pubTransformedPose;
        ros::Publisher pubPoseArray;

        // Optimization parameters
        bool smoothingEnabledFlag=false;
        double voxelRes = 0.3;
        int smoothingFrames = 10;

        int maxIterSmoothing = 20;
        float fxTol = 0.05;
        double stepTol = 1e-15;


        // gtsam estimation members
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values initialEstimate, isamCurrentEstimate;
        gtsam::ISAM2* isam;

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise, odometryNoise, constraintNoise, imuPoseNoise;

        gtsam::noiseModel::Isotropic::shared_ptr imuVelocityNoise, imuBiasNoise;


        pcl::VoxelGrid<pcl::PointNormal> downSizeFilterSurroundingKeyPoses;
        std::mutex mtx;
        pcl::PointXYZ previousPosPoint, currentPosPoint;
        pcl::PointCloud<pcl::PointNormal>::Ptr currentFeatureCloud, currentGroundPlaneCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPositions; // Contains key positions
        pcl::PointCloud<PointXYZRPY>::Ptr cloudKeyPoses; // Contains key poses

        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cloudKeyFrames;
        pcl::PointCloud<pcl::PointNormal>::Ptr localKeyFramesMap, cloudMapFull; //For publishing only
        pcl::octree::OctreePointCloudSearch<pcl::PointNormal>::Ptr octreeMap;


        double disp[6] = { 0 }; // [roll, pitch, yaw, x, y, z]
        gtsam::Pose3 currentPoseInWorld, lastPoseInWorld = gtsam::Pose3::identity();
        gtsam::Pose3 displacement;

        std::shared_ptr<gtsam::PreintegrationType> preintegrated;
        gtsam::NavState prevImuState, predImuState;
        gtsam::imuBias::ConstantBias prevImuBias;
        std::deque<std::pair<double, gtsam::Vector6>> imuMeasurements;

        double timeOdometry, timeMap, timePrevPreintegratedImu = 0;
        bool newLaserOdometry, newMap, newGroundPlane, newImu, updateImu = false;

        void _incrementPosition();
        void _lateralEstimation();
        void _transformMapToWorld();
        void _transformToGlobalMap(); // Adds to the octree structure and fullmap simultaneously
        void _createKeyFramesMap();
        void _performIsam();
        void _publishTrajectory();
        void _publishTransformed();
        void _fromPointXYZRPYToPose3(const PointXYZRPY &poseIn, gtsam::Pose3 &poseOut);
        void _fromPose3ToPointXYZRPY(const gtsam::Pose3 &poseIn, PointXYZRPY &poseOut);
        void _evaluate_transformation(int minNrOfPoints, int latestFrame, const std::vector<PointXYZRPY>& posesBefore, const std::vector<PointXYZRPY>& posesAfter, const std::vector<gtsam::Point3> &pointsWorld, const std::vector<gtsam::Point3> &pointsLocal, double &resultBefore, double &resultAfter);
        void _applyUpdate(std::vector<PointXYZRPY> keyPoses, int latestFrame);
        void _cloud2Map();
        void _initializePreintegration();
        void _preintegrateImuMeasurements();
};
#endif