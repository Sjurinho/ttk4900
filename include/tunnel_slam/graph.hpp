// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef GRAPH //usd for conditional compiling.
#define GRAPH

#include <mutex>
#include <vector>

#include <ros/ros.h> // including the ros header file

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/NoiseModel.h>
//#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

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
        void runOnce();
        void runSmoothing();
    private:
        void _smoothPoses();
        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subOdometry;
        ros::Subscriber subMap;
        ros::Publisher pubTransformedMap;
        ros::Publisher pubTransformedPose;
        ros::Publisher pubPoseArray;


        // Optimization parameters
        bool smoothingEnabledFlag=true;
        double voxelRes = 0.3;
        int smoothingFrames = 5;

        int maxIterSmoothing = 40;
        float fxtol = 0.05;


        // gtsam estimation members
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values initialEstimate, isamCurrentEstimate;
        gtsam::ISAM2* isam;

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise, odometryNoise, constraintNoise;

        std::mutex mtx;
        pcl::PointXYZ previousPosPoint, currentPosPoint;
        pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPositions; // Contains key positions
        pcl::PointCloud<PointXYZRPY>::Ptr cloudKeyPoses; // Contains key poses

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudKeyFrames;
        pcl::PointCloud<pcl::PointXYZ>::Ptr localKeyFramesMap, fullMap; //For publishing only
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr cloudMap;

        double disp[6] = { 0 }; // [roll, pitch, yaw, x, y, z]
        gtsam::Pose3 currentPoseInWorld, lastPoseInWorld = gtsam::Pose3::identity();
        gtsam::Pose3 displacement;

        double timeOdometry, timeMap = 0;
        bool newLaserOdometry, newMap = false;

        void _incrementPosition();
        void _transformMapToWorld();
        void _createKeyFramesMap();
        void _performIsam();
        void _publishTrajectory();
        void _publishTransformed();
        void _fromPointXYZRPYToPose3(const PointXYZRPY &poseIn, gtsam::Pose3 &poseOut);
        void _fromPose3ToPointXYZRPY(const gtsam::Pose3 &poseIn, PointXYZRPY &poseOut);
        void _evaluate_transformation(int minNrOfPoints, int latestFrame, const std::vector<PointXYZRPY>& posesBefore, const std::vector<PointXYZRPY>& posesAfter, const std::vector<gtsam::Point3> &pointsWorld, const std::vector<gtsam::Point3> &pointsLocal, float &resultBefore, float &resultAfter);
        void _applyUpdate(std::vector<PointXYZRPY> keyPoses, int latestFrame);
        void _updateIsam();
};
#endif