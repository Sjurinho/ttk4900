// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef GRAPH //usd for conditional compiling.
#define GRAPH

#include <mutex>
#include <vector>

#include <ros/ros.h> // including the ros header file

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
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
    private:
        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subOdometry;
        ros::Subscriber subMap;
        ros::Publisher pubTransformedMap;
        ros::Publisher pubTransformedPose;


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

        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudKeyFrames;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyFramesMap; //For publishing only


        double disp[6] = { 0 }; // [roll, pitch, yaw, x, y, z]
        gtsam::Pose3 currentPoseInWorld, lastPoseInWorld = gtsam::Pose3::identity();
        gtsam::Pose3 displacement;

        double timeOdometry, timeMap = 0;
        bool newLaserOdometry, newMap = false;

        void _incrementPosition();
        void _smoothPoses();
        void _performIsam();
        void _publishTransformed();
};
#endif