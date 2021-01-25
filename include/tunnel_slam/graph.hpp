// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef GRAPH //usd for conditional compiling.
#define GRAPH
#include <ros/ros.h> // including the ros header file
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

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

        // GRAPH members
        gtsam::NonlinearFactorGraph _graph;
        gtsam::Values initialEstimate, optimizedEstimate, isamCurrentEstimate;
        gtsam::ISAM2* isam;

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise, odometryNoise, constraintNoise;

        std::mutex mtx;

        int latestFrameID;

        pcl::PointXYZ previousPosPoint, currentPosPoint;

        double disp[6]; //[x, y, z, roll, pitch, yaw]

        double timeOdometry, timeMap = 0;
        bool newLaserOdometry, newMap = false;
};
#endif