// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef FEATURE_ASSOCIATION //usd for conditional compiling.
#define FEATURE_ASSOCIATION
#include <ros/ros.h> // including the ros header file
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class FeatureAssociation
{
    public:
        FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~FeatureAssociation(); // destructor method
        void pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);
    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subPointCloud2;
        pcl::PointCloud<pcl::PointXYZ> _prevPointCloud2;

        void _findGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &groundPlane, pcl::PointCloud<pcl::PointXYZ> &excludedGroundPlane);
        void _extractFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &output);

};
#endif