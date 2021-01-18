// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef FEATURE_ASSOCIATION //usd for conditional compiling.
#define FEATURE_ASSOCIATION
#include <ros/ros.h> // including the ros header file
#include <sensor_msgs/PointCloud2.h>

class FeatureAssociation
{
    public:
        FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~FeatureAssociation(); // destructor method
        void pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);

    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subPointCloud2;
};
#endif