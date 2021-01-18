// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef TUNNEL_SLAM //usd for conditional compiling.
#define TUNNEL_SLAM
#include <ros/ros.h> // including the ros header file
#include <sensor_msgs/PointCloud2.h>

/* defining the class */
class TunnelSlam
{
    public:
        TunnelSlam(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~TunnelSlam(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
        void pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);
    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subPointCloud2;
};
#endif  