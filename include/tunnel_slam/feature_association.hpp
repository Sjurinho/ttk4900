// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef FEATURE_ASSOCIATION //usd for conditional compiling.
#define FEATURE_ASSOCIATION

#include <ros/ros.h> // including the ros header file
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <tf/transform_broadcaster.h>

class FeatureAssociation
{
    public:
        FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~FeatureAssociation(); // destructor method
        void runOnce();
    private:
        // Callbacks
        void pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg);

        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subPointCloud2;
        ros::Publisher pubFeatureCloud2;
        ros::Publisher pubGroundPlaneCloud2;
        ros::Publisher pubOdometry;
        tf::TransformBroadcaster odomBroadcaster;

        // Previous point clouds
        pcl::PointCloud<pcl::PointXYZ> _prevFeatureCloud = pcl::PointCloud<pcl::PointXYZ>();
        pcl::PointCloud<pcl::FPFHSignature33> _prevFeatureDescriptor = pcl::PointCloud<pcl::FPFHSignature33>();
        pcl::PointCloud<pcl::PointXYZ> _prevGroundPlaneCloud = pcl::PointCloud<pcl::PointXYZ>();

        //Transformation
        Eigen::Affine3d transformation;
        ros::Time currentTime, prevTime;

        bool newCloud = false;
        pcl::PointCloud<pcl::PointXYZ> currentCloud;

        pcl::RangeImage _pointCloud2RangeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud);

        // Feature extraction
        void _findGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &groundPlane, pcl::PointCloud<pcl::PointXYZ> &excludedGroundPlane);
        void _extractFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &output, pcl::PointCloud<pcl::FPFHSignature33> &descriptors, float leafSize=0.1);
        void _publish(const pcl::PointCloud<pcl::PointXYZ> &featureCloud, const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud);

        //Transformation calculations
        void _warpPoints(); // #TODO
        void _calculateTransformation(const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud, const pcl::PointCloud<pcl::PointXYZ> &featureCloud, const pcl::PointCloud<pcl::FPFHSignature33> &featureDescriptors);

        void _publishTransformation();
        void _publishFeatureCloud(const pcl::PointCloud<pcl::PointXYZ> &featureCloud, const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud);

};
#endif