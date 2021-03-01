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

        float leafSize = 0.2;
        float normalRadius = leafSize*2.5;

        // ROS Members
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber subPointCloud2;
        ros::Publisher pubFeatureCloud2;
        ros::Publisher pubGroundPlaneCloud2;
        ros::Publisher pubOdometry;
        tf::TransformBroadcaster odomBroadcaster;

        // Previous point clouds
        pcl::PointCloud<pcl::PointNormal> _prevFeatureCloud = pcl::PointCloud<pcl::PointNormal>();
        pcl::PointCloud<pcl::FPFHSignature33> _prevFeatureDescriptor = pcl::PointCloud<pcl::FPFHSignature33>();
        pcl::PointCloud<pcl::PointNormal> _prevGroundPlaneCloud = pcl::PointCloud<pcl::PointNormal>();

        //Transformation
        Eigen::Affine3d transformation;
        ros::Time currentTime, prevTime;

        bool newCloud = false;
        pcl::PointCloud<pcl::PointXYZ> currentCloud;

        pcl::RangeImage _pointCloud2RangeImage(const pcl::PointCloud<pcl::PointNormal> &cloud);

        // Feature extraction
        void _calculateNormals(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointNormal> &cloudWithNormals);
        void _findGroundPlane(const pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<pcl::PointNormal> &groundPlane, pcl::PointCloud<pcl::PointNormal> &excludedGroundPlane);
        void _extractFeatures(const pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<pcl::PointNormal> &output, pcl::PointCloud<pcl::FPFHSignature33> &descriptors);
        void _publish(const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud);

        //Transformation calculations
        void _warpPoints(); // #TODO
        void _calculateTransformation(const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud, const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::FPFHSignature33> &featureDescriptors);

        void _publishTransformation();
        void _publishFeatureCloud(const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud);
};
#endif