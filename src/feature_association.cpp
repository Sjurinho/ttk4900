#include "feature_association.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/gicp.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>


//constructor method
FeatureAssociation::FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{   
    nh_ = nh;
    ROS_INFO("Initializing Feature Association Node");

    // Subscribers and publishers
    subPointCloud2          = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 32, &FeatureAssociation::pointCloud2Handler, this);
    pubGroundPlaneCloud2    = nh.advertise<sensor_msgs::PointCloud2>("/groundPlanePointCloud", 32);
    pubFeatureCloud2        = nh.advertise<sensor_msgs::PointCloud2>("/featurePointCloud", 32);
    pubOdometry             = nh.advertise<nav_msgs::Odometry>("/lidarOdom", 32);

    // Variable initialization
    prevTime = ros::Time::now();
}

// Destructor method
FeatureAssociation::~FeatureAssociation()
{

}

void FeatureAssociation::pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{   
        pcl::fromROSMsg(*pointCloud2Msg, currentCloud);
        currentTime = pointCloud2Msg->header.stamp;
        newCloud=true;
}

void FeatureAssociation::_findGroundPlane(const pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<pcl::PointNormal> &groundPlane, pcl::PointCloud<pcl::PointNormal> &excludedGroundPlane)
{
    // Filter to fit plane to points below the sensor

    pcl::PointCloud<pcl::PointNormal>::Ptr potentialGroundPoints(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PassThrough<pcl::PointNormal> pass(true);
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setNegative(false);
    pass.setFilterLimits(-2, -0.1);
    pass.filter(*potentialGroundPoints);
    //std::cout << *excludedGround << std::endl;

    pcl::IndicesConstPtr notGroundPtr = pass.getRemovedIndices();

    pcl::SACSegmentation<pcl::PointNormal> sacGroundPlane;
    sacGroundPlane.setInputCloud(potentialGroundPoints->makeShared());
    sacGroundPlane.setOptimizeCoefficients(true);
    sacGroundPlane.setModelType(pcl::SACMODEL_PLANE);
    sacGroundPlane.setMethodType(pcl::SAC_RANSAC);
    sacGroundPlane.setDistanceThreshold(0.1);

    // Perform fit
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    sacGroundPlane.segment(*inliers, coefficients);
    
    // #TODO: Extract ground plane indices relative to original cloud and output ground plane 

    groundPlane = pcl::PointCloud<pcl::PointNormal>(*potentialGroundPoints, inliers->indices);
    
    pcl::ExtractIndices<pcl::PointNormal> removeGroundPlaneFilter;
    removeGroundPlaneFilter.setInputCloud(cloud.makeShared());
    removeGroundPlaneFilter.setIndices(notGroundPtr);
    removeGroundPlaneFilter.setNegative(false);
    removeGroundPlaneFilter.filter(excludedGroundPlane); 

}

void FeatureAssociation::_extractFeatures(const pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<pcl::PointNormal> &output, pcl::PointCloud<pcl::FPFHSignature33> &descriptors)
{      

    //Extract keypoints
    pcl::ISSKeypoint3D<pcl::PointNormal, pcl::PointNormal> keypointDetector; // Possible to do this after processing if you pass original cloud to setsearchsurface()
    keypointDetector.setInputCloud(cloud.makeShared());
    keypointDetector.setSalientRadius(leafSize*5);
    keypointDetector.setNonMaxRadius(leafSize*3);
    keypointDetector.setThreshold21(0.8);
    keypointDetector.setThreshold32(0.8);
    keypointDetector.setNormalRadius(normalRadius); //#TODO: find a way to use precalculated normals
    //keypointDetector.setNormals(test->);
    keypointDetector.compute(output);

    //Calculate FPFH descriptors
    pcl::PointCloud<pcl::FPFHSignature33> fullCloudDescriptors;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfhEstimator;
    fpfhEstimator.setInputCloud(cloud.makeShared());
    fpfhEstimator.setInputNormals(cloud.makeShared());
    fpfhEstimator.setSearchMethod(tree);
    fpfhEstimator.setRadiusSearch(normalRadius*2);
    fpfhEstimator.compute(fullCloudDescriptors);

    //Only extract descriptors from the keypoints
    pcl::ExtractIndices<pcl::FPFHSignature33> keypointExtractor;
    keypointExtractor.setInputCloud(fullCloudDescriptors.makeShared());
    keypointExtractor.setIndices(keypointDetector.getKeypointsIndices());
    keypointExtractor.filter(descriptors);
}

pcl::RangeImage FeatureAssociation::_pointCloud2RangeImage(const pcl::PointCloud<pcl::PointNormal> &cloud)
{
    
    // SENSOR SPECIFIC

    float angularResolutionX    = (float) (pcl::deg2rad(0.16));
    float angularResolutionY    = (float) (pcl::deg2rad(1.25));
    float maxAngleWidth         = (float)  (pcl::deg2rad((float)180));
    float maxAngleHeight        = (float) (pcl::deg2rad((float)40));

    // Navigation parameters 

    Eigen::Affine3f sensorPose  = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

    float noiseLevel    = 0.00;
    float minRange      = 0.0f;
    int borderSize      = 1;

    // Create Range Image

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(cloud, angularResolutionX, angularResolutionY, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    return rangeImage;

}

void FeatureAssociation::_warpPoints() //#TODO
{
    double timeDiff = currentTime.toSec() - prevTime.toSec();
    double zPrev = transformation.rotation().eulerAngles(0, 1, 2).z();
}

void FeatureAssociation::_calculateTransformation(const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud, const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::FPFHSignature33> &featureDescriptors)
{   
    
    // Find correspondences
    pcl::CorrespondencesPtr allCorrespondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> matcher;
    
    matcher.setInputSource(featureDescriptors.makeShared());
    matcher.setInputTarget(_prevFeatureDescriptor.makeShared());
    matcher.determineReciprocalCorrespondences(*allCorrespondences);    

    // Rejection step
    pcl::CorrespondencesPtr partialOverlapCorrespondences (new pcl::Correspondences);

    pcl::CorrespondencesPtr goodCorrespondences (new pcl::Correspondences);

    pcl::registration::CorrespondenceRejectorTrimmed trimmer;
    trimmer.setInputCorrespondences(allCorrespondences);
    trimmer.setOverlapRatio(0.4);
    trimmer.getCorrespondences(*partialOverlapCorrespondences);


    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> rej;
    rej.setInputSource(featureCloud.makeShared());
    rej.setInputTarget(_prevFeatureCloud.makeShared());
    rej.setInlierThreshold(0.5);
    rej.setMaximumIterations(50);
    rej.setRefineModel(true);
    rej.setInputCorrespondences(partialOverlapCorrespondences);
    rej.getCorrespondences(*goodCorrespondences);

    //Calculate transformation
    //pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> tEst;
    pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal> tEst;
    //pcl::registration::TransformationEstimationLM<pcl::PointNormal, pcl::PointNormal> tEst;
    Eigen::Matrix4f T;


    tEst.estimateRigidTransformation(featureCloud, _prevFeatureCloud, *goodCorrespondences, T);

    /*Eigen::Matrix4f TFinal;
    pcl::PointCloud<pcl::PointNormal> alignedCloud;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp;
    gicp.setInputSource(featureCloud.makeShared());
    gicp.setInputTarget(_prevFeatureCloud.makeShared());
    gicp.align(alignedCloud, T);
    TFinal = gicp.getFinalTransformation();*/



    //TFinal = T;
    //Eigen::Affine3f t; t = T;
    transformation = T.cast<double>();
}

void FeatureAssociation::_publishTransformation()
{   
    if (pubOdometry.getNumSubscribers() > 0){

        double delta_x = transformation.translation().x();
        double delta_y = transformation.translation().y();
        double delta_z = transformation.translation().z();

        geometry_msgs::TransformStamped tfMsg   = tf2::eigenToTransform(transformation);
        tfMsg.header.stamp                      = currentTime;
        tfMsg.header.frame_id                   = "odom";
        tfMsg.child_frame_id                    = "map";

        odomBroadcaster.sendTransform(tfMsg);

        nav_msgs::Odometry odom;
        odom.header.stamp       = currentTime;
        odom.header.frame_id    = "odom";
        
        odom.pose.pose.position.x   = delta_x;
        odom.pose.pose.position.y   = delta_y;
        odom.pose.pose.position.z   = delta_z;
        odom.pose.pose.orientation  = tfMsg.transform.rotation;

        odom.child_frame_id = "map";

        pubOdometry.publish(odom);
    }

}

void FeatureAssociation::_publishFeatureCloud(const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud)
{
    if (pubFeatureCloud2.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(featureCloud, msg);
        msg.header.stamp = currentTime;
        msg.header.frame_id = "lidar";
        pubFeatureCloud2.publish(msg);
    }
    if (pubGroundPlaneCloud2.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(groundPlaneCloud, msg);
        msg.header.stamp = currentTime;
        msg.header.frame_id = "lidar";
        pubGroundPlaneCloud2.publish(msg);
    }
}

void FeatureAssociation::_publish(const pcl::PointCloud<pcl::PointNormal> &featureCloud, const pcl::PointCloud<pcl::PointNormal> &groundPlaneCloud)
{   

    _publishFeatureCloud(featureCloud, groundPlaneCloud);
    _publishTransformation();

}

void FeatureAssociation::_calculateNormals(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointNormal> &cloudWithNormals)
{
    //Calculate normals
    pcl::PointCloud<pcl::Normal> fullCloudNormals;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud.makeShared());
    normalEstimator.setRadiusSearch(normalRadius);
    normalEstimator.compute(fullCloudNormals);

    for (int i = 0; i < cloud.points.size(); i++){
        pcl::PointNormal pointWithNormal;
        pointWithNormal.x = cloud.points[i].x;
        pointWithNormal.y = cloud.points[i].y;
        pointWithNormal.z = cloud.points[i].z;
        pointWithNormal.normal_x = fullCloudNormals.points[i].normal_x;
        pointWithNormal.normal_y = fullCloudNormals.points[i].normal_y;        pointWithNormal.normal_z = fullCloudNormals.points[i].normal_z;
        cloudWithNormals.push_back(pointWithNormal);
    }
}

void FeatureAssociation::runOnce()
{   
    if (newCloud){

        newCloud=false;

        pcl::PointCloud<pcl::PointXYZ> removedNansCloud;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(currentCloud, removedNansCloud, indices);

        pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
        voxelGridFilter.setInputCloud(removedNansCloud.makeShared());
        voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
        voxelGridFilter.filter(cloud);

        pcl::PointCloud<pcl::PointNormal> cloudWithNormals;
        _calculateNormals(cloud, cloudWithNormals);


        //std::cout << "INCLOUD\n" << cloud << std::endl;

        pcl::PointCloud<pcl::PointNormal> groundPlane;
        pcl::PointCloud<pcl::PointNormal> excludedGroundPlane;
        _findGroundPlane(cloudWithNormals, groundPlane, excludedGroundPlane);
        //std::cout << "EXCLUDED GROUND PLANE\n" << excludedGroundPlane << std::endl;
        //std::cout << "GROUND PLANE\n" << groundPlane << std::endl;  

        pcl::PointCloud<pcl::PointNormal> featureCloud;
        pcl::PointCloud<pcl::FPFHSignature33> featureDescriptors;
        _extractFeatures(excludedGroundPlane, featureCloud, featureDescriptors);
        //std::cout << "FEATURES CLOUD\n" << featureCloud << std::endl;
        //std::cout << "FEATURES DESCRIPTORS\n" << featureDescriptors << std::endl;
        if (featureDescriptors.points.size() < minNrOfFeatures){
            return;
        }
        if (_prevFeatureCloud.empty() || _prevFeatureDescriptor.empty() || _prevGroundPlaneCloud.empty() ) {
            std::cout << "INITIALIZING PREVIOUS" << std::endl;
            _prevFeatureCloud       = featureCloud;
            _prevFeatureDescriptor  = featureDescriptors;
            _prevGroundPlaneCloud   = groundPlane;
            prevTime                = currentTime;

        }
        else {

            _calculateTransformation(groundPlane, featureCloud, featureDescriptors);

            _publish(featureCloud, groundPlane);

            _prevFeatureCloud       = featureCloud;
            _prevFeatureDescriptor  = featureDescriptors;
            _prevGroundPlaneCloud   = groundPlane;
            prevTime                = currentTime;

        }
    }
}