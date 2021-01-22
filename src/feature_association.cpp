#include "feature_association.hpp"
//#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/gicp.h>

//constructor method
FeatureAssociation::FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    subPointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 32, &FeatureAssociation::pointCloud2Handler, this);
    pubGroundPlaneCloud2 = nh.advertise<sensor_msgs::PointCloud2>("/groundPlanePointCloud", 32);
    pubFeatureCloud2 = nh.advertise<sensor_msgs::PointCloud2>("/featurePointCloud", 32);
}

// Destructor method
FeatureAssociation::~FeatureAssociation()
{

}

void FeatureAssociation::pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{   
    std::cout << "prevclouds.size()\n" << _prevFeatureCloud.empty() << std::endl;
    std::cout << _prevFeatureDescriptor.empty() << std::endl;
    std::cout << _prevGroundPlaneCloud.empty() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> rawCloud;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<int> indices;
    pcl::fromROSMsg(*pointCloud2Msg, rawCloud);

    pcl::removeNaNFromPointCloud(rawCloud, cloud, indices);

    std::cout << "INCLOUD\n" << cloud << std::endl;

    pcl::PointCloud<pcl::PointXYZ> groundPlane;
    pcl::PointCloud<pcl::PointXYZ> excludedGroundPlane;
    _findGroundPlane(cloud, groundPlane, excludedGroundPlane);
    std::cout << "EXCLUDED GROUND PLANE\n" << excludedGroundPlane << std::endl;
    std::cout << "GROUND PLANE\n" << groundPlane << std::endl;  

    pcl::PointCloud<pcl::PointXYZ> featureCloud;
    pcl::PointCloud<pcl::FPFHSignature33> featureDescriptors;
    pcl::PointCloud<pcl::Normal> featureNormals;
    _extractFeatures(excludedGroundPlane, featureCloud, featureDescriptors, featureNormals);
    std::cout << "FEATURES CLOUD\n" << featureCloud << std::endl;
    std::cout << "FEATURES DESCRIPTORS\n" << featureDescriptors << std::endl;

    if (_prevFeatureCloud.empty() || _prevFeatureDescriptor.empty() || _prevGroundPlaneCloud.empty() || _prevNormals.empty() ) {
        std::cout << "INITIALIZING PREVIOUS" << std::endl;
        _prevFeatureCloud       = featureCloud;
        _prevFeatureDescriptor  = featureDescriptors;
        _prevNormals            = featureNormals;
        _prevGroundPlaneCloud   = groundPlane;

    }
    else {

        _calculateTransformation(groundPlane, featureCloud, featureDescriptors);

        _publish(_prevFeatureCloud, featureCloud);

        _prevFeatureCloud       = featureCloud;
        _prevFeatureDescriptor  = featureDescriptors;
        _prevNormals            = featureNormals;
        _prevGroundPlaneCloud   = groundPlane;
    }
}

void FeatureAssociation::_findGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &groundPlane, pcl::PointCloud<pcl::PointXYZ> &excludedGroundPlane)
{
    // Filter to fit plane to points below the sensor

    pcl::PointCloud<pcl::PointXYZ>::Ptr potentialGroundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass(true);
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setNegative(false);
    pass.setFilterLimits(-2, -0.1);
    pass.filter(*potentialGroundPoints);
    //std::cout << *excludedGround << std::endl;

    pcl::IndicesConstPtr notGroundPtr = pass.getRemovedIndices();

    //pcl::PointCloud<pcl::PointXYZ> potentialGroundPoints(cloud, *notGroundPtr);
    // Set up RANSAC MODEL

    pcl::SACSegmentation<pcl::PointXYZ> sacGroundPlane;
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

    groundPlane = pcl::PointCloud<pcl::PointXYZ>(*potentialGroundPoints, inliers->indices);

    pcl::ExtractIndices<pcl::PointXYZ> removeGroundPlaneFilter;
    removeGroundPlaneFilter.setInputCloud(cloud.makeShared());
    removeGroundPlaneFilter.setIndices(notGroundPtr);
    removeGroundPlaneFilter.setNegative(false);
    removeGroundPlaneFilter.filter(excludedGroundPlane);

    /*pcl::PointIndices::Ptr originalGroundPlaneIndices(new pcl::PointIndices());
    for (auto it : inliers->indices){
        originalGroundPlaneIndices->indices.emplace_back((*notGroundPtr)[it]);
    }
    //std::cout << "removed indices from original cloud: " << originalGroundPlaneIndices->indices.size() << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> removeGroundPlaneFilter(true);
    removeGroundPlaneFilter.setInputCloud(cloud.makeShared());
    removeGroundPlaneFilter.setIndices(originalGroundPlaneIndices);
    removeGroundPlaneFilter.setNegative(false);
    removeGroundPlaneFilter.filter(excludedGroundPlane);*/

    //std::cout << "Original Cloud\n " << cloud << std::endl;  

}

void FeatureAssociation::_extractFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &output, pcl::PointCloud<pcl::FPFHSignature33> &descriptors, pcl::PointCloud<pcl::Normal> &normals)
{      
    //Calculate normals
    pcl::PointCloud<pcl::Normal> fullCloudNormals;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud.makeShared());
    normalEstimator.setRadiusSearch(0.4);
    normalEstimator.compute(fullCloudNormals);

    //Extract keypoints
    
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> keypointDetector;
    keypointDetector.setInputCloud(cloud.makeShared());
    keypointDetector.setSalientRadius(0.6);
    keypointDetector.setNonMaxRadius(0.6);
    keypointDetector.setThreshold21(0.3);
    keypointDetector.setThreshold32(0.3);
    keypointDetector.setNormals(fullCloudNormals.makeShared());
    keypointDetector.compute(output);

    //Calculate FPFH descriptors
    pcl::PointCloud<pcl::FPFHSignature33> fullCloudDescriptors;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimator;
    fpfhEstimator.setInputCloud(cloud.makeShared());
    fpfhEstimator.setInputNormals(fullCloudNormals.makeShared());
    fpfhEstimator.setSearchMethod(tree);
    fpfhEstimator.setRadiusSearch(0.5);
    fpfhEstimator.compute(fullCloudDescriptors);

    //Only extract descriptors from the keypoints
    pcl::ExtractIndices<pcl::FPFHSignature33> keypointExtractor;
    keypointExtractor.setInputCloud(fullCloudDescriptors.makeShared());
    keypointExtractor.setIndices(keypointDetector.getKeypointsIndices());
    keypointExtractor.filter(descriptors);

    //Only extract normals from the keypoints
    pcl::ExtractIndices<pcl::Normal> normalExtractor;
    normalExtractor.setInputCloud(fullCloudNormals.makeShared());
    normalExtractor.setIndices(keypointDetector.getKeypointsIndices());
    normalExtractor.filter(normals);
}

void FeatureAssociation::_publish(const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud, const pcl::PointCloud<pcl::PointXYZ> &featureCloud)
{
    if (pubGroundPlaneCloud2.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(groundPlaneCloud, msg);
        pubGroundPlaneCloud2.publish(msg);
    }
    if (pubFeatureCloud2.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(featureCloud, msg);
        pubFeatureCloud2.publish(msg);
    }
}

pcl::RangeImage FeatureAssociation::_pointCloud2RangeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud)
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

void FeatureAssociation::_calculateTransformation(const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud, const pcl::PointCloud<pcl::PointXYZ> &featureCloud, const pcl::PointCloud<pcl::FPFHSignature33> &featureDescriptors)
{
    // MATCH CORRESPONDENCES
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> matcher;
    matcher.setInputSource(_prevFeatureDescriptor.makeShared());
    matcher.setInputTarget(featureDescriptors.makeShared());

    pcl::CorrespondencesPtr allCorrespondences(new pcl::Correspondences);
    matcher.determineReciprocalCorrespondences(*allCorrespondences);

    std::cout << "MATCHES SIZE: " << allCorrespondences->size() << std::endl;
    

    // #TODO: REJECT CORRESPONDENCES
    pcl::CorrespondencesPtr goodCorrespondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rej;
    rej.setInputSource(_prevFeatureCloud.makeShared());
    rej.setInputTarget(featureCloud.makeShared());
    rej.setInlierThreshold(1.5);
    rej.setMaximumIterations(1000);
    rej.setRefineModel(true);
    rej.setInputCorrespondences(allCorrespondences);
    rej.getCorrespondences(*goodCorrespondences);

    std::cout << "GOOD MATCHES SIZE: " << goodCorrespondences->size() << std::endl;



    /*pcl::registration::CorrespondenceRejectorFeatures correspondenceRejector;
    correspondenceRejector.setSourcePoints(source);
    correspondenceRejector.setInputCorrespondences(allCorrespondences);
    correspondenceRejector.getCorrespondences(*goodCorrespondences);

    std::cout << "AFTER REJECTION MATCHES: " <<goodCorrespondences->size() << std::endl;*/
    
    //Calculate transformation
    /*pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ> tEst;
    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T;

    tEst.estimateRigidTransformation(_prevFeatureCloud, featureCloud, *goodCorrespondences, T);*/
    pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> lm;
    pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T;

    lm.estimateRigidTransformation(_prevFeatureCloud, featureCloud, *goodCorrespondences, T);
    std::cout<< "Initial transformation\n" << T << std::endl;

    /*pcl::PointCloud<pcl::PointXYZ> prev2currentAligned;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(_prevFeatureCloud.makeShared());
    gicp.setInputTarget(featureCloud.makeShared());
    gicp.align(prev2currentAligned, T);
    Eigen::Matrix4f TFinal = gicp.getFinalTransformation();
    
    std::cout << "Final transformation\n" << TFinal << std::endl;*/

}

