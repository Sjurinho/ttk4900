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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointCloud2Msg, cloud);

    pcl::RangeImage rangeImage = _pointCloud2RangeImage(cloud);
    std::cout << "RANGE IMAGE\n" << rangeImage << std::endl;

    pcl::PointCloud<pcl::PointXYZ> groundPlane;
    pcl::PointCloud<pcl::PointXYZ> excludedGroundPlane;
    _findGroundPlane(cloud, groundPlane, excludedGroundPlane);
    std::cout << "EXCLUDED GROUND PLANE\n" << excludedGroundPlane << std::endl;
    std::cout << "GROUND PLANE\n" << groundPlane << std::endl;  

    pcl::PointCloud<pcl::PointXYZ> featuresCloud;
    _extractFeatures(excludedGroundPlane, featuresCloud);
    std::cout << "FEATURES CLOUD" << featuresCloud << std::endl;

    _publish(groundPlane, featuresCloud);
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

void FeatureAssociation::_extractFeatures(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &output)
{      

    pcl::PointCloud<pcl::Normal> normals;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud.makeShared());
    normalEstimator.setRadiusSearch(0.8);
    normalEstimator.compute(normals);

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> keypointDetector;
    keypointDetector.setInputCloud(cloud.makeShared());
    keypointDetector.setSalientRadius(0.5);
    keypointDetector.setNonMaxRadius(0.3);
    keypointDetector.setThreshold21(0.9);
    keypointDetector.setThreshold32(0.9);
    keypointDetector.setNormals(normals.makeShared());
    keypointDetector.compute(output);

}

void FeatureAssociation::_publish(const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud, const pcl::PointCloud<pcl::PointXYZ> &featureCloud){
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

pcl::RangeImage FeatureAssociation::_pointCloud2RangeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud){
    
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

void FeatureAssociation::_calculateTransformation(const pcl::PointCloud<pcl::PointXYZ> &groundPlaneCloud, const pcl::PointCloud<pcl::PointXYZ> &featureCloud){}

