#include "feature_association.hpp"
//#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>

//constructor method
FeatureAssociation::FeatureAssociation(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
    subPointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 1, &FeatureAssociation::pointCloud2Handler, this);
}

// Destructor method
FeatureAssociation::~FeatureAssociation()
{
    std::cout<<"from Destructor \n";
    // Free up the memory assigned from heap
}

void FeatureAssociation::pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointCloud2Msg, cloud);

    pcl::PointCloud<pcl::PointXYZ> groundPlane;
    pcl::PointCloud<pcl::PointXYZ> excludedGroundPlane;
    _findGroundPlane(cloud, groundPlane, excludedGroundPlane);

    pcl::PointCloud<pcl::PointXYZ> features;
    _extractFeatures(excludedGroundPlane, features);
    std::cout<<features<<std::endl;
}

void FeatureAssociation::_findGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &groundPlane, pcl::PointCloud<pcl::PointXYZ> &excludedGroundPlane)
{
    // Filter to fit plane to points below the sensor

    pcl::PointCloud<pcl::PointXYZ>::Ptr potentialGroundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass(true);
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setNegative(true);
    pass.setFilterLimits(-2, 0);
    pass.filter(*potentialGroundPoints);
    //std::cout << *excludedGround << std::endl;

    pcl::IndicesConstPtr groundIndicesPtr = pass.getRemovedIndices();

    //pcl::PointCloud<pcl::PointXYZ> potentialGroundPoints(cloud, *groundIndicesPtr);
    // Set up RANSAC MODEL

    pcl::SACSegmentation<pcl::PointXYZ> sacGroundPlane;
    sacGroundPlane.setInputCloud(potentialGroundPoints->makeShared());
    sacGroundPlane.setOptimizeCoefficients(true);
    sacGroundPlane.setModelType(pcl::SACMODEL_PLANE);
    sacGroundPlane.setMethodType(pcl::SAC_RANSAC);
    sacGroundPlane.setDistanceThreshold(0.01);

    // Perform fit

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    sacGroundPlane.segment(*inliers, coefficients);
    
    // #TODO: Extract ground plane indices relative to original cloud and output ground plane 

    groundPlane = pcl::PointCloud<pcl::PointXYZ>(cloud, inliers->indices);
    pcl::PassThrough<pcl::PointXYZ> removeGroundPlaneFilter;
    removeGroundPlaneFilter.setInputCloud(excludedGroundPlane.makeShared());
    removeGroundPlaneFilter.setIndices(inliers);
    removeGroundPlaneFilter.setNegative(true); // NOT ENOUGH, need to get inliers relative to cloud
    //removeGroundPlaneFilter.filter();
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
