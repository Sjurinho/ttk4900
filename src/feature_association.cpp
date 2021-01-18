#include "feature_association.hpp"

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
    std::cout << "fromCallbackFeatureAssociator \n";
}