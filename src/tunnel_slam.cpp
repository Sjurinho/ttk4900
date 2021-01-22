#include "tunnel_slam.hpp"

//constructor method
TunnelSlam::TunnelSlam(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
    subPointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 1, &TunnelSlam::pointCloud2Handler, this);
}

// Destructor method
TunnelSlam::~TunnelSlam()
{
    std::cout<<"from Destructor \n";
    // Free up the memory assigned from heap
}

void TunnelSlam::runOnce()
{
    std::cout<<"from Runonce \n";
}

void TunnelSlam::pointCloud2Handler(const sensor_msgs::PointCloud2ConstPtr& pointCloud2Msg)
{
}