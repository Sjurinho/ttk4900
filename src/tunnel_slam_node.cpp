// Node file to create object and initialising the ROS node
#include "tunnel_slam.hpp"

int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    ros::init(argc, argv, "tunnel_slam"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    TunnelSlam node(nh,pnh); // Creating the object

    //ros::Rate rate(1.0); // Defing the looping rate

    /* Looking for any interupt else it will continue looping */
    while (ros::ok())
    {   
        //node.runOnce();
        ros::spin();
        //rate.sleep();
    }
    return 0;
}