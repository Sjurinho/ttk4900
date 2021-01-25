// Node file to create object and initialising the ROS node
#include "graph.hpp"

int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    ros::init(argc, argv, "feature_association"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    Graph node(nh,pnh); // Creating the object

    ros::Rate rate(100); // Defing the looping rate

    /* Looking for any interupt else it will continue looping */
    while (ros::ok())
    {   
        ros::spinOnce();
        node.runOnce();
        rate.sleep();
    }
    return 0;
}