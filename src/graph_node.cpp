// Node file to create object and initialising the ROS node
#include "graph.hpp"
#include <thread>

int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    ros::init(argc, argv, "graph"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    Graph node(nh,pnh); // Creating the object
    std::thread refineThread(&Graph::runRefine, &node);

    ros::Rate rate(10); // Defing the looping rate

    /* Looking for any interupt else it will continue looping */
    int runsWithoutUpdate=0;
    while (ros::ok())
    {   
        //ros::spin();
        ros::spinOnce();
        node.runOnce(runsWithoutUpdate);
        rate.sleep();
    }
    refineThread.join();
    std::cout << "SHUTTING DOWN - SAVING GRAPH" << std::endl;
    node.writeToFile();
    return 0;

}