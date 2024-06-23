#include "flir_ros/flir_ros.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv,"flir_ros");
    ROS_INFO("Starting flir_ros camera driver");
    ros::NodeHandle node;
    ros::NodeHandle pnh("~");
    try{
        flirROS camera(node,pnh);
        ros::spin ();
    }
    catch(std::exception &e){
        std::cout<<e.what()<<std::endl;
    }
    
    return(0);
}