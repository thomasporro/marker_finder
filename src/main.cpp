#include "ros/ros.h"

int main(int argc, char** argv){
    std::string node_name = "finder";
    ros::init(argc, argv, node_name);
    ros::spin();
}