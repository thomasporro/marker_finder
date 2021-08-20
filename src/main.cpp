#include "ros/ros.h"
#include "findMarkers/findMarkers.h"

int main(int argc, char** argv){
    std::string node_name = "finder";
    ros::init(argc, argv, node_name);
    FindMarkers fm;
    fm.start();
    ros::spin();
}