#include "ros/ros.h"
#include "findMarkers/findMarkers.h"
#include "findMarkers/calibration.h"

int main(int argc, char** argv){
    std::string node_name = "finder";
    ros::init(argc, argv, node_name);
    FindMarkers fm;
    fm.start();
    

    Calibration calibration;
    calibration.configure();

    ros::spin();
}