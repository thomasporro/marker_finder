#ifndef marker_finder_findMarkers_h
#define marker_finder_findMarkers_h

#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"


class FindMarkers{
public:
    //TODO move to private. Check for variable in sensor_msgs

    FindMarkers(): tsp_(nodeHandle_){};
    ~FindMarkers(){};

    void start();

private:
    const double scale16To8 = 1.0/256.0;
    const double minDiameterRatio = 0.005;
    const double maxDiameterRatio = 0.15;
    const double minCircularity = 0.85;
    const double maxCircularity = 1.05;

    struct Params{
        int queue{1000};
        std::string outTopic{"image"};
        std::string inTopic{"/k01/ir/image_rect"};
    };

    Params params_;
    ros::NodeHandle nodeHandle_{"~"};
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    image_transport::ImageTransport tsp_;

    std::string getImageEncoding();
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> findCenters(cv::Mat image, double imageWidth);
    void listenerCallback(const sensor_msgs::ImageConstPtr& Image);
    cv::Mat drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point> centers);
    cv::Mat convertImage(const cv::Mat& image, const std::string encoding);
};

#endif