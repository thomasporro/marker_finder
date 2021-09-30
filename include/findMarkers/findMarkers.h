#ifndef marker_finder_findMarkers_h
#define marker_finder_findMarkers_h

#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class FindMarkers{
public:

    FindMarkers()/*: tsp_(nodeHandle_){}*/{};
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
        std::string infoTopic{"/k01/ir/camera_info"};
    };

    Params params_;
    ros::NodeHandle nodeHandle_{"~"};

    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    std::string getImageEncoding();
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> findCenters(cv::Mat image, double imageWidth);
    void listenerCallback(const sensor_msgs::ImageConstPtr& Image, const sensor_msgs::CameraInfoConstPtr& info);
    cv::Mat drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point> centers);
    cv::Mat convertImage(const cv::Mat& image, const std::string encoding);
};

#endif