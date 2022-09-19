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
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"


class FindMarkers{
public:

    FindMarkers(): tsp_(nodeHandle_){};
    ~FindMarkers(){};

    void start();

private:
    const double scale16To8 = 1.0/256.0;
    const double minDiameterRatio = 0.0045;
    const double maxDiameterRatio = 0.15;
    const double minCircularity = 0.70;

    struct Params{
        int queue{1000};
        std::string outTopic{"/k01/transform"};
        std::string inTopic{"/k01/ir/image_rect"};
        std::string debugTopic{"/k01/debug"};
    };

    Params params_;
    ros::NodeHandle nodeHandle_{"~"};

    image_transport::ImageTransport tsp_;
    image_transport::CameraSubscriber kinect_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Publisher transformPub_;
    ros::Publisher debug_;


    std::string getImageEncoding();

    // Return a tuple that given an image finds the possible markers in it.
    // It returns a tuple containing a vector of contours and their centers.
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2d>> findCenters(cv::Mat image, double imageWidth);

    // Given a set of contours and their centers it draws them in the given image.
    // It returns a new image that will contain all the drawings
    // The sequences of contours and centers must be ordered to obtain a coherent result.
    cv::Mat drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point2d> centers);

    // Given an image and its enconding it returns an image with the encoding
    // compatible with the system
    cv::Mat convertImage(const cv::Mat& image, const std::string encoding);

    // Given a set of points it orders them following the wand scheme and than
    // it returns them in an ordered vector
    std::vector<cv::Point2d> orderPoints(std::vector<cv::Point2d> points);

    // Given the rotation, translation matrix and the header it publish the transform
    // in the correct topic
    void publishTransform(cv::Mat rvec, cv::Mat tvec, std_msgs::Header header);

    // Finds and return the integer found in a string
    double findInteger(std::string str);

    // Method used every time a new image is received to the system. 
    // Given the image and the camera info it computes the position of the camera
    // in respect of the wand used
    void listenerCallback(const sensor_msgs::ImageConstPtr& Image, const sensor_msgs::CameraInfoConstPtr& info);

    // Return true if the z point is in the middle of the other two.
    // In order to obtain correct results points must be on the same line.
    bool isBetween(cv::Point x, cv::Point z, cv::Point y);
};

#endif