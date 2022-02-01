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

//Test
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


class FindMarkers{
public:

    FindMarkers(): tsp_(nodeHandle_){};
    ~FindMarkers(){};

    void start();

private:
    const double scale16To8 = 1.0/256.0;
    const double minDiameterRatio = 0.0045;
    const double maxDiameterRatio = 0.15;
    const double minCircularity = 0.80;
    const double maxCircularity = 1.05;

    struct Params{
        int queue{1000};
        std::string outTopic{"/k01/transform"};
        std::string inTopic{"/k01/ir/image_rect"};
    };

    Params params_;
    ros::NodeHandle nodeHandle_{"~"};

    image_transport::ImageTransport tsp_;
    image_transport::CameraSubscriber kinect_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Publisher transformPub_;
    message_filters::Subscriber<geometry_msgs::TransformStamped> transform_sub_[5];
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, 
                geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // // Publisher for the relative position of the camera respect to the wan
    // ros::Publisher transformPub_[5];
    // tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    // // Boolean to check if a frame is already a child
    // bool isChild_[5] = {1};

    //Debugging the opencv part
    ros::Publisher blur_;
    ros::Publisher threshold_;
    ros::Publisher dilate_;
    ros::Publisher contours_;

    std::string getImageEncoding();
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2d>> findCenters(cv::Mat image, double imageWidth);
    cv::Mat drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point2d> centers);
    cv::Mat convertImage(const cv::Mat& image, const std::string encoding);
    std::vector<cv::Point2d> orderPoints(std::vector<cv::Point2d> points);
    void publishTransform(cv::Mat rvec, cv::Mat tvec, std_msgs::Header header);
    double findInteger(std::string str);

    void listenerCallback(const sensor_msgs::ImageConstPtr& Image, const sensor_msgs::CameraInfoConstPtr& info);
    void transformCallback(const geometry_msgs::TransformStampedConstPtr& transf1, const geometry_msgs::TransformStampedConstPtr& transf2, 
                const geometry_msgs::TransformStampedConstPtr& transf3, const geometry_msgs::TransformStampedConstPtr& transf4, 
                const geometry_msgs::TransformStampedConstPtr& transf5);
    cv::Mat computePosition(geometry_msgs::Transform pos1, geometry_msgs::Transform pos2);
    geometry_msgs::TransformStamped createTransform(cv::Mat matrix, std::string head_frame, std::string child_frame);

    bool isBetween(cv::Point x, cv::Point z, cv::Point y);
};

#endif