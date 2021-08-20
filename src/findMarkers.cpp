#include "findMarkers/findMarkers.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "image_transport/image_transport.h"

void FindMarkers::start(){

    nodeHandle.getParam("rate", params.rate);
    nodeHandle.getParam("inTopic", params.inTopic);
    nodeHandle.getParam("outTopic", params.outTopic);
    
    sub = nodeHandle.subscribe(params.inTopic, params.rate, &FindMarkers::listenerCallback, this);

};


std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> FindMarkers::findCenters(cv::Mat image){

    cv::Mat gray = image.clone();
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    cv::threshold(gray, gray, 250, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Matrix used to dilate the image
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
    cv::dilate(gray, gray, element);

    std::vector<std::vector<cv::Point>> lol;
    std::vector<cv::Point> ehi;
    return std::make_tuple(lol, ehi);
};


void FindMarkers::listenerCallback(const sensor_msgs::ImageConstPtr& Image){};