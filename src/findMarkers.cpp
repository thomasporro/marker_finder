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
    
    sub = tsp.subscribe(params.inTopic, params.rate, &FindMarkers::listenerCallback, this);
    pub = tsp.advertise(params.outTopic, 1);
};


std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> FindMarkers::findCenters(cv::Mat image){

    cv::Mat gray = image.clone();
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    cv::threshold(gray, gray, 250, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Matrix used to dilate the image
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
    cv::dilate(gray, gray, element);

    std::vector<std::vector<cv::Point>> allContours;
    std::vector<cv::Vec4i> hier;
    cv::findContours(gray, allContours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> centers;
    std::vector<cv::Moments> momentsContours;

    for(auto contour: allContours){
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        if(perimeter == 0 || perimeter < 10)
            continue;
        else if(perimeter > 300)
            return std::make_tuple(contours, centers);
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));
        if(circularity > 0.8 && circularity < 1.2){
            contours.push_back(contour);
            momentsContours.push_back(cv::moments(contour));
        }
    }

    for(auto moment: momentsContours){
        int centerX = static_cast<int>(moment.m10/(moment.m00 + 1e-5));
        int centerY = static_cast<int>(moment.m01/(moment.m00 + 1e-5));
        centers.push_back(cv::Point2f(centerX, centerY));
    }

    return std::make_tuple(contours, centers);
};


void FindMarkers::listenerCallback(const sensor_msgs::ImageConstPtr& image){
    //TODO need to take the encoding from the info of the mesage
    cv_bridge::CvImagePtr imgPointer;
    cv_bridge::CvImage imgPointerColor;
    try{
        imgPointer = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception e){
        ROS_ERROR("cv_bridge error: %s", e.what());
    }
    auto cont_cent = FindMarkers::findCenters(imgPointer->image);

    cv::Mat colorImage = FindMarkers::drawMarkers(imgPointer->image, std::get<0>(cont_cent), std::get<1>(cont_cent));
    imgPointerColor.header = imgPointer->header;
    imgPointerColor.encoding = sensor_msgs::image_encodings::BGR8;
    imgPointerColor.image = colorImage;

    pub.publish(imgPointerColor.toImageMsg());
};

cv::Mat FindMarkers::drawMarkers(cv::Mat image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point> centers){
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar greenColor(0, 255, 0);
    
    cv::Mat tmpImage;
    cv::cvtColor(image, tmpImage, cv::COLOR_GRAY2BGR);
    
    cv::drawContours(tmpImage, contours, -1, greenColor, 2);
    for(unsigned i = 0; i < centers.size(); i++){
        cv::circle(tmpImage, centers[i], 3, redColor, -1);
    }

    return tmpImage;
};