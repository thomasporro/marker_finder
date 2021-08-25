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

    // Checks for the length of the contour's perimeter and fot its area,
    // then if computes if it fits in the formula of a circle. If it is true
    // then save the contour and the center and their moments
    for(auto contour: allContours){
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        // Skips the perimeters that are too small and if is present a too
        // large perimetes it skips the entire frame
        if(perimeter == 0 || perimeter < 10)
            continue;
        else if(perimeter > 300)
            return std::make_tuple(std::vector<std::vector<cv::Point>>(), std::vector<cv::Point>());
        
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));
        if(circularity > 0.85 && circularity < 1.05){
            contours.push_back(contour);
            momentsContours.push_back(cv::moments(contour));
        }
    }

    // Compute the centers using the moments
    for(auto moment: momentsContours){
        int centerX = static_cast<int>(moment.m10/(moment.m00 + 1e-5));
        int centerY = static_cast<int>(moment.m01/(moment.m00 + 1e-5));
        centers.push_back(cv::Point2f(centerX, centerY));
    }

    return std::make_tuple(contours, centers);
};


void FindMarkers::listenerCallback(const sensor_msgs::ImageConstPtr& image){
    std::string encoding = image->encoding;
    double scaleFactor = 1/256.0;

    cv_bridge::CvImagePtr imgPointer;
    cv_bridge::CvImage imgPointerColor;
    try{
        imgPointer = cv_bridge::toCvCopy(image);
    } catch (cv_bridge::Exception e){
        ROS_ERROR("cv_bridge error: %s", e.what());
    }
    cv::Mat tmpImage;

    // In case of different encodings we can easily set them up
    if(encoding.compare(FindMarkers::MONO16) == 1){
        imgPointer->image.convertTo(tmpImage, CV_8UC1, scaleFactor);
    } else {
        imgPointer->image.convertTo(tmpImage, CV_8UC1);
        ROS_WARN("No encoding compatible");
    }
        
    auto cont_cent = FindMarkers::findCenters(tmpImage);

    cv::Mat colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), std::get<1>(cont_cent));
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

    if(centers.size() <= 0 || contours.size()<=0)
        return tmpImage;
    
    cv::drawContours(tmpImage, contours, -1, greenColor, 2);
    for(unsigned i = 0; i < centers.size(); i++){
        cv::circle(tmpImage, centers[i], 3, redColor, cv::FILLED);
    }

    return tmpImage;
};

// TODO possible improvings: 
// - mask to detect the mean color value in order to discard the point that
//   are in the dark
// - check that all the circles found must have a similar size