#include "findMarkers/findMarkers.h"

void FindMarkers::start(){

    nodeHandle_.getParam("queue", params_.queue);
    nodeHandle_.getParam("inTopic", params_.inTopic);
    nodeHandle_.getParam("outTopic", params_.outTopic);
    nodeHandle_.getParam("infoTopic", params_.infoTopic);

    pub_ = nodeHandle_.advertise<sensor_msgs::Image>(params_.outTopic, 1);
    image_sub_.subscribe(nodeHandle_, params_.inTopic, params_.queue);
    info_sub_.subscribe(nodeHandle_, params_.infoTopic, params_.queue);
    sync_.reset(new Sync(MySyncPolicy(10), image_sub_,  info_sub_));
    sync_->registerCallback(boost::bind(&FindMarkers::listenerCallback, this, _1, _2));
};

std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2f>> FindMarkers::findCenters(cv::Mat image, double imageWidth){
    cv::Mat gray = image.clone();

    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
    cv::threshold(gray, gray, 130, 255, cv::THRESH_BINARY);

    // Matrix used to dilate the image
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
    cv::dilate(gray, gray, element); 

    std::vector<std::vector<cv::Point>> allContours;
    std::vector<cv::Vec4i> hier;
    cv::findContours(gray, allContours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point2f> centers;
    std::vector<cv::Moments> momentsContours;

    // Checks for the length of the contour's perimeter and fot its area,
    // then if computes if it fits in the formula of a circle. If it is true
    // then save the contour and the center and their moments
    for(auto contour: allContours){
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        double diameter = perimeter / M_PI;
        // Skips the perimeters that are too small and if is present a too
        // large perimetes it skips the entire frame
    
        if(diameter < FindMarkers::minDiameterRatio * imageWidth)
            continue;
        else if(diameter > FindMarkers::maxDiameterRatio * imageWidth)
            continue;

        double circularity = 4 * M_PI * (area / (perimeter * perimeter));
        if(circularity > FindMarkers::minCircularity && circularity < maxCircularity){
            contours.push_back(contour);
            momentsContours.push_back(cv::moments(contour));
        }
    }

    // Compute the centers using the moments
    for(const auto& moment: momentsContours){
        int centerX = static_cast<int>(moment.m10/(moment.m00 + 1e-5));
        int centerY = static_cast<int>(moment.m01/(moment.m00 + 1e-5));
        centers.push_back(cv::Point2f(centerX, centerY));
    }

    return std::make_tuple(contours, centers);
};

void FindMarkers::listenerCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info){

    cv_bridge::CvImagePtr imgPointer;
    cv_bridge::CvImage imgPointerColor;
    try{
        imgPointer = cv_bridge::toCvCopy(image);
    } catch (cv_bridge::Exception e){
        ROS_ERROR("cv_bridge error: %s", e.what());
    }

    //Convert image in the correct format for opencv
    cv::Mat tmpImage = FindMarkers::convertImage(imgPointer->image, image->encoding);
        
    auto cont_cent = FindMarkers::findCenters(tmpImage, image->width);

    std::vector<cv::Point2f> projectedPoints;
    cv::Mat rvec, tvec;
    if(std::get<1>(cont_cent).size()==5){
        //Variables for solvePnp
        std::vector<cv::Point3f> objectPoints{cv::Point3f(0.25, 0.2, 0.0), cv::Point3f(0.125, 0.2,  0.0), 
                    cv::Point3f(0.0, 0.0, 0.0), cv::Point3f(0.0, 0.2, 0.0), cv::Point3f(0.0, 0.3, 0.0)};

        // Conversion needed beacause solvePnP accept std::vector<cv::Point2d> and
        // passing directly the vector don't work
        std::vector<cv::Point2f> imagePoints;
        for(int i = 0; i<std::get<1>(cont_cent).size(); i++){
            imagePoints.push_back(std::get<1>(cont_cent).at(i));
        }

        // Brute force for creating matrix (search for a new method)
        cv::Mat cameraMatrix(3, 3, CV_64FC1);
        for(int i = 0; i<info->K.size(); i++){
            cameraMatrix.at<double>(i/3, i%3) = info->K.at(i);
        }
        
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, info->D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        // Converts the rotation matrix into rotation vector
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);

        // Computing the mean error of solvePnP by reprojecting the objects point into
        // the camera plane
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, info->D, projectedPoints);
        double meanError = cv::norm(imagePoints, projectedPoints, cv::NORM_L2);
        printf("Error on reprojection: %f\n", meanError);

        cv::Mat translateRotation;
        cv::transpose(rotationMatrix, translateRotation);

        //Finds the camera coordinates by iverting the projection matrix
        cv::Mat wrlCoordinates = (-translateRotation) * tvec;
    }

    cv::Mat colorImage;
    if(std::get<1>(cont_cent).size()==5){
        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), projectedPoints);
    }
    else
        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), std::get<1>(cont_cent));
    imgPointerColor.header = imgPointer->header;
    imgPointerColor.encoding = sensor_msgs::image_encodings::BGR8;
    imgPointerColor.image = colorImage;

    pub_.publish(imgPointerColor.toImageMsg());
};

cv::Mat FindMarkers::drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point2f> centers){
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar greenColor(0, 255, 0);
    
    cv::Mat tmpImage;
    cv::cvtColor(image, tmpImage, cv::COLOR_GRAY2BGR);

    //if(centers.size() <= 0 || contours.size()<=0)
    //    return tmpImage;
    
    if(contours.size() != 0)
        cv::drawContours(tmpImage, contours, -1, greenColor, 2);

    for(unsigned i = 0; i < centers.size(); i++){
        cv::circle(tmpImage, centers[i], 3, redColor, cv::FILLED);
    }

    return tmpImage;
};

cv::Mat FindMarkers::convertImage(const cv::Mat& image, const std::string encoding){
    cv::Mat tmpImage;

    // At the moment theese are the supported ones, need to check if the sensors
    // can provide other formats
    if(encoding.compare(sensor_msgs::image_encodings::MONO8) == 0){

    } else if(encoding.compare(sensor_msgs::image_encodings::MONO16) == 0
            || encoding.compare(sensor_msgs::image_encodings::RGB16) == 0
            || encoding.compare(sensor_msgs::image_encodings::RGBA16) == 0
            || encoding.compare(sensor_msgs::image_encodings::BGR16) == 0
            || encoding.compare(sensor_msgs::image_encodings::BGRA16) == 0
            || encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0){
        image.convertTo(tmpImage, CV_8UC1, FindMarkers::scale16To8);
    } else if (encoding.compare(sensor_msgs::image_encodings::RGB8) == 0
            || encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0
            || encoding.compare(sensor_msgs::image_encodings::BGR8) == 0
            || encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0){
        image.convertTo(tmpImage, CV_8UC1);
    }else {
        image.convertTo(tmpImage, CV_8UC1);
        ROS_WARN("No encoding compatible: converted without a scale factor");
    }
    
    return tmpImage;
}

// TODO possible improvings: 
// - mask to detect the mean color value in order to discard the point that
//   are in the dark
// - check that all the circles found must have a similar size