#include "findMarkers/findMarkers.h"

#include <opencv2/core/fast_math.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>

void FindMarkers::start(){

    nodeHandle_.getParam("queue", params_.queue);
    nodeHandle_.getParam("inTopic", params_.inTopic);
    nodeHandle_.getParam("outTopic", params_.outTopic);
    nodeHandle_.getParam("infoTopic", params_.infoTopic);

    kinect1_ = tsp_.subscribeCamera("/k01/ir/image_rect", params_.queue, &FindMarkers::listenerCallback, this);
    // kinect2_ = tsp_.subscribeCamera("/k02/ir/image_rect", params_.queue, &FindMarkers::listenerCallback, this);
    // kinect3_ = tsp_.subscribeCamera("/k03/ir/image_rect", params_.queue, &FindMarkers::listenerCallback, this);
    // kinect4_ = tsp_.subscribeCamera("/k04/ir/image_rect", params_.queue, &FindMarkers::listenerCallback, this);
    // kinect5_ = tsp_.subscribeCamera("/k05/ir/image_rect", params_.queue, &FindMarkers::listenerCallback, this);

    // transformPub_[0] = nodeHandle_.advertise<geometry_msgs::TransformStamped>("transform1", 1000);
    // transformPub_[1] = nodeHandle_.advertise<geometry_msgs::TransformStamped>("transform2", 1000);
    // transformPub_[2] = nodeHandle_.advertise<geometry_msgs::TransformStamped>("transform3", 1000);
    // transformPub_[3] = nodeHandle_.advertise<geometry_msgs::TransformStamped>("transform4", 1000);
    // transformPub_[4] = nodeHandle_.advertise<geometry_msgs::TransformStamped>("transform5", 1000);

    // transform_sub_[0].subscribe(nodeHandle_, "transform1", 1000);
    // transform_sub_[1].subscribe(nodeHandle_, "transform2", 1000);
    // transform_sub_[2].subscribe(nodeHandle_, "transform3", 1000);
    // transform_sub_[3].subscribe(nodeHandle_, "transform4", 1000);
    // transform_sub_[4].subscribe(nodeHandle_, "transform5", 1000);

    // sync_.reset(new Sync(MySyncPolicy(10), transform_sub_[0], transform_sub_[1], transform_sub_[2],
    //             transform_sub_[3], transform_sub_[4]));
    // sync_->registerCallback(boost::bind(&FindMarkers::transformCallback, this, _1, _2, _3, _4, _5));


    pub_ = nodeHandle_.advertise<sensor_msgs::Image>(params_.outTopic, 1);
    blur_ = nodeHandle_.advertise<sensor_msgs::Image>("GaussianBlur", 1);
    threshold_ = nodeHandle_.advertise<sensor_msgs::Image>("threshold", 1);
    dilate_ = nodeHandle_.advertise<sensor_msgs::Image>("dilate", 1);
    contours_ = nodeHandle_.advertise<sensor_msgs::Image>("contours", 1);

    // TODO remove hardocoding
};

std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2d>> FindMarkers::findCenters(cv::Mat image, double imageWidth){
    cv_bridge::CvImage imgBlur;
    cv_bridge::CvImage imgThreshold;
    cv_bridge::CvImage imgDilate;
    cv_bridge::CvImage imgContours;

    imgBlur.encoding = sensor_msgs::image_encodings::MONO8;
    imgThreshold.encoding = sensor_msgs::image_encodings::MONO8;
    imgDilate.encoding = sensor_msgs::image_encodings::MONO8;
    imgContours.encoding = sensor_msgs::image_encodings::BGR8;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "test";

    imgBlur.header = header;
    imgThreshold.header = header;
    imgDilate.header = header;
    imgContours.header = header;
    
    cv::Mat gray = image.clone();

    // cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    imgBlur.image = gray.clone();
    cv::threshold(gray, gray, 180, 255, cv::THRESH_BINARY);
    imgThreshold.image = gray.clone();

    // Matrix used to dilate the image
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(gray, gray, element); 
    imgDilate.image = gray.clone();

    std::vector<std::vector<cv::Point>> allContours; 
    std::vector<cv::Vec4i> hier;
    cv::findContours(gray, allContours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat colorata;
    cv::cvtColor(gray, colorata, cv::COLOR_GRAY2BGR);

    if(allContours.size() != 0)
        cv::drawContours(colorata, allContours, -1, cv::Scalar(0, 0, 255), 1);
    imgContours.image = colorata.clone();

    //blur_.publish(imgBlur.toImageMsg());
    threshold_.publish(imgThreshold.toImageMsg());
    dilate_.publish(imgDilate.toImageMsg());
    contours_.publish(imgContours.toImageMsg());


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point2d> centers;
    std::vector<cv::Moments> momentsContours;
    // printf("image width: %f\n", imageWidth);

    // Checks for the length of the contour's perimeter and for its area,
    // then if computes if it fits in the formula of a circle. If it is true
    // then save the contour and the center and their moments
    for(auto contour: allContours){
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        double diameter = perimeter / M_PI;
        // Skips the perimeters that are too small and if is present a too
        // large perimetes it skips the entire frame
        // printf("diameter: %f\n", diameter);
        // if(diameter < FindMarkers::minDiameterRatio * imageWidth)
        //     continue;
        // else if(diameter > FindMarkers::maxDiameterRatio * imageWidth)
        //     continue;

        double circularity = 4 * M_PI * (area / (perimeter * perimeter));
        //printf("circularity: %f\n", circularity);
        if(/*circularity > minCircularity && circularity < maxCircularity*/ circularity > 0.70){
            contours.push_back(contour);
            momentsContours.push_back(cv::moments(contour));
        }
    }

    //Checking if the dimensions of the circles are similar


    // Compute the centers using the moments
    for(const auto& moment: momentsContours){
        int centerX = static_cast<int>(moment.m10/(moment.m00 + 1e-5));
        int centerY = static_cast<int>(moment.m01/(moment.m00 + 1e-5));
        centers.push_back(cv::Point2d(centerX, centerY));
    }

    // Trying the usage of a rectangle instead of the moments
    // for(auto contour : contours){
    //     cv::Rect rect = cv::boundingRect(contour);
    //     int centerX = rect.x + (rect.height / 2);
    //     int centerY = rect.y + (rect.width / 2);
    //     centers.push_back(cv::Point2d(centerX, centerY));
    // }

    return std::make_tuple(contours, centers);
    

    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Point2d> centers;
    // cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    // cv::threshold(gray, gray, 150, 255, cv::THRESH_BINARY);
    // std::vector<cv::Vec3f> circles;
    // cv:HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2, 5, 50, 10, 1, 50);
    // for(int i = 0; i < circles.size(); i++){
    //     centers.push_back(cv::Point2d(cvRound(circles[i][0]), cvRound(circles[i][1])));
    // }
    // return std::make_tuple(contours, centers);
};

void FindMarkers::listenerCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info){
   
    cv_bridge::CvImagePtr imgPointer;
    cv_bridge::CvImage imgPointerColor;

    try{
        imgPointer = cv_bridge::toCvCopy(image);
    } catch (cv_bridge::Exception e){
        ROS_ERROR("cv_bridge error: %s\n", e.what());
    }

    cv::Mat tmpImage;
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2d>> cont_cent;
    try{
        //Convert image in the correct format for opencv
        tmpImage = FindMarkers::convertImage(imgPointer->image, image->encoding);
        cont_cent = FindMarkers::findCenters(tmpImage, image->width);
    } catch (cv::Exception e) {
        ROS_ERROR("Error while finding the centers of the markers: %s\n", e.what());
    }
    

    std::vector<cv::Point3d> planePoints{cv::Point3d(0.0 , 0.0, 0.0), cv::Point3d(0.0, 0.3, 0.0), 
                         cv::Point3d(0.3, 0.3, 0.0), cv::Point3d(0.3, 0.0, 0.0)};
    std::vector<cv::Point2d> projectedPlanePoints;


    std::vector<cv::Point2d> projectedPoints;
    cv::Mat rvec, tvec;
    int collinear = 1;
    if(std::get<1>(cont_cent).size()==5){
        std::vector<cv::Point3d> objectPoints{cv::Point3d(0.0, 0.0, 0.0), cv::Point3d(0.2, 0.0, 0.0), 
                    cv::Point3d(0.3, 0.0, 0.0), cv::Point3d(0.2, 0.125, 0.0), cv::Point3d(0.2, 0.25, 0.0)};

        // Orders the point to be consistent with the objects points
        std::vector<cv::Point2d> imagePoints2 = FindMarkers::orderPoints(std::get<1>(cont_cent));
        std::vector<cv::Point2d> imagePoints;
        for(int i=0; i<imagePoints2.size();i++){
            imagePoints.push_back((cv::Point2d)imagePoints2[i]);
            if(imagePoints2[i].x == -1.0){
                collinear=0;
            }
        }

        if(collinear){
            // Brute force for creating matrix
            // TODO search if it is possible to improve
            cv::Mat cameraMatrix(3, 3, CV_64FC1);
            for(int i = 0; i<info->K.size(); i++){
                cameraMatrix.at<double>(i/3, i%3) = info->K.at(i);
            }

            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, info->D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            // cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, info->D, rvec, tvec, false, 1000, 1.5, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP);
            
            // Converts the rotation matrix into rotation vector
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);

            // Computing the mean error of solvePnP by reprojecting the objects point into
            // the camera plane
            cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, info->D, projectedPoints);
            // double meanError = cv::norm(imagePoints, projectedPoints, cv::NORM_L2);
            // printf("Error on reprojection: %f\n", meanError);


            // Debugging lines
            cv::projectPoints(planePoints, rvec, tvec, cameraMatrix, info->D, projectedPlanePoints);

            cv::Mat translateRotation;
            cv::transpose(rotationMatrix, translateRotation);

            //Finds the camera coordinates by inverting the projection matrix
            cv::Mat wrlCoordinates = (-translateRotation) * tvec;

            // printf("Coordinates: %f %f %f\n", wrlCoordinates.at<double>(0, 0), 
            //         wrlCoordinates.at<double>(0, 1), wrlCoordinates.at<double>(0, 2));

        }
    }
    FindMarkers::publishTransform(rvec, tvec, info->header);

    cv::Mat colorImage;    
    if(std::get<1>(cont_cent).size()==5 && collinear){
        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), projectedPoints);
        // colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), std::get<1>(cont_cent));


        // Draw the bounding box that show us the plane
        cv::Scalar blueColor(255, 0, 0);
        int thickness = 2;
        std::vector<cv::Point> finalPoints;
        for(int i=0; i<projectedPlanePoints.size(); i++){
            finalPoints.push_back(cv::Point(projectedPlanePoints[i].x, projectedPlanePoints[i].y));
        }
        projectedPlanePoints.clear();
        planePoints.clear();
        cv::polylines(colorImage, finalPoints, true, blueColor, thickness, cv::FILLED, 0);

        
        
    }
    else{
        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), std::get<1>(cont_cent));
        //cv::cvtColor(colorImage, tmpImage, cv::COLOR_GRAY2BGR);
    }

    
    imgPointerColor.header = imgPointer->header;
    imgPointerColor.encoding = sensor_msgs::image_encodings::BGR8;
    imgPointerColor.image = colorImage;

    pub_.publish(imgPointerColor.toImageMsg());
    
};

cv::Mat FindMarkers::drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point2d> centers){
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar greenColor(0, 255, 0);
    
    cv::Mat tmpImage;
    cv::cvtColor(image, tmpImage, cv::COLOR_GRAY2BGR);
    
    if(contours.size() != 0)
        cv::drawContours(tmpImage, contours, -1, greenColor, 2);

    for(unsigned i = 0; i < centers.size(); i++){
        cv::circle(tmpImage, centers[i], 1, redColor, cv::FILLED);
    }

    return tmpImage;
};

cv::Mat FindMarkers::convertImage(const cv::Mat& image, const std::string encoding){
    cv::Mat tmpImage;

    // At the moment theese are the supported ones, need to check if the sensors
    // can provide other formats
    if(encoding.compare(sensor_msgs::image_encodings::MONO8) == 0){
        tmpImage = image.clone();
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
//   are in the dark (is it correct? i don't think so)
// - check that all the circles found must have a similar size

std::vector<cv::Point2d> FindMarkers::orderPoints(std::vector<cv::Point2d> points){

    // Check for the area of the triangle for 3 points to find
    // the ones that are on the same line
    std::vector<std::vector<cv::Point2d>> lines;
    for(int i = 0; i < points.size() - 2; i++){
        for(int j = i + 1; j < points.size() - 1; j++){
            for(int k = j + 1; k < points.size(); k++){
                cv::Point2d p1 = points[i];
                cv::Point2d p2 = points[j];
                cv::Point2d p3 = points[k];
                int cross = (p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x);
                if(cross < 50 && cross > -50){
                    lines.push_back(std::vector<cv::Point2d> {p1, p2, p3});
                }
            }
        }
    }

    std::vector<cv::Point2d> outputPoints(5);
    cv::Point2d outPoint = cv::Point2d(-1, -1);
    std::fill(outputPoints.begin(), outputPoints.end(), outPoint);

    for(int i = 0; i < lines.size(); i++){
        cv::Point2d p1 = lines[i][0];
        cv::Point2d p2 = lines[i][1];
        cv::Point2d p3 = lines[i][2];

        cv::Point2d center;
        cv::Point2d right;
        cv::Point2d left;

        if(FindMarkers::isBetween(p2, p1, p3)){
            center = p1;
            right = p3;
            left = p2;
        } else if(FindMarkers::isBetween(p1, p2, p3)){
            center = p2;
            right = p3;
            left = p1;
        } else if(FindMarkers::isBetween(p1, p3, p2)){
            center = p3;
            right = p2;
            left = p1;
        }

        double fraction =  cv::norm(left-center) / cv::norm(center-right);

        if(fraction > 1.7 && fraction < 2.3){
            outputPoints[0] = left;
            outputPoints[2] = right;
            if(outputPoints[1].x != -1.0){
                // If the point is not equal to center, inverts out[4] and out[1]
                if(!(outputPoints[1].x == center.x && outputPoints[1].y == center.y)){
                    outputPoints[4] = outputPoints[1];
                    outputPoints[1] = center;
                }
            } else {
                outputPoints[1] = center;
            }
        } else if(fraction > 0.4 && fraction < 0.6){
            outputPoints[0] = right;
            outputPoints[2] = left;
            if(outputPoints[1].x != -1.0){
                // If the point is not equal to center, inverts out[4] and out[1]
                if(!(outputPoints[1].x == center.x && outputPoints[1].y == center.y)){
                    outputPoints[4] = outputPoints[1];
                    outputPoints[1] = center;
                }
            } else {
                outputPoints[1] = center;
            }
        } else if(fraction > 0.7 && fraction < 1.4) {
            outputPoints[3] = center;
            if(outputPoints[1].x != -1.0){
                if(outputPoints[1].x == left.x && outputPoints[1].y == left.y){
                    outputPoints[4] = right;
                } else {
                    outputPoints[4] = left;
                }
            } else {
                outputPoints[1] = left;
                outputPoints[4] = right;
            }
        }

    }
    return outputPoints;
}

void FindMarkers::publishTransform(cv::Mat rvec, cv::Mat tvec, std_msgs::Header header){
    // Setting the message to be sent
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header = header;
    transformStamped.child_frame_id = header.frame_id;

    if(rvec.empty() || tvec.empty()){
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 0.0;
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
    } else {
        tf2::Vector3 translation(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);
        tf2::Matrix3x3 tfMatrix(rotationMatrix.at<double>(0, 0), rotationMatrix.at<double>(0, 1), rotationMatrix.at<double>(0, 2),
                                rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(1, 1), rotationMatrix.at<double>(1, 2),
                                rotationMatrix.at<double>(2, 0), rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
        tf2::Quaternion rotation;
        tfMatrix.getRotation(rotation);

        // Setting the message to be sent
        transformStamped.transform.rotation.x = rotation.x();
        transformStamped.transform.rotation.y = rotation.y();
        transformStamped.transform.rotation.z = rotation.z();
        transformStamped.transform.rotation.w = rotation.w();
        transformStamped.transform.translation.x = translation[0];
        transformStamped.transform.translation.y = translation[1];
        transformStamped.transform.translation.z = translation[2];
    }

    // Find on which topic to publish
    int kinect_number = FindMarkers::findInteger(header.frame_id);

    transformPub_[kinect_number-1].publish(transformStamped);
}

double FindMarkers::findInteger(std::string str){
    // Finds the first and last position where a digit appears
    size_t i = 0;
    for(; i < str.length(); i++){
        if(isdigit(str[i]))
            break;
    }
    size_t start, end;
    start = i;
    for(; i < str.length(); i++){
        if(!isdigit(str[i]))
            break;
    }
    end=i;

    int integer = atoi( str.substr(start, end-1).c_str());
    return integer;
}

void FindMarkers::transformCallback(const geometry_msgs::TransformStampedConstPtr& transf1, const geometry_msgs::TransformStampedConstPtr& transf2, 
                const geometry_msgs::TransformStampedConstPtr& transf3, const geometry_msgs::TransformStampedConstPtr& transf4, 
                const geometry_msgs::TransformStampedConstPtr& transf5){
    
    if(transf1->transform.translation.z!=0.0 && transf2->transform.translation.z!=0.0){
        cv::Mat positions;
        geometry_msgs::TransformStamped transform;

        int n_transf1 = FindMarkers::findInteger(transf1->header.frame_id);
        int n_transf2 = FindMarkers::findInteger(transf2->header.frame_id);
        
        // Check if the boolean the boolean values works
        // At the moment is in test
        if(isChild_[n_transf2 - 1] == false){
            positions = computePosition(transf1->transform, transf2->transform);
            transform = FindMarkers::createTransform(positions, transf1->header.frame_id, transf2->header.frame_id);
        } else if(isChild_[n_transf1 - 1] == false){
            positions = computePosition(transf2->transform, transf1->transform);
            transform = FindMarkers::createTransform(positions, transf2->header.frame_id, transf3->header.frame_id);
        }

        static_broadcaster.sendTransform(transform);
    }   

}

cv::Mat FindMarkers::computePosition(geometry_msgs::Transform pos1, geometry_msgs::Transform pos2){
    cv::Mat output(4, 4, CV_64FC1);
    output.at<double>(3, 3) = 1.0;

    tf2::Matrix3x3 preliminaryRotation1(tf2::Quaternion(pos1.rotation.x, 
                pos1.rotation.y, pos1.rotation.z, pos1.rotation.w));
    tf2::Matrix3x3 preliminaryRotation2(tf2::Quaternion(pos2.rotation.x, 
                pos2.rotation.y, pos2.rotation.z, pos2.rotation.w));
    
    // Initialization of the rotation matrices
    cv::Mat rotation1(3, 3, CV_64FC1);
    cv::Mat rotation2(3, 3, CV_64FC1);
    for(int i = 0; i < 3; i++){
        rotation1.at<double>(i, 0) = preliminaryRotation1[0].getX();
        rotation1.at<double>(i, 1) = preliminaryRotation1[0].getY();
        rotation1.at<double>(i, 2) = preliminaryRotation1[0].getZ();

        rotation2.at<double>(i, 0) = preliminaryRotation2[0].getX();
        rotation2.at<double>(i, 1) = preliminaryRotation2[0].getY();
        rotation2.at<double>(i, 2) = preliminaryRotation2[0].getZ();
    }   

    // Initialization of the translation vectors
    cv::Mat translation1(3, 1, CV_64FC1);
    cv::Mat translation2(3, 1, CV_64FC1);

    translation1.at<double>(0, 0) = pos1.translation.x;
    translation1.at<double>(1, 0) = pos1.translation.y;
    translation1.at<double>(2, 0) = pos1.translation.z;

    translation2.at<double>(0, 0) = pos2.translation.x;
    translation2.at<double>(1, 0) = pos2.translation.y;
    translation2.at<double>(2, 0) = pos2.translation.z;
    
    cv::Mat finalRotation(3, 3, CV_64FC1);
    cv::Mat finalTranslation(3, 3, CV_64FC1);

    // Actual code to compute the tranformation
    finalRotation = rotation2.t() * rotation1;
    finalTranslation = rotation2.t() * (translation1 - translation2);

    // Filling the output matrix
    // In Rect documentation the inialization is based on x, y, width, height,
    // so the concept of row, col is inverted
    cv::Rect rect_rot(0, 0, finalRotation.cols, finalRotation.rows);
    cv::Rect rect_transl(3, 0, finalTranslation.cols, finalTranslation.rows);
    cv::Mat output_R = output(rect_rot);
    cv::Mat output_T = output(rect_transl);
    finalRotation.copyTo(output_R);
    finalTranslation.copyTo(output_T);

    return output;
}

geometry_msgs::TransformStamped FindMarkers::createTransform(cv::Mat matrix, std::string head_frame_id, std::string child_frame_id){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = head_frame_id;
    transform.child_frame_id = child_frame_id;

    tf2::Vector3 translation(matrix.at<double>(3, 0), matrix.at<double>(3, 1), matrix.at<double>(3, 2));
    tf2::Matrix3x3 tfMatrix(matrix.at<double>(0, 0), matrix.at<double>(0, 1), matrix.at<double>(0, 2),
                            matrix.at<double>(1, 0), matrix.at<double>(1, 1), matrix.at<double>(1, 2),
                            matrix.at<double>(2, 0), matrix.at<double>(2, 1), matrix.at<double>(2, 2));

    tf2::Quaternion rotation;
    tfMatrix.getRotation(rotation);

    // Setting the message to be sent
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];

    return transform;
}

bool FindMarkers::isBetween(cv::Point x, cv::Point z, cv::Point y){
    double dotProduct = (z.x - x.x) * (y.x - x.x) + (z.y - x.y) * (y.y - x.y);
    if(dotProduct < 0.0)
        return false;

    double sqrtlength = (x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y);
    if(dotProduct > sqrtlength)
        return false;


    return true;
}

