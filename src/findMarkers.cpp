#include "findMarkers/findMarkers.h"

#include <opencv2/core/fast_math.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Transform.h"

#include <cmath>

void FindMarkers::start(){

    nodeHandle_.getParam("queue", params_.queue);
    nodeHandle_.getParam("inTopic", params_.inTopic);
    nodeHandle_.getParam("outTopic", params_.outTopic);
    nodeHandle_.getParam("debugTopic", params_.debugTopic);

    kinect_ = tsp_.subscribeCamera(params_.inTopic, params_.queue, &FindMarkers::listenerCallback, this);
    transformPub_ = nodeHandle_.advertise<geometry_msgs::TransformStamped>(params_.outTopic, 1000);

    debug_ = nodeHandle_.advertise<sensor_msgs::Image>(params_.debugTopic, 1);
};


std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point2d>> FindMarkers::findCenters(cv::Mat image, double imageWidth){

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "pose";
    
    cv::Mat gray = image.clone();
    
    cv::threshold(gray, gray, 100, 255, cv::THRESH_TOZERO);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0, cv::BORDER_CONSTANT);
    cv::threshold(gray, gray, 100, 255, cv::THRESH_BINARY);


    // Matrix used to dilate the image
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));

    // Retrieve all the contours in the image
    std::vector<std::vector<cv::Point>> allContours; 
    std::vector<cv::Vec4i> hier;
    cv::findContours(gray, allContours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point2d> centers;
    std::vector<cv::Point2f> encloCircle;

    // Checks for the length of the contour's perimeter and for its area,
    // then if computes if it fits in the formula of a circle. If it is true
    // then save the contour and the center and their moments
    for(auto contour: allContours){
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        double diameter = perimeter / M_PI;

        //Find the markers that respect the rules
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));
        if(circularity > minCircularity){
            contours.push_back(contour);

            cv::Point2f single_point;
            float radius;
            cv::minEnclosingCircle( contour, single_point, radius);
            encloCircle.push_back(single_point);
        }
    }

    for(const auto& center: encloCircle){
        centers.push_back(cv::Point2d(center.x, center.y));
    }

    return std::make_tuple(contours, centers);
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

        // Mask to remove part of the image where the IR light is reflected by others kinects
        // it happens only with the kinect number one and four of the test footage
        cv::Mat mask = cv::Mat::zeros(image->height, image->width, tmpImage.type());
        int n_kinect = FindMarkers::findInteger(info->header.frame_id);
        cv::Scalar whiteColor(255, 255, 255);
        int radius = 15;
        if(n_kinect == 1){
            cv::circle(mask, cv::Point(482, 64), radius, whiteColor, cv::FILLED);
        } else if (n_kinect == 4){
            cv::circle(mask, cv::Point(127, 32), radius, whiteColor, cv::FILLED);
        }
        cv::subtract(tmpImage, mask, tmpImage);
        
        //Actually finding the centers
        cont_cent = FindMarkers::findCenters(tmpImage, image->width);
    } catch (cv::Exception e) {
        ROS_ERROR("Error while finding the centers of the markers: %s\n", e.what());
        return;
    }
    
    // Points used to dispaly the plane on screen
    std::vector<cv::Point3d> planePoints{cv::Point3d(0.0 , 0.0, 0.0), cv::Point3d(0.0, 0.3, 0.0), 
                         cv::Point3d(0.3, 0.3, 0.0), cv::Point3d(0.3, 0.0, 0.0)};
    std::vector<cv::Point2d> projectedPlanePoints;


    std::vector<cv::Point2d> projectedPoints;

    cv::Mat rvec, tvec;    

    int collinear = 1;
    if(std::get<1>(cont_cent).size()==5){ 
        std::vector<cv::Point3d> objectPoints{cv::Point3d(0.0, 0.250, 0.0), cv::Point3d(0.2, 0.250, 0.0), 
                    cv::Point3d(0.3, 0.250, 0.0), cv::Point3d(0.2, 0.125, 0.0), cv::Point3d(0.2, 0.0, 0.0)};

        // Orders the point to be consistent with the objects points
        std::vector<cv::Point2d> orderedImagePoints = FindMarkers::orderPoints(std::get<1>(cont_cent));

        std::vector<cv::Point2d> imagePoints;
        for(int i=0; i<orderedImagePoints.size();i++){
            imagePoints.push_back((cv::Point2d)orderedImagePoints[i]);
            if(orderedImagePoints[i].x == -1.0){
                collinear=0;
            }
        }

        if(collinear){
            // Brute force for creating matrix
            cv::Mat cameraMatrix(3, 3, CV_64FC1);
            for(int i = 0; i<info->K.size(); i++){
                cameraMatrix.at<double>(i/3, i%3) = info->K.at(i);
            }

            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, info->D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            
            // Converts the rotation matrix into rotation vector
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);

            // Computing the mean error of solvePnP by reprojecting the objects point into
            // the camera plane. Uncomment this line to print its value
            // cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, info->D, projectedPoints);
            // double meanError = cv::norm(imagePoints, projectedPoints, cv::NORM_L2);
            // printf("Error on reprojection: %f\n", meanError);

            // Compute the points that will represent the plane in debug_
            cv::projectPoints(planePoints, rvec, tvec, cameraMatrix, info->D, projectedPlanePoints);
            

            cv::Mat translateRotation;
            cv::transpose(rotationMatrix, translateRotation);

            //Finds the camera coordinates by inverting the projection matrix
            cv::Mat wrlCoordinates = (-translateRotation) * tvec;
        }
    }
    

    cv::Mat colorImage;    
    if(std::get<1>(cont_cent).size()==5 && collinear){
        // If it finds the markers send the transform to the static broadcaster
        FindMarkers::publishTransform(rvec, tvec, image->header);

        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), projectedPoints);

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
        
    } else {
        colorImage = FindMarkers::drawMarkers(tmpImage, std::get<0>(cont_cent), std::get<1>(cont_cent));
    }

    
    imgPointerColor.header = imgPointer->header;
    imgPointerColor.encoding = sensor_msgs::image_encodings::BGR8;
    imgPointerColor.image = colorImage;

    debug_.publish(imgPointerColor.toImageMsg());
    
};

cv::Mat FindMarkers::drawMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point2d> centers){
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar greenColor(0, 255, 0);
    
    cv::Mat tmpImage;
    cv::cvtColor(image, tmpImage, cv::COLOR_GRAY2BGR);

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

std::vector<cv::Point2d> FindMarkers::orderPoints(std::vector<cv::Point2d> points){

    // Check the cross product of 3 points to find
    // if they are on the same line
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
    transformStamped.child_frame_id = "world";
 
    if(!rvec.empty() && !tvec.empty()){
        tf2::Vector3 translation(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);
        tf2::Matrix3x3 tfMatrix((float)rotationMatrix.at<double>(0, 0), (float)rotationMatrix.at<double>(0, 1), (float)rotationMatrix.at<double>(0, 2),
                                (float)rotationMatrix.at<double>(1, 0), (float)rotationMatrix.at<double>(1, 1), (float)rotationMatrix.at<double>(1, 2),
                                (float)rotationMatrix.at<double>(2, 0), (float)rotationMatrix.at<double>(2, 1), (float)rotationMatrix.at<double>(2, 2));
        tf2::Quaternion rotation;
        tfMatrix.getRotation(rotation);

        tf2::Transform pose(rotation.normalize(), translation);

        // Trasform that change the axis direction according to ROS documentation
        tf2::Vector3 no_translation(0.0, 0.0, 0.0);
        double roll, pitch, yaw;
        yaw = 0;
        pitch = -90 * CV_PI / 180; 
        roll = 90 * CV_PI / 180; 
        tf2::Matrix3x3 axis_rotation;
        axis_rotation.setEulerYPR(yaw, pitch, roll);
        tf2::Quaternion axis_quaternion;
        axis_rotation.getRotation(axis_quaternion);
        tf2::Transform axis_transform(axis_quaternion.normalize(), no_translation);

        // Inverts and change rotation of the transform
        pose = pose.inverse() * axis_transform;

        transformStamped.transform.rotation.x = pose.getRotation().getX();
        transformStamped.transform.rotation.y = pose.getRotation().getY();
        transformStamped.transform.rotation.z = pose.getRotation().getZ();
        transformStamped.transform.rotation.w = pose.getRotation().getW();
        transformStamped.transform.translation.x = pose.getOrigin().getX();
        transformStamped.transform.translation.y = pose.getOrigin().getY();
        transformStamped.transform.translation.z = pose.getOrigin().getZ();
        transformPub_.publish(transformStamped);

        int camera_id = findInteger(transformStamped.header.frame_id);
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = std::to_string(camera_id);
        tf_broadcaster_.sendTransform(transformStamped);
    }
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

bool FindMarkers::isBetween(cv::Point x, cv::Point z, cv::Point y){
    double dotProduct = (z.x - x.x) * (y.x - x.x) + (z.y - x.y) * (y.y - x.y);
    if(dotProduct < 0.0)
        return false;

    double sqrtlength = (x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y);
    if(dotProduct > sqrtlength)
        return false;

    return true;
}