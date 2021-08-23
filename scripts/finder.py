#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math


def findMarkers(image):
    """Method that find the ir markers given an image.

    Keyword arguments:
    image -- the image where the markers are searched (must be np.uint8, 
                in grayscale and single channel)
    """

    # TODO improving by forcing the contours to have a similar
    # dimension

    # Use gray image in order to use the filters
    gray = image.copy()

    # Blurs the image and apply the otsu treshold
    gray = cv.GaussianBlur(gray, (5, 5), 0)
    ret, gray = cv.threshold(gray, 250, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)

    # Dilate the image so the small components get connected and will no be
    # detected anymore
    gray = cv.dilate(gray, cv.getStructuringElement(
        cv.MORPH_ELLIPSE, (15, 15)))

    # Find the contours in the image
    allContours, hier = cv.findContours(
        gray, cv.CHAIN_APPROX_SIMPLE, cv.CHAIN_APPROX_SIMPLE)

    contours = []
    centers = []
    momentsContours = []
    for contour in allContours:
        perimeter = cv.arcLength(contour, True)
        area = cv.contourArea(contour)
        # If there is a permiter too much big the entire
        # frame is skippet from detecting the circles
        if perimeter == 0 or perimeter < 10:
            continue
        elif perimeter > 300:
            return contours, centers
        # Check the circularity so we can skip the ellipses
        circularity = 4*math.pi*(area/(perimeter*perimeter))
        if 0.8 < circularity < 1.2:
            contours.append(contour)
            momentsContours.append(cv.moments(contour))

    # Find and print the center of the contour
    for moment in momentsContours:
        centerX = int(moment["m10"] / moment["m00"])
        centerY = int(moment["m01"] / moment["m00"])
        centers.append((centerX, centerY))

    return contours, centers

def houghCircles(image):
    """
    Detect circle in a single image using the hough transform

    Keyword arguments:
    image -- the image where the markers are searched (must be np.uint8, 
                in grayscale and single channel)
    """
    dist = image.shape[0]/3
    gray = image.copy()
    gray = cv.medianBlur(gray, 11)
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, dist,
                            param1=150, param2=10,
                            minRadius=0, maxRadius=10)
    radius = []
    centers = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            centers.append((i[0], i[1]))
            radius.append(i[2])

    return radius, centers

def findCenter(Image):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(Image, desired_encoding="mono8")
    image = image.astype(np.uint8)
    cont, cent = findMarkers(image)  

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/k01/ir/image_rect', Image, findCenter)

    # Keeps the node alive
    rospy.spin()

if __name__ == "__main__":
    listener()