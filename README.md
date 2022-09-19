# Marker Finder

This repository contains the first of the two ROS package developed for my master thesis at the University of Padua.
In particular this ROS package searches and finds the position of passive markers inside an IR image.

## Usage

To use this repository with a kinect camera it is sufficient to modify the input parameters inside the marker.launch file, such as inTopic and outTopic. InTopic is the topic where the videcamera publishes the images, outTopic is the one where the ros node publishes the images with the markers found.