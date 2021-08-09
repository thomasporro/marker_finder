import numpy as np
import rosbag
import cv2
from cv_bridge import CvBridge
import os
import marker_finder

bagFilePath = os.path.join(os.path.dirname(__file__), '../bagfiles/marker_calibration/marker_ir_test.bag')
bag = rosbag.Bag(bagFilePath)
windowName = "Markers"


if __name__ == "__main__":
    
    print("DETECTED CIRCLE: ", end='')
    cv2.namedWindow(windowName, cv2.WINDOW_AUTOSIZE)
    bridge = CvBridge()

    for topic, message, time in bag.read_messages(topics=['/k01/ir/image_rect']):
    
        image = bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        image = image.astype(np.uint8)
        
        color = image.copy()
        color = cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)

        # Marker detection using the most reliable method
        cont, cent = marker_finder.find_markers(image)
        for c, ce in zip(cont, cent):
            cv2.drawContours(color, c, -1, (0,255,0), 2)
            cv2.circle(color, (ce[0], ce[1]), 3, (0, 0, 255), -1)

        # Alternatives using the hough transform
        # radius, cent = marker_finder.hough_circles(image)
        # for r, c, in zip(radius, cent):
        #     cv2.circle(color, c, 2, (0, 0, 255), -1)
        #     cv2.circle(color, c, r, (0, 255, 0), 2)

        if cent:
            print('+', end='')
            
        cv2.imshow(windowName, color)
        cv2.waitKey(20)

