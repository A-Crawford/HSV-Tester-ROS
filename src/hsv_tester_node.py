#! /usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def nothing(x):
    pass

class HSVTester():
    def __init__(self, topic):
        
        # Create a window
        cv2.namedWindow('image')

        # Create trackbars for color change
        # Hue is from 0-179 for Opencv
        cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
        cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
        cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
        cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

        # Set default value for Max HSV trackbars
        cv2.setTrackbarPos('HMax', 'image', 179)
        cv2.setTrackbarPos('SMax', 'image', 255)
        cv2.setTrackbarPos('VMax', 'image', 255)

        # Initialize HSV min/max values
        self.hMin = self.sMin = self.vMin = self.hMax = self.sMax = self.vMax = 0
        self.phMin = self.psMin = self.pvMin = self.phMax = self.psMax = self.pvMax = 0
        
        # Define Pubs, Subs and Service clients
        # Define Service proxy
        # self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.HSVTest)
        self.image_sub = rospy.Subscriber(str(topic), Image, self.HSVTest)
        
        # Bridge needed for OpenCV to work with ROS
        self.bridge = CvBridge()
        

    def HSVTest(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        while(1):
            # Get current positions of all trackbars
            self.hMin = cv2.getTrackbarPos('HMin', 'image')
            self.sMin = cv2.getTrackbarPos('SMin', 'image')
            self.vMin = cv2.getTrackbarPos('VMin', 'image')
            self.hMax = cv2.getTrackbarPos('HMax', 'image')
            self.sMax = cv2.getTrackbarPos('SMax', 'image')
            self.vMax = cv2.getTrackbarPos('VMax', 'image')

            # Set minimum and maximum HSV values to display
            lower = np.array([self.hMin, self.sMin, self.vMin])
            upper = np.array([self.hMax, self.sMax, self.vMax])

            # Convert to HSV format and color threshold
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(image, image, mask=mask)

            # Print if there is a change in HSV value
            if((self.phMin != self.hMin) | (self.psMin != self.sMin) | (self.pvMin != self.vMin) | (self.phMax != self.hMax) | (self.psMax != self.sMax) | (self.pvMax != self.vMax) ):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (self.hMin , self.sMin , self.vMin, self.hMax, self.sMax , self.vMax))
                self.phMin = self.hMin
                self.psMin = self.sMin
                self.pvMin = self.vMin
                self.phMax = self.hMax
                self.psMax = self.sMax
                self.pvMax = self.vMax

            # Display result image
            cv2.imshow('image', result)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('HSV_Tester', anonymous=True)
    topicData = rospy.get_param('/hsv_tester/camera_topic')
    ic = HSVTester(topic=topicData)
    while not rospy.is_shutdown():
        rospy.spin()
        