#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CVBridgeError
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String

class obstDetect():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("need to find", String, queue_size=10)
        self.focal_length = 0.20
        self.cam_dist = 0.10
        self.pix_size = 0.01

    def callback(self, img):
        #convert img msg to cv2 img using bridge
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_img2 = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
        except CvBridge:
            rospy.loginfo("failed conversion to cv2 format")
        
        contour = self.imageProcessing(cv_img)
        #cv2.drawContours(cv_img, contour, -1, (0,255,0), 2, cv2.LINE_AA)

        start = tuple(contour[0][0])
        l = len(contour)/2
        end = tuple(contour[l][0])

        location = self.templateMatching(cv_img, start,end)
        disparity = location[0] - start[0]
        depth = float(self.focal_length * self.cam_dist / (self.pix_size*disparity))

        if depth<0.15:
            pub.publish("initiate avoidance sequence")




    def imageProcessing(self, cv_img):

        cv_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        cv_blur = cv2.GaussianBlur(cv_gray, (3,3), 0)
        cv_edge = cv2.Canny(cv_blur, 100, 200)
        _, contours, _ = cv2.findContours(cv_edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            areas.append(area)
            if maximumArea<area:
                maximumArea = area
                index = i
        contour = contours[index]
        return contour
    
    def templateMatching(self, cv_img, cv_img2, start, end):
        roi1 = cv_img[start[1]:end[1], start[0]:end[0]]
        roi2 = cv_img2[start[1]:end[1],:]
        cv_temp = cv2.matchTemplate(roi2, roi1, eval('cv2.TM_CCOEFF'))
        _, _, _, max_loc = cv2.minMaxLoc(cv_temp)
        return max_loc


if __init__=="__main__":
    rospy.init_node('obstacle_detection')
    ob = obstDetect()
    image_sub = rospy.Subscriber("need to find", Image, ob.callback)
    rospy.spin()
    

