#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import math
from visual_lane_following.msg import LaneMsg
from lane_detection_classical_v3 import ClassicalLaneDetector
from cv_bridge import CvBridge
import cv2
import csv


class DeviationPublisher:

    def __init__(self):

        self.lane_msg = LaneMsg()
        self.deviation_pub = rospy.Publisher("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, queue_size=1)
        self.__realsense_image_sub  = rospy.Subscriber("/ika_racer/perception/realsense/camera/color/image_raw", Image, self.__realsense_image_callback)
        #self.__fisheye_image_sub  = rospy.Subscriber("/ika_racer/perception/fisheye_camera/image_raw", Image, self.__fisheye_image_callback)
        #self.__ipm_image_sub  = rospy.Subscriber("/ika_racer/functions/ipm/bird_eye_view_image", Image, self.__ipm_image_callback)
        self.steering_angle_factor = 1.0
        self.intial_time = rospy.get_time()
        self.w = 2*math.pi*0.05
        self.img_bridge = CvBridge()
        self.cl_detector = ClassicalLaneDetector()
        self.list_cmd = []
    
    
    def __ipm_image_callback(self, msg):

        try: 
            cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Detected Lane", cv_image)
            cv2.waitKey(3)

        except Exception as e:

            print(e)

    def __realsense_image_callback(self, msg):

        #try:
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image, left_line, right_line = self.cl_detector.detect_pipeline(cv_image,"REALSENSE")
        self.lane_msg.left_lane = left_line
        self.lane_msg.right_lane = right_line

        slp_r = "---"
        x_r = "---"
        slp_l = "---"
        x_l = "---"
        if len(left_line) == 5:

            self.draw_line(image,left_line)
            slp_l = str(left_line[4])[0:4]
            x_l = str(left_line[0])[0:3]
            
        if len(right_line) == 5:

            self.draw_line(image,right_line)
            slp_r = str(right_line[4])[0:4]
            x_r = str(right_line[0])[0:3]

        print("Left Slope: " + slp_l + ". Left X: " + x_l + ". Right Slope: " + slp_r + ". Right X: " + x_r)   
        if True:
            cv2.imshow("Detected Lane", image)
            cv2.waitKey(10)
        
        self.deviation_pub.publish(self.lane_msg)

        #except Exception as e:

            #print(e)

    def __fisheye_image_callback(self,msg):

        #try:
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image, left_line, right_line = self.cl_detector.detect_pipeline(cv_image,"FISHEYE")
        slp_r = "---"
        x_r = "---"
        slp_l = "---"
        x_l = "---"
        if len(left_line) == 5:

            #self.draw_line(image,left_line)
            slp_l = str(left_line[4])[0:4]
            x_l = str(left_line[0])[0:3]
            
        if len(right_line) == 5:

            #self.draw_line(image,right_line)
            slp_r = str(right_line[4])[0:4]
            x_r = str(right_line[0])[0:3]

        #print("Left Slope: " + slp_l + ". Left X: " + x_l + ". Right Slope: " + slp_r + ". Right X: " + x_r)   
        if True:
            cv2.imshow("Detected Lane", image)
            cv2.waitKey(30)

        #except Exception as e:

            #print(e)
            
    def draw_line(self,image,line):

        line = [int(i) for i in line]
        cv2.line(image,(line[0],line[1]),(line[2],line[3]),(0,255,0),10)

if __name__=="__main__":

    rospy.init_node("lane_deviation_publisher")
    #rospy.sleep(9)
    publisher = DeviationPublisher()
    rospy.spin()