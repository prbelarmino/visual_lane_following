#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import math
from visual_lane_following.msg import LaneMsg
from lane_detection_classical_v4 import ClassicalLaneDetector
from cv_bridge import CvBridge
import cv2
import numpy as np


class DeviationPublisher:

    def __init__(self):

        self.lane_msg = LaneMsg()
        self.deviation_pub = rospy.Publisher("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, queue_size=1)
        self.__realsense_image_sub  = rospy.Subscriber("/ika_racer/perception/realsense/camera/color/image_raw", Image, self.__realsense_image_callback)
        #self.__fisheye_image_sub  = rospy.Subscriber("/ika_racer/perception/fisheye_camera/image_raw", Image, self.__fisheye_image_callback)
        #self.__ipm_image_sub  = rospy.Subscriber("/ika_racer/functions/ipm/bird_eye_view_image", Image, self.__ipm_image_callback)
        self.steering_angle_factor = 1.0
        self.intial_time = rospy.get_time()
        self.img_bridge = CvBridge()
        self.cl_detector = ClassicalLaneDetector()
        self.list_cmd = []
        self.width = 424
        self.height = 240
        self.lanes = []
    

    def __realsense_image_callback(self, msg):

        cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cl_detector.detection_pipeline(cv_image)
        self.lane_msg.left_lane = self.cl_detector.left_lane
        self.lane_msg.right_lane = self.cl_detector.right_lane
        
        self.deviation_pub.publish(self.lane_msg)
        self.cl_detector.plot_flag = False
        if self.cl_detector.plot_flag and cv_image.shape == self.cl_detector.output.shape:

            stack_image = np.hstack((cv_image, self.cl_detector.output))
            filtered3 =  np.stack((self.cl_detector.filtered_hsv,) * 3, axis=-1)
            stack_image2 = np.hstack((self.cl_detector.hsv_image,filtered3))
            final_stack =  np.vstack((stack_image, stack_image2))
            
            cv2.imshow("image", final_stack)
            cv2.waitKey(3)
                 
        
    
if __name__=="__main__":

    rospy.init_node("lane_deviation_publisher")
    publisher = DeviationPublisher()
    rospy.spin()