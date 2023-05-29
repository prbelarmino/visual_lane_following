#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import math
from lane_detection_classical import ClassicalLaneDetector
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt


class DeviationPublisher:

    def __init__(self):

        self.deviation_pub = rospy.Publisher("/ika_racer/lane_visual_following/lane_deviation", Float64, queue_size=10)
        self.__realsense_image_sub  = rospy.Subscriber("/ika_racer/perception/realsense/camera/color/image_raw", Image, self.__realsense_image_callback)
        self.__fisheye_image_sub  = rospy.Subscriber("/ika_racer/perception/fisheye_camera/image_raw", Image, self.__fisheye_image_callback)
        self.steering_angle_factor = 1.0
        self.intial_time = rospy.get_time()
        self.w = 2*math.pi*0.05
        self.img_bridge = CvBridge()
        self.cl_detector = ClassicalLaneDetector()

    def __realsense_image_callback(self, msg):

        self.deviation_pub.publish(math.sin(self.w*(rospy.get_time()-self.intial_time)))
        self.cl_detector

    def __fisheye_image_callback(self,msg):

        try: 
            cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            detected_lane = self.cl_detector.detect_pipeline(cv_image)
            #cv2.imshow("Detected Lane", detected_lane)
            #cv2.waitKey(3)

        except Exception as e:

            print(e)

if __name__=="__main__":

    rospy.init_node("lane_deviation_publisher")
    #rospy.sleep(9)
    publisher = DeviationPublisher()
    rospy.spin()