#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from visual_lane_following.msg import LaneMsg
from lane_detection_classical_v4 import ClassicalLaneDetector
from cv_bridge import CvBridge

class DeviationPublisher:

    def __init__(self):

        self.lane_msg = LaneMsg()
        self.deviation_pub = rospy.Publisher("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, queue_size=1)
        self.__realsense_image_sub  = rospy.Subscriber("/ika_racer/perception/realsense/camera/color/image_raw", Image, self.__realsense_image_callback)
        self.img_bridge = CvBridge()
        self.cl_detector = ClassicalLaneDetector()
    
    def __realsense_image_callback(self, msg):

        cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cl_detector.detection_pipeline(cv_image)
        self.lane_msg.left_lane = self.cl_detector.left_lane
        self.lane_msg.right_lane = self.cl_detector.right_lane
        self.lane_msg.yaw = self.cl_detector.yaw
        self.lane_msg.lateral_deviation = self.cl_detector.lateral_deviation
        self.deviation_pub.publish(self.lane_msg)
                 
if __name__=="__main__":

    rospy.init_node("lane_deviation_publisher")
    publisher = DeviationPublisher()
    rospy.spin()