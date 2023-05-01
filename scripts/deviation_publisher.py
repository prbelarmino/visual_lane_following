#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import math

class DeviationPublisher:

    def __init__(self):

        self.deviation_pub = rospy.Publisher("/ika_racer/lane_visual_following/lane_deviation", Float64, queue_size=10)
        self.__realsense_image_sub  = rospy.Subscriber("/ika_racer/perception/realsense/camera/color/image_raw", Image, self.__realsense_image_callback)
        self.steering_angle_factor = 1.0
        self.intial_time = rospy.get_time()
        self.w = 2*math.pi*0.02

    def __realsense_image_callback(self, msg):

        self.deviation_pub.publish(math.sin(self.w*(rospy.get_time()-self.intial_time)))
    

if __name__=="__main__":

    rospy.init_node("lane_deviation_publisher")
    #rospy.sleep(9)
    publisher = DeviationPublisher()
    rospy.spin()