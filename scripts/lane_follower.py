#!/usr/bin/env python

import rospy
from acvp_msgs.msg import DriveCommandStamped
from std_msgs.msg import Float64

class LaneFollower:

    def __init__(self):

        self.drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
        self.drive_cmd = DriveCommandStamped()
        self.drive_cmd.drive.driver_code = 1
        self.drive_cmd.drive.steering_angle_velocity = 2.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.acceleration = 0.69
        self.__lane_deviation_sub  = rospy.Subscriber("/ika_racer/lane_visual_following/lane_deviation", Float64, self.__lane_deviation_callback)
        self.steering_angle_factor = 1.0

    def __lane_deviation_callback(self, msg):

        self.vehicle_controller(msg.data)
    
    def vehicle_controller(self,deviation):

        self.drive_cmd.drive.steering_angle = self.steering_angle_factor * deviation
        self.drive_cmd_pub.publish(self.drive_cmd)

if __name__=="__main__":

    rospy.init_node("lane_follower")
    #rospy.sleep(9)
    follower = LaneFollower()
    rospy.spin()
    follower.steering_angle_factor = 0.0
    follower.drive_cmd.drive.speed = 0.0
    follower.drive_cmd_pub.publish(follower.drive_cmd)
