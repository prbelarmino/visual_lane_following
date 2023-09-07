#!/usr/bin/env python

import rospy
from acvp_msgs.msg import DriveCommandStamped, Float64Stamped
from visual_lane_following.msg import LaneMsg
import numpy as np
import math

class LaneFollower:

    def __init__(self):

        self.left_lane = []
        self.right_lane = []
        self.initial_time_flag = True
        self.target_speed = 0.60 #range: 0.0(0.5) to 1.0
        self.current_speed = 0.0
        self.int_error_speed = 0.0
        self.kp_speed = 0.25
        self.ki_speed = 0.3
        self.kp_steer = -1.25 #0.3/120
        self.kd_steer = 2.35
        self.yaw = 0.0
        self.lateral_deviation = 0.0
        self.drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
        self.drive_cmd = DriveCommandStamped()
        self.drive_cmd.drive.driver_code = 1
        self.drive_cmd.drive.steering_angle_velocity = 2.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.acceleration = 0.69
        self.speed_sub  = rospy.Subscriber("/ika_racer/perception/encoder/speed_filtered", Float64Stamped, self.speed_callback)
        self.__lane_deviation_sub  = rospy.Subscriber("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, self.__lane_deviation_callback)
        self.lane_timeout = rospy.get_time()
        
    def speed_callback(self,msg):

        self.current_speed  = msg.data

    def __lane_deviation_callback(self, msg):

        self.left_lane = msg.left_lane
        self.right_lane = msg.right_lane
        self.yaw = msg.yaw
        self.lateral_deviation = msg.lateral_deviation
        if len(self.left_lane) + len(self.right_lane) > 0:
            self.lane_timeout = rospy.get_time()
       
        self.vehicle_controller()
        
    def vehicle_controller(self):

        if self.initial_time_flag:
            self.previous_time = rospy.get_time()
            self.initial_time_flag = False

        speed_error = self.target_speed - self.current_speed 
        current_time = rospy.get_time()
        self.int_error_speed += speed_error*(current_time - self.previous_time)
        self.previous_time = current_time
        speed_cmd = self.ki_speed*self.int_error_speed + self.kp_speed*speed_error 
        steer_cmd = self.kp_steer*self.lateral_deviation + self.kd_steer* self.current_speed *math.cos(self.yaw)
        #print(self.current_speed , self.lateral_deviation,math.degrees(self.yaw), self.current_speed, steer_cmd)
        speed_cmd = np.clip(speed_cmd, 0, 1)
        steer_cmd = np.clip(steer_cmd, -1, 1)
        self.drive_cmd.drive.speed = speed_cmd
        self.drive_cmd.drive.steering_angle = steer_cmd
        if rospy.get_time() - self.lane_timeout > 0.5:
            print("NO LANE TIMEOUT")
            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0

        self.drive_cmd_pub.publish(self.drive_cmd)

    def stop_vehicle(self):
        
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.steering_angle = 0.0
        self.drive_cmd_pub.publish(self.drive_cmd)
        print("NODE HAS BEEN SHUTDOWN")

if __name__=="__main__":

    rospy.init_node("lane_follower")
    follower = LaneFollower()
    rospy.on_shutdown(follower.stop_vehicle)
    rospy.spin()

