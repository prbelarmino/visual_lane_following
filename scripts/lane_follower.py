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
        self.target_speed = 0.6 #range: 0.0(0.5) to 0.7
        self.target_steering_angle = 1.0 #range: -1.0 to 1.0
        self.current_speed = 0.0
        self.int_error_speed = 0.0
        self.kp_speed = 0.25
        self.ki_speed = 0.075
        self.ki_speed = 0.3
        self.int_error_steer = 0.0
        self.kp_steer = -1.25 #0.3/120
        self.kd_steer = 2.35
        self.yaw = 0.0
        self.delta_x = 0.0
        self.drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
        self.drive_cmd = DriveCommandStamped()
        self.drive_cmd.drive.driver_code = 1
        self.drive_cmd.drive.steering_angle_velocity = 2.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.acceleration = 0.69
        self.speed_sub  = rospy.Subscriber("/ika_racer/perception/encoder/speed_filtered", Float64Stamped, self.speed_callback)
        self.__lane_deviation_sub  = rospy.Subscriber("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, self.__lane_deviation_callback)
        self.xr_offset = 400
        self.lane_timeout = rospy.get_time()
        self.vehicle_center_dis = 0.25
        self.pixel_to_meter = 0.3/400
        
    def speed_callback(self,msg):

        self.current_speed  = msg.data

    def __lane_deviation_callback(self, msg):

        self.left_lane = msg.left_lane
        self.right_lane = msg.right_lane
        if len(self.left_lane) + len(self.right_lane) > 0:
            self.lane_timeout = rospy.get_time()
        self.get_deviation_and_slope()
       
        self.vehicle_controller()
        
    def vehicle_controller(self):

        if self.initial_time_flag:
            self.previous_time = rospy.get_time()
            self.initial_time_flag = False

        speed_error = self.target_speed - self.current_speed 
        current_time = rospy.get_time()
        self.int_error_speed += speed_error*(current_time - self.previous_time)
        self.int_error_steer += self.delta_x*(current_time - self.previous_time)
        self.previous_time = current_time
        speed_cmd = self.ki_speed*self.int_error_speed + self.kp_speed*speed_error 
        steer_cmd = self.kp_steer*self.delta_x + self.kd_steer* self.current_speed *math.cos(self.yaw)
        print(self.current_speed , self.delta_x,math.degrees(self.yaw), self.int_error_steer, steer_cmd)
        speed_cmd = np.clip(speed_cmd, 0, 0.65)
        steer_cmd = np.clip(steer_cmd, -1, 1)
        self.drive_cmd.drive.speed = speed_cmd
        self.drive_cmd.drive.steering_angle = steer_cmd
        if rospy.get_time() - self.lane_timeout > 0.5:
            print("NO LANE TIMEOUT")
            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0

        self.drive_cmd_pub.publish(self.drive_cmd)

    def get_deviation_and_slope(self):

        parameters = []
        if len(self.left_lane):

            parameters.append([self.left_lane[0],self.left_lane[4]])

        if len(self.right_lane):

            parameters.append([self.right_lane[0]-self.xr_offset,self.right_lane[4]])

        if len(parameters):

            parameters_np = np.array(parameters)
            parameters_np = np.average(parameters_np, axis=0)
            self.yaw = parameters_np[1]
            self.delta_x = self.pixel_to_meter*parameters_np[0]*math.sin(self.yaw) 
            
    def stop_vehicle(self):
        
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.steering_angle = 0.0
        self.drive_cmd_pub.publish(self.drive_cmd)
        print("NODE HAS BEEN SHUTDOWN")
        print(self.kd_steer)

if __name__=="__main__":

    rospy.init_node("lane_follower")
    follower = LaneFollower()
    rospy.on_shutdown(follower.stop_vehicle)
    rospy.spin()

