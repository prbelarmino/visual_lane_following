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
        self.max_speed = 0.3
        self.drive_speed = 0.0
        self.previous_time = rospy.get_time()
        self.int_error = 0.0
        self.ki_speed = 0.3
        self.kp_steer = -2 #0.3/120
        self.kd_steer = 1/(self.max_speed*math.cos(math.radians(50))) #0.91 = cos 25
        self.kp_speed = 0.1
        self.yaw = 0.0
        self.delta_x = 0.0
        self.target_speed = 0.5 #range: 0.0 to 0.7 
        self.target_steering_angle = 1.0 #range: 0.0 to 1.0
        self.max_steering_angle = 0.85
        self.drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
        self.drive_cmd = DriveCommandStamped()
        self.drive_cmd.drive.driver_code = 1
        self.drive_cmd.drive.steering_angle_velocity = 2.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.acceleration = 0.69
        self.speed_sub  = rospy.Subscriber("/ika_racer/perception/encoder/speed_filtered", Float64Stamped, self.speed_callback)
        self.__lane_deviation_sub  = rospy.Subscriber("/ika_racer/lane_visual_following/lane_deviation", LaneMsg, self.__lane_deviation_callback)
        self.steering_angle_factor = 1.0
        self.x_left_offset = -3
        self.x_right_offset = 400
        self.upper_right_threshold =  -15
        self.lower_right_threshold = 10
        self.right_threshold_flag = False
        self.upper_left_threshold = 100
        self.lower_left_threshold = 90
        self.left_threshold_flag = False
        self.image_width = 400
        self.lane_timeout = rospy.get_time()
        self.vehicle_center_dis = 0.25
        self.pixel_to_meter = 0.3/400
        self.list_cmd =[]
        
    def speed_callback(self,msg):

        self.drive_speed  = msg.data
       
    def drive_speed_controller(self):

        error = self.max_speed - self.drive_speed 
        current_time = rospy.get_time()
        self.int_error += error*(current_time - self.previous_time)
        self.previous_time = current_time
        self.drive_cmd.drive.speed = self.kp_speed*error + self.ki_speed*self.int_error

    def __lane_deviation_callback(self, msg):

        self.left_lane = msg.left_lane
        self.right_lane = msg.right_lane
        self.get_deviation_and_slope()
        self.vehicle_controller()
        self.drive_speed_controller()
        #self.print_lanes_par()
        
        
        
    
    def vehicle_controller(self):

        #self.drive_speed = 0.3
        steer_cmd = self.kp_steer*self.delta_x + self.kd_steer*self.drive_speed*math.cos(self.yaw)
        print(self.delta_x,math.degrees(self.yaw),self.drive_speed,steer_cmd)
        #self.max_speed = 0.0
        steer_cmd = np.clip(steer_cmd,-1,1)
        #print(self.delta_x,math.degrees(self.yaw),self.kp_steer*self.delta_x,self.kd_steer*0.3*math.cos(self.yaw), steer_cmd)
        if not (len(self.left_lane) == 0 and len(self.right_lane) == 0):
            
            self.lane_timeout = rospy.get_time() 
            if len(self.left_lane) == 0:
                
                self.drive_cmd.drive.steering_angle = self.max_steering_angle

            elif len(self.right_lane) == 0:

                self.drive_cmd.drive.steering_angle = -self.max_steering_angle
           
            else: 

                self.drive_cmd.drive.steering_angle = 0.0

        self.drive_cmd.drive.steering_angle = steer_cmd
        if rospy.get_time() - self.lane_timeout > 0.5:

            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0

        self.drive_cmd_pub.publish(self.drive_cmd)

    def print_lanes_par(self):

        slp_r = "---"
        x_r = "---"
        slp_l = "---"
        x_l = "---"
        
        if len(self.left_lane) == 5:

            slp_l = str(self.left_lane[4])[0:4]
            x_l = str(self.left_lane[0])[0:3]
            
        if len(self.right_lane) == 5:

            slp_r = str(self.right_lane[4])[0:4]
            x_r = str(self.right_lane[0])[0:3]

        print("Left Slope: " + slp_l + ". Left X: " + x_l + ". Right Slope: " + slp_r + ". Right X: " + x_r)

    def get_deviation_and_slope(self):

        parameters = []
        if len(self.left_lane):

            parameters.append([self.left_lane[0],self.left_lane[4]])

        if len(self.right_lane):

            parameters.append([self.right_lane[0]-self.image_width,self.right_lane[4]])

        if len(parameters):

            parameters_np = np.array(parameters)
            parameters_np = np.average(parameters_np, axis=0)
            self.yaw = parameters_np[1]
            self.delta_x = self.pixel_to_meter*parameters_np[0]*math.sin(self.yaw) 
            #+ self.vehicle_center_dis*math.cos(self.yaw)  
            #print(self.pixel_to_meter*parameters_np[0]*math.sin(self.yaw),self.vehicle_center_dis*math.cos(self.yaw))
    def stop_vehicle(self):
        
        self.steering_angle_factor = 0.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd_pub.publish(self.drive_cmd)

if __name__=="__main__":

    rospy.init_node("lane_follower")
    follower = LaneFollower()
    rospy.on_shutdown(follower.stop_vehicle)
    rospy.spin()

