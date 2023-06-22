#!/usr/bin/env python

import rospy
from acvp_msgs.msg import DriveCommandStamped
from visual_lane_following.msg import LaneMsg
import csv
class LaneFollower:

    def __init__(self):

        self.left_lane = []
        self.right_lane = []
        self.drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
        self.drive_cmd = DriveCommandStamped()
        self.drive_cmd.drive.driver_code = 1
        self.drive_cmd.drive.steering_angle_velocity = 2.0
        self.drive_cmd.drive.speed = 0.0
        self.drive_cmd.drive.acceleration = 0.69
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
        self.lane_timeout = rospy.get_time()
        self.list_cmd =[]
        self.left_line = []
        self.right_line = []

    def __lane_deviation_callback(self, msg):

        self.left_line = msg.left_lane
        self.right_line = msg.right_lane
        self.vehicle_controller(msg)
    
    def vehicle_controller(self,lane):

        self.left_lane = lane.left_lane
        self.right_lane = lane.right_lane

        x = []
        if len(self.left_lane) > 0:
            self.lane_timeout = rospy.get_time()
            x.append(self.left_lane[0] - self.x_left_offset)

        if len(self.right_lane) > 0:
                self.lane_timeout = rospy.get_time()
                x.append(self.right_lane[0] - self.x_right_offset)

        x_len = len(x)
        if  x_len > 0:

            if len(self.left_lane) == 0:
                self.drive_cmd.drive.steering_angle = 0.75

            elif len(self.right_lane) == 0:
                self.drive_cmd.drive.steering_angle = -0.75

            else: 
                self.drive_cmd.drive.steering_angle = 0.0

            print(self.drive_cmd.drive.steering_angle)
        if False:
        #if  x_len > 0:

            x_avg = sum(x)/x_len
            # if x_avg <  self.lower_right_threshold:

            #     self.drive_cmd.drive.steering_angle = 0.75

            # elif  x_avg >  self.lower_left_threshold:

            #     self.drive_cmd.drive.steering_angle = -0.75

            # else:

            #     self.drive_cmd.drive.steering_angle = 0.0
            if x_avg <  self.upper_right_threshold:

                self.drive_cmd.drive.steering_angle = 0.75

            elif  x_avg >  self.upper_left_threshold:

                self.drive_cmd.drive.steering_angle = -0.75

            else:

                if  x_avg <  self.lower_left_threshold and x_avg > self.lower_right_threshold:

                     self.drive_cmd.drive.steering_angle = 0.0

            #print(x_avg,self.drive_cmd.drive.steering_angle)

        if rospy.get_time() - self.lane_timeout > 0.5:
            print(0.0)
            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0

        else:

            self.drive_cmd.drive.speed = 1.0



        #self.drive_cmd.drive.steering_angle = self.steering_angle_factor * deviation
        self.drive_cmd_pub.publish(self.drive_cmd)

if __name__=="__main__":

    rospy.init_node("lane_follower")
    #rospy.sleep(9)
    follower = LaneFollower()
    rospy.spin()
    #stop = raw_input()
    k = ""
    while k == "s":

        #self.vehicle_controller(msg)
        slp_r = "---"
        x_r = "---"
        slp_l = "---"
        x_l = "---"
        if len(follower.left_line) == 5:

            slp_l = str(follower.left_line[4])[0:4]
            x_l = str(follower.left_line[0])[0:3]
            
        if len(follower.right_line) == 5:

            slp_r = str(follower.right_line[4])[0:4]
            x_r = str(follower.right_line[0])[0:3]

        print("Left Slope: " + slp_l + ". Left X: " + x_l + ". Right Slope: " + slp_r + ". Right X: " + x_r)   
        k = raw_input("enter cmd: ")
        if k == "1" or k =="0" or k=="-1":

            follower.list_cmd.append([x_l,slp_l,x_r,slp_r,k])

  

    with open('GFG', 'w') as f:
      
                # using csv.writer method from CSV package
                write = csv.writer(f)
                
                write.writerows(follower.list_cmd)
    follower.steering_angle_factor = 0.0
    follower.drive_cmd.drive.speed = 0.0
    follower.drive_cmd_pub.publish(follower.drive_cmd)
