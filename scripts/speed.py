#!/usr/bin/env python

import rospy
from acvp_msgs.msg import DriveCommandStamped, Float64Stamped

def speed_callback(msg):

    global cmd
    global int_error
    global previous_time
    ref = 0.5
    ki = 0.25
    kp = 0.1
    speed = msg.data
    error = ref - speed
    current_time = rospy.get_time()
    int_error += error*(current_time - previous_time)
    previous_time = current_time
    cmd.drive.speed = kp*error + ki*int_error
    print(error,int_error,cmd.drive.speed)
    # if error > 0.05 and cmd.drive.speed < 0.7:

    #     cmd.drive.speed  += 0.0001

    # elif error < -0.05 and cmd.drive.speed > 0.0:

    #     cmd.drive.speed  -= 0.0001
def stop():
    
    global cmd
    cmd.drive.speed = 0.0
    drive_cmd_pub.publish(cmd)
    
if __name__=="__main__":

    rospy.init_node("speed")
    speed_error = 0.0
    int_error = 0.0
    previous_time = rospy.get_time()
    cmd = DriveCommandStamped()

    cmd.drive.driver_code = 1
    cmd.drive.steering_angle_velocity = 2.0
    cmd.drive.speed  = 0.5
    cmd.drive.acceleration = 0.69
    rospy.on_shutdown(stop)
    
    drive_cmd_pub = rospy.Publisher("/ika_racer/locomotion/drive_command", DriveCommandStamped, queue_size=10)
    speed_sub  = rospy.Subscriber("/ika_racer/perception/encoder/speed_filtered", Float64Stamped, speed_callback)
    #rospy.sleep(9)
    while not rospy.is_shutdown():

        drive_cmd_pub.publish(cmd)
        rospy.sleep(0.25)

    

   