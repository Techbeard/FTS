#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, Vector3
import struct
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_servo import BrickServo

HOST = "192.168.178.114"
PORT = 4223
UID = "6rH3rG" # Change XXYYZZ to the UID of your Servo Brick



speed = 0
direction = 0
timeout = 0
ser = None

TIMEOUT      = 10
MAX_SPEED    = 1 # m/s
MAX_STEERING = 0.1 # rad/s

import threading



def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def callback(data):
    global speed, direction, timeout
    speed = clamp(data.linear.x, -MAX_SPEED, MAX_SPEED)
    direction = clamp(data.angular.z, -MAX_STEERING, MAX_STEERING)
    timeout = 0
    #rospy.loginfo("Speed: %f", speed)

def main():
    global speed, direction, timeout, ser
    rospy.init_node('ros_servo')

    ipcon = IPConnection() # Create IP connection
    servo = BrickServo(UID, ipcon) # Create device object
    ipcon.connect(HOST, PORT)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    r = rospy.Rate(100) # Hz
    vel_tp = [0] * 50 # 50 sample low-pass for speed
    dir_tp = [0] * 10 # 10 sample low-pass for steering

  



    while not rospy.is_shutdown():
        vel_tp[len(vel_tp)-1] = speed #if not timeout > TIMEOUT else 0
        vel_tp[:-1] = vel_tp[1:]

        dir_tp[len(dir_tp)-1] = direction
        dir_tp[:-1] = dir_tp[1:]

        tx_speed = (sum(vel_tp)/len(vel_tp))*18000
        tx_dir = (sum(dir_tp)/len(dir_tp))*-90000

        rospy.loginfo("Speed: %f", tx_speed)
        rospy.loginfo("Steering: %f", tx_dir)

       #motorR = tx_speed + tx_dir
       #motorL= tx_speed - tx_dir

        servo.set_degree(0, -9000, 9000)
        
        servo.set_velocity(0, 65535)
        servo.set_position(0, tx_dir)
        servo.enable(0)
       
        servo.set_degree(1, -9000, 9000)
        servo.set_velocity(1, 65535)
        servo.set_position(1, tx_speed)
        servo.enable(1)

        timeout+=1
        r.sleep()

main()
