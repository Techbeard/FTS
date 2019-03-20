#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, Vector3
from sessel_otter.msg import MotorTicks
import serial
import struct

speed = 0
direction = 0
timeout = 0
ser1 = None
ser2 = None

TIMEOUT      = 10
MAX_SPEED    = 1 # m/s
MAX_STEERING = 0.1 # rad/s

import threading


def receiveSerial():
    r = rospy.Rate(250) # Hz
    global ser1
    pub = rospy.Publisher('motor_ticks', MotorTicks, queue_size = 1)
    while not rospy.is_shutdown():
        # try:
            # TODO: add odometry ####################################################
            # response = ser1.readline()
            #ticks = map(int, response.split(';'))
            #print(ticks[0])
            #print(ticks[1])
            #pub.publish(MotorTicks(ticks[0], ticks[1]))
        # except Exception as ex:
        #     print ex
        #     pass
        r.sleep()


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def callback(data):
    global speed, direction, timeout
    speed = clamp(data.linear.x, -MAX_SPEED, MAX_SPEED)
    direction = clamp(data.angular.z, -MAX_STEERING, MAX_STEERING)
    timeout = 0
    #rospy.loginfo("Speed: %f", speed)

def calc_checksum(data):
    # rospy.loginfo(data)
    sum = 0
    for b in data:
        sum += int(b)
    sum -= 0x02 # remove protocol header

    sum = 0 - sum
    return (sum & 0xFF)

def to_int16(value):
    byte_stream = struct.pack('h', value)
    b1 = struct.unpack('BB', byte_stream)
    # rospy.loginfo(b1)
    return b1

def main():
    global speed, direction, timeout, ser1, ser2
    rospy.init_node('hardware_driver')

    try:
        ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        rospy.logwarn("Using serial interface: %s, %s", ser1.name, ser2.name)
        connected = True
    except Exception:
        rospy.logerr("No Serial device found!")
        connected = False
        pass

    rospy.Subscriber("drive_command", Twist, callback)
    r = rospy.Rate(100) # Hz
    vel_tp = [0] * 50 # 50 sample low-pass for speed
    dir_tp = [0] * 10 # 10 sample low-pass for steering

    thread = threading.Thread(target=receiveSerial, args=[])
    thread.start()

    while not rospy.is_shutdown():
        vel_tp[len(vel_tp)-1] = speed if not timeout > TIMEOUT else 0
        vel_tp[:-1] = vel_tp[1:]

        dir_tp[len(dir_tp)-1] = direction
        dir_tp[:-1] = dir_tp[1:]

        tx_speed = sum(vel_tp)/len(vel_tp)
        tx_dir = sum(dir_tp)/len(dir_tp)

        #rospy.loginfo("Speed: %f", tx_speed)
        #rospy.loginfo("Steering: %f", tx_dir)

        motorR = tx_speed + tx_dir
        motorL= tx_speed - tx_dir

        # binR = struct.pack('f', motorR)
        # binL = struct.pack('f', motorL)

        # phail hoverboard protocol, see protocol.c in his repository
        speed_command_l = [0x02, 0x06, 0x07]
        speed_command_r = [0x02, 0x06, 0x07]

        # speed values, -1000 to 1000
        speed_command_l.extend(to_int16(motorL * 1000))
        speed_command_r.extend(to_int16(motorR * 1000))

        # values for direction, should be zero
        speed_command_l.extend([0x00, 0x00])
        speed_command_r.extend([0x00, 0x00])

        # calculate checksum
        speed_command_l.append(calc_checksum(speed_command_l))
        speed_command_r.append(calc_checksum(speed_command_r))
        
        debug_str = "[Motor command] left:" + str(speed_command_l) + "  right: " + str(speed_command_r)
        rospy.loginfo(debug_str)

        if connected:
            for b in speed_command_l:
                ser1.write(b)
            for b in speed_command_r:
                ser2.write(b)

        timeout+=1
        r.sleep()

main()
