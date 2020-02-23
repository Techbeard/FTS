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
MAX_STEERING = 0.3 # rad/s

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

# def calc_checksum(data):
#     # rospy.loginfo(data)
#     sum = 0
#     for b in data:
#         sum += int(b)
#     sum -= 0x02 # remove protocol header

#     sum = 0 - sum
#     return (sum & 0xFF)

# def to_int16(value):
#     byte_stream = struct.pack('h', value) # int16_t
#     b1 = struct.unpack('BB', byte_stream) # two uint8_t s
#     # rospy.loginfo(b1)
#     return b1

# def to_int32(value):
#     byte_stream = struct.pack('i', value) # int32_t
#     b1 = struct.unpack('BBBB', byte_stream)
#     return b1

# def generate_drive_command(speed, steer):
#     # phail hoverboard protocol, see protocol.c in his repository
#     cmd = [0x02, 0x00, ord('W'), 0x07]
#     speed_command_r = [0x02, 0x06, 0x07]

#     # speed values, -1000 to 1000
#     cmd.extend(to_int16(speed))

#     # direction values
#     cmd.extend(to_int16(steer))

#     # set length of data (will be data + checksum, so: current packet - header + checksum = -1)
#     cmd[1] = len(cmd) - 1

#     # calculate checksum
#     cmd.append(calc_checksum(cmd))

#     return cmd

# def generate_speed_command(speedL, speedR, maxPower = 600, minPower = -600, minSpeed = 40):
#     cmd = [0x02, 0x00, ord('W'), 0x03]

#     cmd.extend(to_int32(speedL))
#     cmd.extend(to_int32(speedR))
#     # cmd.extend(to_int32(maxPower))
#     # cmd.extend(to_int32(minPower))
#     # cmd.extend(to_int32(minSpeed))
#     # cmd.extend(to_int32(0)) # placeholder for reading back diff mm/s 0
#     # cmd.extend(to_int32(0)) # placeholder for reading back diff mm/s 1
#     # cmd.extend(to_int32(0)) # placeholder for reading back power demand 0
#     # cmd.extend(to_int32(0)) # placeholder for reading back power demand 1

#     # set length of data (will be data + checksum, so: current packet - header + checksum = -1)
#     cmd[1] = len(cmd) - 1

#     # calculate checksum
#     cmd.append(calc_checksum(cmd))

#     return cmd

def to_hexstring(bytestring):
    return ' '.join(['{:02X}'.format(x) for x in bytestring])

def generate_foc_drive_command(speed, steer):
    # emanuel FOC hoverboard protocol (https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)

    # start frame
    cmd = struct.pack('H', 0xAAAA)

    # direction values
    cmd += struct.pack('h', steer)

    # speed values, -1000 to 1000
    cmd += struct.pack('h', speed)

    # calculate checksum
    checksum = 0xAAAA ^ steer ^ speed
    cmd += struct.pack('H', checksum)

    return cmd


def main():
    global speed, direction, timeout, ser1, ser2
    rospy.init_node('hardware_driver')

    try:
        ser1 = serial.Serial('/dev/ttyUSB0/', 38400, timeout=1)
        ser2 = serial.Serial('/dev/ttyUSB1/', 38400, timeout=1)
        rospy.logwarn("Using serial interface: %s, %s", ser1.name, ser2.name)
        connected = True
    except Exception:
        rospy.logerr("No Serial device found!")
        connected = False
        pass

    rospy.Subscriber("cmd_vel", Twist, callback)
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

        motorR = (tx_speed + tx_dir) * 1000 # convert to tank control and mm/s
        motorL = (tx_speed - tx_dir) * 1000

        # binR = struct.pack('f', motorR)
        # binL = struct.pack('f', motorL)

        # command_l = generate_drive_command(motorL * 1000, 0)
        # command_r = generate_drive_command(motorR * 1000, 0)
        command_l = generate_foc_drive_command(motorL, 0)
        command_r = generate_foc_drive_command(motorR, 0)

        
        rospy.loginfo(debug_str)

        if connected:
            debug_str = "[Motor command] left:" + to_hexstring(command_l) + "  right: " + to_hexstring(command_r)
            ser1.write(command_l)
            ser2.write(command_r)

        timeout+=1
        r.sleep()

main()
