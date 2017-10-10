#!/usr/bin/env python
import sys

import serial
import rospy
from geometry_msgs.msg import PoseStamped
from videoray_usbl.msg import USBL


def open_usbl(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except ValueError as e:
        rospy.logerr('Serial port configuration error\n{}'.format(e))
        sys.exit()
    except serial.SerialException as e:
        rospy.logerr('Cannot open serial port\n{}'.format(e))
        sys.exit()
    return ser


def process_usbl_msg(usbl_msg):
    """:type usbl_msg:string"""
    valid = True

    data_array = usbl_msg.split(',')
    if data_array[0][0:2] != '%D' or data_array[0][6:] != '082031':
        rospy.logwarn('USBL msg format not allowed: {}'.format(data_array[0]))
        valid = False
    if data_array[6] != '30':
        rospy.logwarn('Valid reply error: {}'.format(data_array[6]))
        valid = False

    x = int(data_array[2]) / 1000.0
    y = int(data_array[3]) / 1000.0
    z = int(data_array[4]) / 1000.0
    rms_error = float(data_array[5])
    # use timestamp in ROS instead
    # timestamp = int(data_array[-1])

    return valid, x, y, z, rms_error


def usbl_pose(frame_id):
    pos = PoseStamped()
    pos.header.frame_id = frame_id
    def from_xyz(x, y, z, rms_error):
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
    	pos.pose.orientation.x = rms_error
        pos.header.stamp = rospy.Time.now()
        return pos
    return from_xyz


def start_usbl(port, baudrate, topic, frame_id):
    ser = open_usbl(port, baudrate)
    ser.readline()

    pub = rospy.Publisher(topic, USBL, queue_size=3)

    while not rospy.is_shutdown():
        for line in ser:
            valid, x, y, z, rms_error = process_usbl_msg(line)
            if valid:
                usbl = USBL()
                usbl.time = rospy.Time.now().to_sec()
                usbl.position.x = x
                usbl.position.y = y
                usbl.position.z = z
                usbl.rms = rms_error
                pub.publish(usbl)

    ser.close()

if __name__ == '__main__':
    rospy.init_node('videoray_usbl')
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 19200)
    topic = rospy.get_param('~topic', '~usbl_position')
    frame_id = rospy.get_param('~frame_id', '/odom')
    start_usbl(port, baudrate, topic, frame_id)

