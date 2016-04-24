#!/usr/bin/env python
import os
import signal
import subprocess

import rospy
from rosbag import rosbag_main
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2
from videoray_control.msg import DesiredTrajectory, Throttle

valid_topic = "/videoray_sonar/valid"
autopilot_topic = "/videoray_input/desired_trajectory"
throttle_topic = "/videoray_input/throttle"
sonar_topic = "/sonar_mapping/filtered_sonarscan"

sonar_valid_pub = rospy.Publisher(valid_topic, Bool, queue_size=1000, latch=True)
autopilot_pub = rospy.Publisher(autopilot_topic, DesiredTrajectory, queue_size=1000, latch=True)
throttle_pub = rospy.Publisher(throttle_topic, Throttle, queue_size=1000, latch=True)


def set_autopilot(depth_enabled, heading_enabled, depth=0, heading=0):
    rospy.loginfo('Set autopilot : depth_enabled {}, depth {}, heading_enabled {}, heading {}'.
        format(depth_enabled, depth, heading_enabled, heading))
    d = DesiredTrajectory()
    d.heading_enabled = heading_enabled
    d.depth_enabled = depth_enabled
    d.depth = depth
    d.heading = heading
    autopilot_pub.publish(d)


def set_throttle(port, star, vert):
    rospy.loginfo('Set throttle cmd: port {}, start {}, vert {}'.format(port, star, vert))
    t = Throttle()
    t.VertInput = vert
    t.StarInput = star
    t.PortInput = port
    throttle_pub.publish(t)


def enable_sonar(enable):
    rospy.loginfo('Enable sonar {}'.format(enable))
    sonar_valid_pub.publish(Bool(enable))


def callback_shutdown():
    rospy.loginfo('Shutdown simple trajectory.')
    d = DesiredTrajectory()
    d.heading_enabled = False
    d.depth_enabled = False
    autopilot_pub.publish(d)


def go_deeper(depth):
    enable_sonar(False)
    set_autopilot(True, False, depth)
    rospy.sleep(10)
    enable_sonar(True)


def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.get_children(recursive=True):
        try:
            sub_process.send_signal(signal.SIGINT)
        except Exception:
            pass
    p.wait()  # we wait for children to terminate
    try:
        p.terminate()
    except Exception:
        pass


if __name__ == '__main__':
    rospy.init_node('simple_trajectory')
    callback_shutdown()

    # move_forward()
    enable_sonar(False)
    set_autopilot(True, False, 0.5, 0)
    rospy.sleep(10.0)

    rosbag_process = subprocess.Popen('rosbag record -a -O {}'.format('depth.bag'),\
        stdin=subprocess.PIPE, shell=True, cwd='/home/jinkun/Downloads/')
    rospy.sleep(0.5)

    enable_sonar(True)

    set_throttle(10, 10, 0)

    # full_scan_sub = rospy.Subscriber(sonar_topic, PointCloud2, go_deeper, queue_size=10)

    # for i in range(0, 5):
    #     rospy.sleep(10)
    #     go_deeper(0.5 + i * 0.1)

    rospy.sleep(30)
    terminate_process_and_children(rosbag_process)
    callback_shutdown()

    # rospy.on_shutdown(callback_shutdown)
    # rospy.spin()
