#!/usr/bin/env python
# python practice_wander.py cmd_vel:=cmd_vel_mux/input/teleop
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Obstacle_detecter:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.rospy.init_node('obstacle_detected')
        self.run = False

def scan_callback(self, msg):
    global g_range_ahead
    #g_range_ahead = min(msg.ranges)
    #g_range_ahead = msg.ranges[(len(msg.ranges) / 2)]
    if self.run:
        angle_180 = len(msg.ranges) / 2
        angle_90 = len(msg.ranges) / 4
        angle_45 = len(msg.ranges) / 8

    self.range_ahead = msg.ranges[len(msg.ranges) / 2]
    self.range_right = max(msg.ranges[len(msg.ranges) / 2:])

    twist = Twist()

    if self.range_ahead > 1.8 or self.range_right > 1.8 or \
            ((math.isnan(self.range_ahead)) and math.isnan(self.range_right)):
        twist.linear.x = 0.6
        print('go')

    else:
        twist.linear.x = 0
        print('stop')
