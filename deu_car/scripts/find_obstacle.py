#!/usr/bin/env python
import math

import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from deu_car.msg import drive_key

# BEGIN MEASUREMENT

class DetectObtacle:

    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.find_obstacle_pub = rospy.Publisher('find_obstacle', drive_key, queue_size=1)
        self.drive_key = drive_key()
        self.drive_key.key = "obstacle"
        self.twist = Twist()

    def scan_callback(self, msg):
        range_center = msg.ranges[len(msg.ranges) / 2]

        range_right = max(msg.ranges)

        if range_center > 2.3 or range_right > 1.6 or \
                ((math.isnan(range_center)) and math.isnan(range_right)):
            self.twist.linear.x = 1.0
            self.drive_key.key = "no_obstacle"
        else:
            self.twist.linear.x = 0.0
            self.drive_key.key = "obstacle"
        self.drive_key.twist = self.twist
        self.find_obstacle_pub.publish(self.drive_key)

rospy.init_node('DetectObtacle')
wander = DetectObtacle()
rospy.spin()