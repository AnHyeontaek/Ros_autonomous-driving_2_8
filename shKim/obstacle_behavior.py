#!/usr/bin/env python

import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# BEGIN MEASUREMENT

class DetectObtacle:

    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def scan_callback(self, msg):
        range_center = msg.ranges[len(msg.ranges) / 2]
        # range_left = msg.ranges[len(msg.ranges) - 1]
        # range_right = msg.ranges[0]
        range_left = min(msg.ranges)
        range_right = max(msg.ranges)
        print "range ahead: left - %0.1f" % range_left, " center- %0.1f" % range_center, " right - %0.1f" % range_right
        # if range_center <= 2.5 or range_left <= 2 or range_left <= 2:
        #     rospy.loginfo("Detecting Obtacle")
        #     self.twist.linear.x = 0.0
        #     self.twist.angular.x = 0.0
        #     time.sleep(2)
        # else:
        #     self.twist.linear.x = 0.7
        #     self.twist.angular.x = 0.0
        # self.cmd_vel_pub.publish(self.twist)
        # if (range_center <= 1.0 and range_center >= 0.8) or (range_left <= 1.0 and range_left >= 0.8):
        #     rospy.loginfo("Detecting Obtacle")
        #     self.twist.linear.x = 0.0
        #     self.twist.angular.x = 0.0
        #     time.sleep(2)
        # else:
        #     self.twist.linear.x = 0.5
        #     self.twist.angular.x = 0.0
        # self.cmd_vel_pub.publish(self.twist)


# END MEASUREMENT
rospy.init_node('wander')
wander = DetectObtacle()
rospy.spin()