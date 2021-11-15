#!/usr/bin/env python
# python practice_wander.py cmd_vel:=cmd_vel_mux/input/teleop
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    print 'g_range_ahead = %.1f' % g_range_ahead

g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
    # BEGIN FORWARD
        if (g_range_ahead < 2.0):
            #if sensor value is under 1.5
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(3)
            print("Obstacle detect")

    else:
        if rospy.Time.now() > state_change_time:
            driving_forward = True
            state_change_time = rospy.Time.now() + rospy.Duration(5)

    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.5
        # else:
        #     twist.angular.z = 1#
    cmd_vel_pub.publish(twist)

    rate.sleep()
