#!/usr/bin/env python
# python practice_wander.py cmd_vel:=cmd_vel_mux/input/teleop
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead

    # #g_range_ahead = min(msg.ranges)
    # g_range_ahead = min(msg.ranges)
    #g_range_ahead = msg.ranges[(len(msg.ranges) / 2)]

    range_size = len(msg.ranges)
    g_range_ahead = reduce(lambda x, y: x + y, list(
         filter(lambda x: x >= 0, msg.ranges[range_size / 2:range_size * 3 / 4])))
    print 'g_range_ahead = %.1f' % g_range_ahead



g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    # if driving_forward:
    # # BEGIN FORWARD
    #     if (g_range_ahead < 1.0):
    #         #if sensor value is under 1.5
    #         # if(g_range_ahead > )
    #         driving_forward = False
    #         #state_change_time = rospy.Time.now() + rospy.Duration(3)
    #         print("Obstacle detect, stop 3s second")
    #
    # else:
    #     if rospy.Time.now() > state_change_time:
    #         driving_forward = True
    #         #state_change_time = rospy.Time.now() + rospy.Duration(5)

    twist = Twist()

    # if 130 < g_range_ahead < 204:
    #     twist.linear.x = 0
    #
    # else:
    #     twist.linear.x = 0.7
    cmd_vel_pub.publish(twist)
    rate.sleep()