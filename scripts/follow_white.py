#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String
from deu_car.msg import drive_key
import threading

class Follower:
    def __init__(self):
        self.count = 1
        self.bridge = cv_bridge.CvBridge()
        self.drive_pub = rospy.Publisher('drive', drive_key, queue_size=1)
        self.drive_pub_2 = rospy.Publisher('drive_2', drive_key, queue_size=1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.twist = Twist()
        self.drive_key = drive_key()
        self.drive_key.key = "stop"

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        search_left = w/2 - 10
        search_right = search_left + 20
        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0
        mask_white[0:h, 0:search_left] = 0
        mask_white[0:h, search_right:w] = 0
        M = cv2.moments(mask_white)
        if M['m00'] > 0:
            self.twist.linear.x = 0
            self.drive_key.twist = self.twist
            self.drive_pub.publish(self.drive_key)
            self.drive_pub_2.publish(self.drive_key)
        cv2.waitKey(3)


rospy.init_node('follower_white')
follower = Follower()
rospy.spin()