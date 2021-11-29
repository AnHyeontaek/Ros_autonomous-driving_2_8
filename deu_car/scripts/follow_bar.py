#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from deu_car.msg import drive_key
import time


class Follower_bar:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.drive_pub = rospy.Publisher('bar', drive_key, queue_size=1)
        self.drive_key = drive_key()
        self.drive_key.twist = Twist()

    def image_callback(self, msg):


        bar_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bar_image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([-10, 100, 100])
        upper_red = numpy.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = bar_image.shape
        search_h = h/2
        search_w = w/3
        mask_red[search_h:h, 0:w] = 0
        mask_red[0:h, 0:search_w] = 0

        M = cv2.moments(mask_red)
        if M['m00'] > 0:
            self.drive_key.key = "bar"
            self.drive_pub.publish(self.drive_key)
        else :
            self.drive_key.key = "go"
            self.drive_pub.publish(self.drive_key)

        cv2.waitKey(3)


rospy.init_node('detect')
detect = Follower_bar()
rospy.spin()