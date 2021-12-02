#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String


class Follower:
    def __init__(self):
        self.count = 1
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("orig_window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        masked_white = cv2.bitwise_and(image, image, mask=mask_white)

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

        if M['m01'] > 0:
            self.twist.linear.x = 0.7
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("orig_window", image)
        cv2.imshow("white_window", masked_white)

        cv2.waitKey(3)


rospy.init_node('follower_white')
follower = Follower()
rospy.spin()