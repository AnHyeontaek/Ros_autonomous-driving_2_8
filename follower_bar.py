#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower_bar:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        # self.image_sub = rospy.Subscriber('usb_cam/image_raw',
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        bar_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bar_image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([0, 0, 96]) 
        upper_red = numpy.array([0, 0, 102]) 
        #lower_red = numpy.array([0, 30, 30])
        #upper_red = numpy.array([10, 255, 130])

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        #mask_white = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = bar_image.shape
       # search_top = 1
        search_top = 0
        search_bot = 3 * h / 4
        mask_red[0:search_top, 0:w] = 0
        mask_red[search_bot:h, 0:w] = 0
        M = cv2.moments(mask_red)
        #M2 = cv2.moments(mask_white)
        if M['m00'] > 0:
            cx_red = int(M['m10'] / M['m00'])
            cy_red = int(M['m01'] / M['m00'])
            cv2.circle(bar_image, (cx_red + 250, cy_red), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = (cx_red + 250) - w / 2
            self.twist.linear.x = 0.0
            # END CONTROL
        cv2.imshow("window", bar_image)
        cv2.imshow("mask", mask_red)
        cv2.waitKey(3)


rospy.init_node('detect')
detect = Follower_bar()
rospy.spin()
