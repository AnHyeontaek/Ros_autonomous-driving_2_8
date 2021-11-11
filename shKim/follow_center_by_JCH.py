#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        # self.image_sub = rospy.Subscriber('usb_cam/image_raw',
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # gray color
        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])
        #lower_white = numpy.array([0,0,100])
        #upper_white = numpy.array([0,0,255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked_center = cv2.bitwise_and(image, image, mask=mask_yellow)
        #mask_white = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top+20
        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0
        M = cv2.moments(mask_yellow)
        #M2 = cv2.moments(mask_white)
        if M['m00'] > 0:
            cx_yellow = int(M['m10'] / M['m00'])
            cy_yellow = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx_yellow + 250, cy_yellow), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = (cx_yellow + 250) - w / 2
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 212.5  # 400: 0.1, 300: 0.15, 250, 0.2
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
        cv2.imshow("window", image)
        cv2.imshow("mask", mask_yellow)
        cv2.imshow("masked", masked_center)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
