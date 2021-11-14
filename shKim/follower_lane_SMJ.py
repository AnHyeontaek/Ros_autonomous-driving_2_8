#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("lane_follow_window", 1)
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
        # detect_center
        lower = numpy.array([0, 0, 100])
        # lower_white, cuz Lower_white is lower than lower_yellow
        upper = numpy.array([100, 255, 255])
        # upper_yellow, cuz upper_yellow is upper than upper_yellow
        mask_center = cv2.inRange(hsv, lower, upper)
        masked_center = cv2.bitwise_and(image, image, mask=mask_center)

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 50
        mask_center[0:search_top, 0:w] = 0
        mask_center[search_bot:h, 0:w] = 0
        M = cv2.moments(mask_center)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self.twist.linear.x = 0.15
            self.twist.angular.z = -float(err) / 300  # 400: 0.1, 300: 0.15, 250, 0.2
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
        cv2.imshow("lane_follow_window", image)
        cv2.imshow("mask", masked_center)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL