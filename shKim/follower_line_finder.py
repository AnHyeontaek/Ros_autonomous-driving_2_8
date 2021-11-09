#!/usr/bin/env python
#detected_white_and_yellow_lane
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("orignal_window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #detect_center
        lower = numpy.array([0, 0, 100])
        #lower_white, cuz Lower_white is lower than lower_yellow
        upper = numpy.array([100, 255, 255])
        #upper_yellow, cuz upper_yellow is upper than upper_yellow
        mask_center = cv2.inRange(hsv, lower, upper)
        masked_center = cv2.bitwise_and(image, image, mask=mask_center)
        # masked is show color use mask(binary screen)
        #####
        #detect_white
        #lower_white = numpy.array([0, 0, 100], dtype=numpy.uint8)
        #upper_white = numpy.array([0, 0, 255], dtype=numpy.uint8)
        # mask_white = cv2.inRange(hsv, lower_white, upper_white)
        #masked_white = cv2.bitwise_and(image, image, mask=mask_white)

        # BEGIN CROP
        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = search_top + 20
        mask_center[0:search_top, 0:w] = 0
        mask_center[search_bot:h, 0:w] = 0
        # END CROP
        # BEGIN FINDER
        M = cv2.moments(mask_center)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # END FINDER
            # BEGIN CIRCLE
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        # END CIRCLE

        cv2.imshow("orignal_window", image)
        cv2.imshow("hsv_center", masked_center)
        #cv2.imshow("hsv_yellow", masked_yellow)
        #cv2.imshow("hsv_white", masked_white)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL