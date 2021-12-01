#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from deu_car.msg import drive_key


class Follower:
    def __init__(self):
        self.count = 1
        self.bridge = cv_bridge.CvBridge()
        self.parking_pub = rospy.Publisher('parking', drive_key, queue_size=1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.switch_sub = rospy.Subscriber('park', String, self.switch)
        self.twist = Twist()
        self.drive_key = drive_key()
        self.drive_key.key = "run"
        self.parking = False
        self.parking_yellow = False

    def switch(self, msg):
        self.parking = True

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 4 * h / 5 + 20
        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0
        mask_white[0:h, w/2:w] = 0

        search_top_yellow = 4 * h / 5
        search_bot_yellow = search_top_yellow + 20
        search_left_yellow = w / 2 - 10
        search_right_yellow = search_left_yellow + 20
        mask_yellow[0:search_top_yellow, 0:w] = 0
        mask_yellow[search_bot_yellow:h, 0:w] = 0
        mask_yellow[0:h, 0:search_left_yellow] = 0
        mask_yellow[0:h, search_right_yellow:w] = 0

        M = cv2.moments(mask_white)
        M_yellow = cv2.moments(mask_yellow)

        if self.parking and M_yellow['m00'] > 0:
            self.parking = False
            self.twist = Twist()
            self.drive_key.twist = self.twist
            self.drive_key.key = "parking_end"
            self.parking_pub.publish(self.drive_key)
            return

        if self.parking and M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            err = (cx + 200) - w / 2
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 212.5  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist
            self.parking_pub.publish(self.drive_key)

        #if self.parking and M_yellow['m00'] > 0:
        #    self.parking_yellow = True

        cv2.waitKey(3)


rospy.init_node('follower_park')
follower = Follower()
rospy.spin()