#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from deu_car.msg import drive_key
from std_msgs.msg import String


class Follow_last_course:
    def __init__(self):
        self.drive_key = drive_key()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.drive_pub = rospy.Publisher('drive_last_course', drive_key, queue_size=1)
        self.twist = Twist()
        self.drive_route_count = 0
        self.speed = 0.5
        self.angular = 200

    def speed_cb(self, msg):
        if msg.data == "low":
            self.speed = 0.3
            self.angular = 212.5
        elif msg.data == "high":
            self.speed = 0.7
            self.angular = 200
        elif msg.data == "middle":
            self.speed = 0.5
            self.angular = 200


    def image_callback(self, msg):
        self.drive_key.key = ""
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])
        lower = numpy.array([0,0,128])
        upper = numpy.array([100,255,255])
        mask = cv2.inRange(hsv,lower,upper)

        """
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow_right = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
            
        """
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, w/2:w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            # BEGIN CONTROL
            cx = int(M['m10'] / M['m00'])
            err = (cx + 250) - w / 2
            self.twist.linear.x = self.speed
            self.twist.angular.z = -float(err) / (self.angular)  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist
            self.drive_key.key = "run"
            self.drive_pub.publish(self.drive_key)
            # END CONTROL

        cv2.imshow("Dasd", mask)
        cv2.waitKey(3)

rospy.init_node('drive_last_course')
follower = Follow_last_course()
rospy.spin()