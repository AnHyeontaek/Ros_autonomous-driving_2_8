#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from deu_car.msg import drive_key
from std_msgs.msg import String


class Follower:
    def __init__(self):
        self.drive_key = drive_key()
        #self.drive_key.key = "run"
        self.bridge = cv_bridge.CvBridge()
        self.speed_sub = rospy.Subscriber('speed', String, self.speed_cb)
        self.select_route_sub = rospy.Subscriber('select_route', String, self.select_route_cb)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.drive_pub = rospy.Publisher('drive', drive_key, queue_size=1)
        self.twist = Twist()
        self.only_left = False
        self.find_right = 0
        self.found_right = 0
        self.found_right2 = 0
        self.parking_start = 0
        self.speed = 0.5
        self.angular = 200

    def select_route_cb(self, msg):
        if msg.data == "1":
            self.only_left = False
        elif msg.data == "2":
            print("pub2")
            self.only_left = True
            self.find_right = 0
            self.parking_start = 0

        elif msg.data == "parking":
            print("pub3")
            self.only_left = True
            self.find_right = 1
            self.found_right = 0
            self.found_right2 = 0

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
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.drive_key.key = "run"

        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow_right = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow_not_find = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top+20
        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0
        if self.only_left:
            mask_yellow[0:h, w/2:w] = 0

        mask_yellow_right[0:search_top, 0:w] = 0
        mask_yellow_right[search_bot:h, 0:w] = 0
        mask_yellow_right[0:h, 0:w/2] = 0

        M_right = cv2.moments(mask_yellow_right)

        if self.find_right == 1:
            if M_right['m00'] > 0:
                self.found_right = 1
                self.found_right2 = 1
            else:
                self.found_right2 = 0
            if self.found_right == 1 and self.found_right2 == 0:
                self.find_right = 0
                self.only_left = 0
                self.parking_start = 1

        if self.parking_start == 1 and M_right['m00'] > 0:
            self.drive_key.key = "parking_start"
            self.drive_key.twist = Twist()
            self.drive_pub.publish(self.drive_key)

        M = cv2.moments(mask_yellow)
        #list of moments to find next center line if not find center line
        moments_list = list()

        if M['m00'] > 0:
            del moments_list[:]
            # BEGIN CONTROL
            cx_yellow = int(M['m10']/M['m00'])
            err = (cx_yellow + 250) - w / 2
            self.twist.linear.x = self.speed
            self.twist.angular.z = -float(err) / (self.angular)  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist
            self.drive_key.key = "run"
            self.drive_pub.publish(self.drive_key)
            # END CONTROL
        else:
            if self.parking_start:
                self.twist.linear.x = 0.2
                self.drive_key.twist = self.twist
                self.drive_key.key = "run"
                self.drive_pub.publish(self.drive_key)
            else:
                for i in range(7):
                    for j in range(5):
                        search_top_ = (i*h)/10
                        search_bot_ = ((i+1)*h)/10
                        mask_yellow_not_find[0:search_top_, 0:w] = 0
                        mask_yellow_not_find[search_bot_:h, 0:w] = 0
                        mask_yellow_not_find[0:h, 0:j*w/10] = 0
                        mask_yellow_not_find[0:h, (j+1)*w/10:w] = 0
                        M_temp = cv2.moments(mask_yellow_not_find)
                        if M_temp['m00'] > 0:
                            moments_list.append(M_temp)
                        mask_yellow_not_find[0:h, 0:w] = 1
                M_temp_last = moments_list.pop()
                cx_yellow = int(M_temp_last['m10'] / M_temp_last['m00'])
                err = (cx_yellow) - w / 2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 200  # 400: 0.1, 300: 0.15, 250, 0.2,0.25:225 212.5 0.3
                self.drive_key.twist = self.twist
                self.drive_pub.publish(self.drive_key)
        cv2.waitKey(3)

rospy.init_node('follower_yellow')
follower = Follower()
rospy.spin()