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
        self.select_route = 1
        self.drive_key.key = "run"
        self.bridge = cv_bridge.CvBridge()
        self.select_route_sub = rospy.Subscriber('select_route', String, self.select_route_cb)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.drive_pub = rospy.Publisher('drive_2', drive_key, queue_size=1)
        self.twist = Twist()

    def select_route_cb(self, msg):
        if msg.data == "1":
            self.select_route = 1
        elif msg.data == "2":
            self.select_route = 2

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow_ = cv2.inRange(hsv, lower_yellow, upper_yellow) # mask to find new yellow if car can't find yellow line
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top+20
        if self.select_route == 1:
            print("ok")
            mask_yellow[0:search_top, 0:w] = 0
            mask_yellow[search_bot:h, 0:w] = 0
            mask_yellow[0:h, 0:w/2] = 0
        elif self.select_route == 2:
            print("ok2")
            mask_yellow[0:search_top, 0:w] = 0
            mask_yellow[search_bot:h, 0:w] = 0
        M = cv2.moments(mask_yellow)

        #list of moments to find next center line if not find center line
        moments_list = list()

        if M['m00'] > 0:
            del moments_list[:]
            # BEGIN CONTROL
            cx_yellow = int(M['m10']/M['m00'])
            err = (cx_yellow - 250) - w / 2
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 212.5  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist
            self.drive_pub.publish(self.drive_key)
            # END CONTROL
        else:
            for i in range(7):
                for j in range(5):
                    search_top_ = (i*h)/10
                    search_bot_ = ((i+1)*h)/10
                    mask_yellow_[0:search_top_, 0:w] = 0
                    mask_yellow_[search_bot_:h, 0:w] = 0
                    mask_yellow_[0:h, 0:j*w/10] = 0
                    mask_yellow_[0:h, (j+1)*w/10:w] = 0
                    M_temp = cv2.moments(mask_yellow_)
                    if M_temp['m00'] > 0:
                        moments_list.append(M_temp)
                    mask_yellow_[0:h, 0:w] = 1
            M_temp_ = moments_list.pop()
            cx_yellow = int(M_temp_['m10'] / M_temp_['m00'])
            err = cx_yellow - w / 2
            self.drive_key.key = "not_find"
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 212.5  # 400: 0.1, 300: 0.15, 250, 0.2,0.25:225 212.5 0.3
            self.drive_key.twist = self.twist
            self.drive_pub.publish(self.drive_key)
        cv2.waitKey(3)
        cv2.imshow("window", mask_yellow)

rospy.init_node('follower_yellow_2')
follower = Follower()
rospy.spin()