#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, os, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
from pathlib import Path
from deu_car.msg import drive_key


class DetectSign:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        simp_path = '/home/rosuser/practice/catkin_ws/src/deu_car/scripts/red_sign.png'
        abs_path = os.path.abspath(simp_path)
        self.sign_image = cv2.imread(abs_path)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.sign_pub = rospy.Publisher('red_sign', drive_key, queue_size=1)
        self.twist = Twist()
        self.drive_key = drive_key()
        self.drive_key.key = "sign"

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        orb = cv2.ORB_create()

        kp1, des1 = orb.detectAndCompute(image, None)
        kp2, des2 = orb.detectAndCompute(self.sign_image, None)

        matcher = cv2.BFMatcher_create()
        matches = matcher.knnMatch(des1, des2, 2)

        good_matches = []
        for m in matches:
            if m[0].distance / m[1].distance < 0.7:
                good_matches.append(m[0])

        sign_image = cv2.drawMatches(image, kp1, self.sign_image, kp2, good_matches, None)

        if len(matches) >= 430 and len(matches) <= 433 and len(good_matches) >= 33 and len(good_matches) <= 35:
            self.twist.linear.x = 0.0
            self.drive_key.twist = self.twist
            self.sign_pub.publish(self.drive_key)
        cv2.waitKey(3)
        cv2.destroyAllWindows()
rospy.init_node('detect')
detect = DetectSign()
rospy.spin()