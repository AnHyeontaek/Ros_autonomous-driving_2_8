#!/usr/bin/env python
# BEGIN ALL
# follow_bar.py
# 차단바 인식 후 주행관리 소스
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from deu_car.msg import drive_key
import time
class Follower_bar:
   def __init__(self):
       self.bridge = cv_bridge.CvBridge()
       self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
       # 터틀봇에 장착된 카메라의 영상을 구독한다.
       self.drive_pub = rospy.Publisher('bar', drive_key, queue_size=1)
       self.drive_key = drive_key()
       self.drive_key.twist = Twist()
   def image_callback(self, msg):
   # Image메시지를 처리하기 위한 콜백함수
       bar_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       # Image메시지를 OpenCV영상 형식으로 변환한다.
       hsv = cv2.cvtColor(bar_image, cv2.COLOR_BGR2HSV)
       # Image영상을 활용해 2진 영상을 제작한다.
       lower_red = numpy.array([-10, 100, 100])
       upper_red = numpy.array([10, 255, 255])
       # 차단바의 빨간색을 인식하기 위해 hsv형태의 빨간색 범위를 지정한다.
       mask_red = cv2.inRange(hsv, lower_red, upper_red)
       h, w, d = bar_image.shape
       search_h = h/2
       search_w = w/3
       mask_red[search_h:h, 0:w] = 0
       mask_red[0:h, 0:search_w] = 0
		# 32 ~ 37은 Image에서 인식할 부분을 처리하는 부분이다.
       M = cv2.moments(mask_red)
       if M['m00'] > 0:
       # 빨간색을 인식하면 차단바가 내려와있다는 의미이므로 터틀봇의 이동을 중지시킨다.
           self.drive_key.key = "bar"
           self.drive_pub.publish(self.drive_key)
       else :
       # 빨간색이 인식되지 않으면 차단바가 올라가있다는 의미이므로 터틀봇을 이동시킨다.
           self.drive_key.key = "go"
           self.drive_pub.publish(self.drive_key)
       cv2.waitKey(3)
rospy.init_node('detect')
detect = Follower_bar()
rospy.spin()
