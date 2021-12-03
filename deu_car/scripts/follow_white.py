#!/usr/bin/env python
# BEGIN ALL
# follow_white.py
#차선 인식 후 주행 담당
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String
from deu_car.msg import drive_key
import threading
class Follow_white:
   def __init__(self):
       self.count = 1
       self.bridge = cv_bridge.CvBridge()
       self.bridge = cv_bridge.CvBridge()
       self.drive_pub = rospy.Publisher('drive', drive_key, queue_size=1) #drive_key라는 메세지 타입을 갖는 drive 토픽 발행
       self.drive_pub_2 = rospy.Publisher('drive_2', drive_key, queue_size=1) #drive_key라는 메세지 타입을 갖는 drive_2 토픽 발행
       self.park_pub = rospy.Publisher('parking', drive_key, queue_size=1) #drive_key라는 메세지 타입을 갖는 parking 토픽 발행
       self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback) # 터틀봇에 장착된 카메라의 영상을 구독한다.
       self.twist = Twist() #twist 객체 생성
       self.drive_key = drive_key() #twist랑 string drive_key를 갖는 커스텀 메세지
       self.stop_count = 0 #정지선 카운트-차단바인식 후 대기 동안의 정지선 읽는 것을 방지하기 위한 정지선 카운트 변수, default값으로 초기화되어 있다. 
       self.drive_key.key = "stop" #twist를 보낸 이유 지정-정지선
  
   def image_callback(self, msg): #정지선을 인식하여 처리하는 함수
       image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
  # Image메시지를 OpenCV영상 형식으로 변환한다
       hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
# Image영상을 활용해 2진 영상을 제작한다.
       lower_white = numpy.array([0, 0, 200])
       upper_white = numpy.array([0, 0, 255])
		# 정지선의 흰색을 인식하기 위해 hsv형태의 흰색 범위를 지정한다.
       mask_white = cv2.inRange(hsv, lower_white, upper_white)
       h, w, d = image.shape
       search_top = 4*h/5
       search_bot = search_top + 20
       search_left = w/2 - 10
       search_right = search_left + 20
       mask_white[0:search_top, 0:w] = 0
       mask_white[search_bot:h, 0:w] = 0
       mask_white[0:h, 0:search_left] = 0
       mask_white[0:h, search_right:w] = 0
		# 37 ~ 46은 Image에서 인식할 부분을 처리하는 부분이다.
       M = cv2.moments(mask_white)
       if M['m00'] > 0:
		# 흰색을 인식하면 정지선을 인식했다는 의미이므로 터틀봇의 이동을 중지시킨다.
           self.twist.linear.x = 0
           if self.stop_count == 0:
 #정지선 카운트가 0일 경우 처음 읽은 정지선으로 인식함
               self.drive_key.key = "stop"
           else: 
# 흰색이 인식되지 않으면 정지선이 인식되지 않았다는 의미이므로 터틀봇을 이동시킨다.
               self.drive_key.key = "nonstop"
           self.drive_key.twist = self.twist
           self.drive_pub.publish(self.drive_key)
           self.drive_pub_2.publish(self.drive_key)
           self.park_pub.publish(self.drive_key)
           self.stop_count += 1 #정지선을 읽으면 정지선 카운트를 1 증가시킴
       else:
           self.stop_count = 0 #정지선에 벗어나면 초기화
       cv2.waitKey(3)
rospy.init_node('follower_white')
follower = Follow_white()
rospy.spin()
