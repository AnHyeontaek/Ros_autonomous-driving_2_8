#!/usr/bin/env python
#follow_last_course.py
#마지막 코스 구간 주행
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
       self.drive_key = drive_key() #twist랑 string drive_key를 갖는 커스텀 메세지
       self.bridge = cv_bridge.CvBridge()
       self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback) # 터틀봇에 장착된 카메라의 영상을 구독한다.
       self.drive_pub = rospy.Publisher('drive_last_course', drive_key, queue_size=1) #drive_key라는 메세지 타입을 갖는 drive_last_course 토픽 발행
       self.twist = Twist()
       self.drive_route_count = 0
       self.speed = 0.5
       self.angular = 200
   def speed_cb(self, msg): #속도를 처리하는 함수
       if msg.data == "low": #drive.py에서 low라는 메세지가 들어왔을 경우
           self.speed = 0.3 #선속도를 0.3으로 지정한다.
           self.angular = 212.5 # 각속도는 212.5으로 지정한다.
       elif msg.data == "high": #drive.py에서 high라는 메세지가 들어왔을 경우
           self.speed = 0.7 #선속도를 0.7으로 지정한다.
           self.angular = 200 #각속도는 200으로 지정한다.
       elif msg.data == "middle": #drive.py에서 middle 이라는 메세지가 들어왔을 경우
           self.speed = 0.5 #선속도를 0.5으로 지정한다.
           self.angular = 200  #각속도는 200으로 지정한다.
   def image_callback(self, msg): #마지막 코스를 인식하여 주행하는 함수
       self.drive_key.key = ""
       image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Image메시지를 OpenCV영상 형식으로 변환한다.
       hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Image영상을 활용해 2진 영상을 제작한다.
      
       lower = numpy.array([0,0,128])
       upper = numpy.array([100,255,255])
		 # 노란 차선과 흰색 차선을 인식하기 위해 hsv형태의 흰색, 노란색 범위를 합쳐서 지정한다.
       mask = cv2.inRange(hsv,lower,upper)
   
       h, w, d = image.shape
       search_top = 3*h/4
       search_bot = search_top + 20
       mask[0:search_top, 0:w] = 0
       mask[search_bot:h, 0:w] = 0
       mask[0:h, w/2:w] = 0
		# 50 ~ 58은 Image에서 인식할 부분을 처리하는 부분이다.
       M = cv2.moments(mask)
       if M['m00'] > 0:
		# 흰색과 노란색을 인식하면 마지막 구간이라는 의미이므로 터틀봇의 선속도와 각속도를 지정하여 주행한다.
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
