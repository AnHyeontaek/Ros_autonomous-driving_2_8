#detect_sign.py
#!/usr/bin/env python
#표지판 인식 후 주행 담당
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
       simp_path = '/home/rosuser/practice/catkin_ws/src/deu_car/scripts/red_sign.png' # 대상 영상에 대한 특정 상대 경로
       abs_path = os.path.abspath(simp_path) #특정 상대 경로에 대한 절대 경로 지정
       self.sign_image = cv2.imread(abs_path) # 함수를 이용하여 이미지 파일을 읽음
       self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)  # 터틀봇에 장착된 카메라의 영상을 구독한다.
       self.sign_pub = rospy.Publisher('red_sign', drive_key, queue_size=1) #표지판 인식 메세지 발행
       self.twist = Twist() #Twist 객체
       self.drive_key = drive_key() #twist랑 string drive_key를 갖는 커스텀 메세지
       self.drive_key.key = "sign" #twist를 보낸 이유 지정-표지판
       self.count = 40 # 두개의 표지판을 인식할 때 매칭되는 점의 개수 지정
   def image_callback(self, msg): #표지판을 인식하여 처리하는 함수
       image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       orb = cv2.ORB_create() #특징점을 ORB로 검출합니다.
		#특징점 클래스들은 create 메소드를 사용해 객체를 생성함
		#opencv에 기본 값이 정해져있기에 인자값 없이 기본 설정으로 생성도 가능함
		#특징점 검출 및 기술자 계산
       kp1, des1 = orb.detectAndCompute(image, None)
       kp2, des2 = orb.detectAndCompute(self.sign_image, None)
       matcher = cv2.BFMatcher_create() #추출한 특징점을 매칭하기 위해 BFMatcher를 사용함
       matches = matcher.knnMatch(des1, des2, 2) # knnMatch로 특징점 2개 검출
		#기준 영상과 대상영상의 매칭 값( 매칭되는 점의 개수) 을 선별하는 과정
       good_matches = []
       for m in matches: # matches는 두개의 리스트로 구성
           if m[0].distance / m[1].distance < 0.7: #임계값 지정
			#임계값을 너무 높게 설정해줄 경우 관련이 없는 이미지를 검출할 가능성이 높아짐
			#임계값을 너무 낮게 설정해줄 경우 일치하는 이미지를 검출하지 못할 가능성이 높아짐
               good_matches.append(m[0]) #저장
       if len(good_matches) >= self.count: # 만약 선별해서 저장된 기존 영상과 대상영사의 매칭값이 지정된 표지판을 인식할 때 매칭되는 값보다 클 경우
           self.twist.linear.x = 0.0 # 선속도를 0.0으로 지정하여 정지한다
           self.drive_key.twist = self.twist # twist 메세지 출력
           self.sign_pub.publish(self.drive_key) #twist를 보낸 이유 메세지 발행
           self.count = 50 #한 개의 표지판을 인식할 때 매칭되는 점의 개수 지정
       cv2.waitKey(3)
       cv2.destroyAllWindows()
rospy.init_node('detect')
detect = DetectSign()
rospy.spin()
