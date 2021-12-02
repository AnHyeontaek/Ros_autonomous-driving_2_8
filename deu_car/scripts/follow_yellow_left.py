#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from deu_car.msg import drive_key
from std_msgs.msg import String


class Follow_yellow_left:
    def __init__(self):
        self.drive_key = drive_key() # 클래스 내부에서 사용할 커스텀 메시지 String 변수인 key 와 Twist 변수인 twist를 가지고 있다.
        self.bridge = cv_bridge.CvBridge() 
        self.speed_sub = rospy.Subscriber('speed', String, self.speed_cb) # 속도 변환 시 발행될 토픽을 받을 구독
        self.select_route_sub = rospy.Subscriber('select_route', String, self.select_route_cb) # 주차, 곡선코스 등 코스 변경 시 발행될 토픽을 구독
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback) # 이미지 카메라 사용을 위한 구독
        self.drive_pub = rospy.Publisher('drive', drive_key, queue_size=1) # 클래스에서 모인 정보를 drive 라는 토픽으로 발행한다. 발행된 정보는 drive.py에서 구독해 사용한다.
        self.twist = Twist() # twist 초기화
        self.route = "" # 주차, 곡선코스 등 코스 변경 시 이용될 변수
        self.only_left = False # True 시 화면의 왼쪽에서만 노란색을 찾아낸다.
        self.find_right = 0 # 주차코스 진입을 위해 오른쪽 노란색을 찾아야 할때 값이 1로 주어져 켜진다.
        self.found_right = 0 # 주차코스 진입을 위해 오른쪽 노란색을 찾다가 찾았을 때 값이 1로 주어져 켜진다.
        self.found_right2 = 0 # 노란색을 찾아 읽고 있다가 노란색이 끊기면 값이 1로 주어진다.
        self.parking_start = 0 # 주차 시작을 알리는 변수 주차가 시작될 때 1로 값이 주어진다.
        self.speed = 0.5 # 기본 속도 0.5
        self.angular = 200 # 기본 회전값 200

    def select_route_cb(self, msg): # 주차, 곡선코스 등 코스 변경 토픽을 구독할때 불러올 콜백 함수
        if msg.data == "course_turning": # 곡선코스 진입 시
            self.route = "course_turning" # 루트를 곡선코스로 변경
            self.only_left = False # 색 추적 범위를 왼쪽 오른쪽 모두 찾는다.
        elif msg.data == "1": # 주차구간 시작 시 
            self.only_left = False # 색 추적 범위를 왼쪽 오른쪽 모두 찾는다.
        elif msg.data == "2": # 주차구간 종료 시
            self.only_left = True # 색 추적 범위를 화면 왼쪽에 제한한다.
            self.find_right = 0 # 오른쪽 색 추적 을 종료한다.
            self.parking_start = 0 # 주차 시작 변수를 0으로 초기화한다.
        elif msg.data == "parking": # 주차 준비 코스 진입 시
            self.only_left = True # 색 추적 범위를 화면 왼쪽에 제한한다.
            self.find_right = 1 # 화면 오른쪽의 색 추적을 따로 시작한다.
            self.found_right = 0 
            self.found_right2 = 0


    def speed_cb(self, msg): # 속도 변경 토픽 구독 콜백 함수
        if msg.data == "low": # 메시지값이 "low"일때
            self.speed = 0.3 # 속도 0.3
            self.angular = 212.5 # 회전값 212.5
        elif msg.data == "high": 
            self.speed = 0.9
            self.angular = 200
        elif msg.data == "middle":
            self.speed = 0.5
            self.angular = 200

    def image_callback(self, msg): # 기본적으로 계속 반복 실행되는 콜백함수
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 카메라를 읽어와 변수로 저장
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # 카메라 이용을 위한 hsv 변수 제작
        self.drive_key.key = "run" # 기본적으로 "run" 이라는 키값을 drive.py에 보낸다

        lower_yellow = numpy.array([0, 128, 128]) # 노란색의 범위 최소값
        upper_yellow = numpy.array([100, 255, 255]) # 노란색의 범위 최대값
        lower_white = numpy.array([0, 0, 200]) # 흰색의 범위 최소값
        upper_white = numpy.array([0, 0, 255]) # 흰색의 범위 최대값

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) # 노란색 추적 변수
        mask_yellow_right = cv2.inRange(hsv, lower_yellow, upper_yellow) # 노란색 추적 변수 ( 오른쪽 전용 )
        mask_yellow_not_find = cv2.inRange(hsv, lower_yellow, upper_yellow) # 노란색 추적 불가 시 새 노란색 추적을 위한 변수
        mask_white_right = cv2.inRange(hsv, lower_white, upper_white) # 오른쪽의 흰색 추적 변수

        h, w, d = image.shape 
        search_top = 3*h/4 # 화면 위에서부터 아래로 4/3 만큼 밑의 좌표
        search_bot = search_top+20 # search_top 에서 20만큼 밑의 좌표
        mask_yellow[0:search_top, 0:w] = 0 # 화면 맨 위부터 search_top 의 높이까지의 화면을 색 추적하지 않는다
        mask_yellow[search_bot:h, 0:w] = 0 # 화면 search_bot 의 위치부터 맨 밑까지 색 추적하지 않는다.
        mask_white_right[0:search_top, 0:w] = 0 # 화면 맨 위부터 search_top 의 높이까지의 화면을 색 추적하지 않는다
        mask_white_right[search_bot:h, 0:w] = 0 # 화면 search_bot 의 위치부터 맨 밑까지 색 추적하지 않는다.
        mask_white_right[0:h, 0:w/2] = 0 # 화면 맨 왼쪽부터 중앙까지 색 추적하지 않는다.
        if self.only_left: # 만약 왼쪽만 색 추적 해야하는 경우
            mask_yellow[0:h, w/2:w] = 0 # 화면 중앙부터 맨 오른쪽까지 색 추적하지 않는다.

        mask_yellow_right[0:search_top, 0:w] = 0 
        mask_yellow_right[search_bot:h, 0:w] = 0
        mask_yellow_right[0:h, 0:w/2] = 0 # 화면 맨 왼쪽부터 중앙까지 색 추적하지 않는다.

        M_right = cv2.moments(mask_yellow_right) # 색 추적한 결과를 이용하기 위한 moments 변수

        if self.find_right == 1: # 우측 색추적이 1일경우
            if M_right['m00'] > 0: # 우측에서 색추적에 성공했을 경우
                self.found_right = 1 
                self.found_right2 = 1
            else: # 우측에서 색추적을 못했을 경우
                self.found_right2 = 0
            if self.found_right == 1 and self.found_right2 == 0: # 우측에서 색추적을 하다가 끊겼을 경우
                self.find_right = 0 # 우측 색추적 종료
                self.only_left = 0 # 좌측 색추적 제한 종료
                self.parking_start = 1 # 주차 시작가능

        if self.parking_start == 1 and M_right['m00'] > 0: # 주차가 시작됐으며 우측의 노란색을 색추적 성공했을 경우
            self.drive_key.key = "parking_start" # 주차 시작 키값을 drive.py에 보낸다.
            self.drive_key.twist = Twist() # 보낼 twist 초기화
            self.drive_pub.publish(self.drive_key) # 만들어진 drive_key를 보낸다

        M = cv2.moments(mask_yellow) 
        M_white = cv2.moments(mask_white_right)
        #list of moments to find next center line if not find center line
        moments_list = list() # 색 추적 불가 시 사용할 moments 변수의 리스트

        if M['m00'] > 0: # 노란색 추적 성공 시
            del moments_list[:] # moments_list 초기화
            # BEGIN CONTROL
            cx_yellow = int(M['m10']/M['m00']) # 색 추적한 공간의 x 중앙 값
            err = (cx_yellow + 250) - w / 2 # 중앙값의 250만큼 오른쪽을 목표로 한다.
            self.twist.linear.x = self.speed # 지정된 속도 저장
            self.twist.angular.z = -float(err) / (self.angular)  # 지정된 회전값 저장
            self.drive_key.twist = self.twist # drive_key에 저장한다
            self.drive_key.key = "run" # drive_key.key 값은 run
            self.drive_pub.publish(self.drive_key) # drive_key를 보낸다
            # END CONTROL
        else: # 노란색 추적 불가 시
            if self.route == "course_turning": # 곡선 코스에선 오목한 부분이 있어 다음 길을 못찾는 경우가 있다. 그럴 시 오른쪽의 흰선을 잠시 추적한다.
                if M_white['m00'] > 0:
                    cx_white = int(M_white['m10'] / M_white['m00'])
                    err = (cx_white - 250) - w / 2
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = -float(err) / (self.angular)  # 400: 0.1, 300: 0.15, 250, 0.2
                    self.drive_key.twist = self.twist
                    self.drive_key.key = "run"
                    self.drive_pub.publish(self.drive_key)
                else: # 흰선이 사라지면 곡선 코스가 종료됐다는 뜻이다.
                    self.route = ""
            elif self.parking_start: # 주차 시작이 가능할 경우
                self.twist.linear.x = 0.2 # 잠시 속도를 늦춘다.
                self.drive_key.twist = self.twist
                self.drive_key.key = "run"
                self.drive_pub.publish(self.drive_key)
            else: # 노란선을 범위내에서 추적이 불가능할 경우
                for i in range(7):
                    for j in range(5): # 2중 for 문으로 화면의 왼쪽 위 부터 가로 1/10, 세로 1/10의 크기의 칸을 나눠 화면의 가로로 1/2 세로로 7/10 구간까지 탐색한다. 
                        search_top_ = (i*h)/10
                        search_bot_ = ((i+1)*h)/10
                        mask_yellow_not_find[0:search_top_, 0:w] = 0
                        mask_yellow_not_find[search_bot_:h, 0:w] = 0
                        mask_yellow_not_find[0:h, 0:j*w/10] = 0
                        mask_yellow_not_find[0:h, (j+1)*w/10:w] = 0
                        M_temp = cv2.moments(mask_yellow_not_find)
                        if M_temp['m00'] > 0: # 노란색을 추적 성공했을 시
                            moments_list.append(M_temp) # 추적한 노란색의 위치를 리스트에 저장
                        mask_yellow_not_find[0:h, 0:w] = 1 # 추적 후엔 mask 변수 초기화 (초기화하지 않을 시 다음 추적 구간이 지워져 있어 추적할 수 없다.)
                M_temp_last = moments_list.pop() # 가장 마지막에 들어간 moments 변수를 하나 뽑아내 그것을 추적한다.
                cx_yellow = int(M_temp_last['m10'] / M_temp_last['m00']) 
                err = (cx_yellow) - w / 2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 200  # 400: 0.1, 300: 0.15, 250, 0.2,0.25:225 212.5 0.3
                self.drive_key.twist = self.twist
                self.drive_pub.publish(self.drive_key) # 추적한 방향과 속도를 보낸다.
        cv2.waitKey(3)

rospy.init_node('follower_yellow')
follower = Follow_yellow_left()
rospy.spin()
