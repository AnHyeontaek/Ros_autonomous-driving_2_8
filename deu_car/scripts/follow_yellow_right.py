#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from deu_car.msg import drive_key
from std_msgs.msg import String


class Follow_yellow_right:
    def __init__(self):
        self.this_pub_state = 1 #  색추적 범위 변경 시 사용될 변수 1일시 가능
        self.drive_key = drive_key() # 클래스 내부에서 사용할 커스텀 메시지 String 변수인 key 와 Twist 변수인 twist를 가지고 있다.
        self.only_right = True # True 일시 우측만 색추적 한다.
        self.bridge = cv_bridge.CvBridge()
        self.speed_sub = rospy.Subscriber('speed', String, self.speed_cb) # 속도 변환 시 발행될 토픽을 받을 구독
        self.select_route_sub = rospy.Subscriber('select_route', String, self.select_route_cb) # 주차, 곡선코스 등 코스 변경 시 발행될 토픽을 구독
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.drive_pub = rospy.Publisher('drive_2', drive_key, queue_size=1) # 클래스에서 모인 정보를 drive2 라는 토픽으로 발행한다. 발행된 정보는 drive.py에서 구독해 사용한다.
        self.twist = Twist() # twist 초기화
        self.drive_route_count = 0 # 색 추적 불가 시 색 추적 범위 변경을 위해 사용될 변수 5 이상일시 가능
        self.speed = 0.5 # 기본 속도 0.5
        self.angular = 212.5 # 기본 회전값 200
        self.change_course = False # 주차 시작 시 True 가 된다.

    def speed_cb(self, msg):  # 속도 변경 토픽 구독 콜백 함수
        if msg.data == "low":
            self.speed = 0.3
            self.angular = 212.5
        elif msg.data == "high":
            self.speed = 0.9
            self.angular = 200
        elif msg.data == "middle":
            self.speed = 0.5
            self.angular = 200

    def select_route_cb(self, msg): # 주차, 곡선코스 등 코스 변경 토픽을 구독할때 불러올 콜백 함수
        if msg.data == "course_turning": # 곡선코스 진입 시
            self.only_right = False # 우측 색추적 제한 해제
            self.this_pub_state = 1 # 색 추적 범위 변경 가능
        elif msg.data == "2": # 주차구간 종료 시
            self.only_right = False
            self.this_pub_state = 1
        elif msg.data == "1": # 주차구간 시작 시
            self.only_right = True # 색 추적을 오직 오른쪽만
        else: # change_course 넘어옴
            self.change_course = True # 주차 중

    def image_callback(self, msg):
        self.drive_key.key = "" # drive_key.key 초기화
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([0, 128, 128]) # 노란색 범위 최소값
        upper_yellow = numpy.array([100, 255, 255]) # 노란색 범위 최대값

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) # 노란색 추적
        mask_yellow_not_find = cv2.inRange(hsv, lower_yellow, upper_yellow) # 범위 내에서 노란색을 못 찾았을 때, 다음 노란색을 찾기 위한 추적 범위
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        mask_yellow[0:search_top, 0:w] = 0
        if not self.change_course: # 주차중이 아닐때
            mask_yellow[search_bot:h, 0:w] = 0 # 아래쪽은 찾지 않음
            # 주차코스 진입 시 오른쪽의 노란색 차선을 보고 들어가는데 
        if self.only_right: # 탐색 범위를 오른쪽으로 제한 할 경우
            mask_yellow[0:h, 0:w/2] = 0
        M = cv2.moments(mask_yellow)

        #list of moments to find next center line if not find center line
        moments_list = list() # 노란색 추적 불가 시 다음 색 추적을 위해 사용할 리스트

        if M['m00'] > 0: # 범위 내에서 노란색을 찾았을 경우
            del moments_list[:] # 리스트 초기화
            # BEGIN CONTROL
            cx_yellow = int(M['m10']/M['m00']) # 노란색의 x축 중앙값
            err = (cx_yellow - 250) - w / 2 # 노란색의 x축 중앙값의 250만큼 왼쪽을 목표로 잡고 간다.
            self.twist.linear.x = self.speed 
            self.twist.angular.z = -float(err) / (self.angular)  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist # 정해진 방향과 속도를 보낸다.
            self.drive_key.key = "run" # run이라는 키값을 보낸다.
            self.drive_pub.publish(self.drive_key)
            # END CONTROL
        else:
            if self.change_course: # 방향전환 코스에 도달했을 시 실행될 코드 ( 주차구간 들어가기 직전에 노란색이 없어져 정지한다. 때문에 조금 직진하기 위해서 만들었다. )
                self.twist.linear.x = 0.5 # 속도는 0.5
                self.drive_key.twist = self.twist # 방향은 가던 방향 그대로
                self.drive_key.key = "run"
                self.drive_pub.publish(self.drive_key)
            elif self.this_pub_state == 1: 
                # follow_yellow_right 를 따라 가다가 선을 못찾으면 다음 선을 찾고 follow_yellow_left 로 바꿔주는데, 그때 못찾았다는 명령어로 "not_find"를 보내준다.
                # follow_yellow_left 로 넘어간 후에도 not_find를 계속 보내서 그걸 방지하기 위한 조건문이다.
                
                self.drive_route_count += 1
                # 곡선코스 진입 시 왼쪽 노란선을 따라가다가 왼쪽에 있는 노란선을 오른쪽에 놓고 따라가게 바꿔야 하는데 그 순간 오른쪽엔 노란선이 없어 not_find를 보낸다.
                # 따라서 굴절코스 진입과 함께 추적 범위를 왼쪽도 읽게 바꿔주는데
                # 추적범위를 바꿔도 그 순간에 이 콜백 함수가 실행되면서 노란선을 못 찾아 다음 노란선을 찾으며 follow_yellow_left 로 바뀐다.
                # 따라서 추적 범위가 바뀌기 전까진 이 아래의 코드가 실행되면 안된다. 때문에 만든 변수다.
                # 이 콜백 함수가 5번 실행되기 전까진 아래의 코드로 진입을 못한다.
                
                if self.drive_route_count == 5:
                    for i in range(7): 
                        for j in range(5): # 화면의 왼쪽 위부터 가로 1/10, 세로 1/10 크기의 칸을 나눠 가로로 5칸 세로로 7칸 동안 노란색을 찾아 마지막 노란색을 목표로 나아간다.
                            search_top_ = (i*h)/10
                            search_bot_ = ((i+1)*h)/10
                            mask_yellow_not_find[0:search_top_, 0:w] = 0
                            mask_yellow_not_find[search_bot_:h, 0:w] = 0
                            mask_yellow_not_find[0:h, 0:j*w/10] = 0
                            mask_yellow_not_find[0:h, (j+1)*w/10:w] = 0
                            M_temp = cv2.moments(mask_yellow_not_find)
                            if M_temp['m00'] > 0:
                                moments_list.append(M_temp) # 찾는 노란색은 모두 리스트에 저장
                            mask_yellow_not_find[0:h, 0:w] = 1 # 한칸 탐색이 끝나면 탐색범위 초기화.
                    M_temp_last = moments_list.pop() # 가장 마지막에 들어온 칸을 추출
                    cx_yellow = int(M_temp_last['m10'] / M_temp_last['m00']) # 목표로 삼고 전진
                    err = (cx_yellow) - w / 2
                    self.drive_key.key = "not_find"
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = -float(err) / 200  # 400: 0.1, 300: 0.15, 250, 0.2,0.25:225 212.5 0.3
                    self.drive_key.twist = self.twist
                    self.drive_pub.publish(self.drive_key)
                    self.drive_route_count = 0 # 초기화
                    self.this_pub_state = 0 # follow_yellow_left로 넘어갔으므로 이 elif 문으로 못들어오게 한다.

        cv2.waitKey(3)

rospy.init_node('follower_yellow_2')
follower = Follow_yellow_right()
rospy.spin()
