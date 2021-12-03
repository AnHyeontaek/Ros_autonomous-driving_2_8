#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from std_msgs.msg import String
from deu_car.msg import drive_key
import threading
import sys
import math

class Drive:
    def __init__(self, count):
        if count == "1": # 1 번 코스
            self.start_route = 1
            self.drive_route = 1
        elif count == "2": # 2번 코스
            self.start_route = 2
            self.drive_route = 2
        self.on_bar = 0 # 차단바 탐지 시 1로 변경
        self.stop_line_count = 0 # 정지선 만난 횟수.
                                 # 시작 정지선은 카운트에 들어가지 않는다. ( 일정 시간 후 노드 실행 )
        self.drive_state = "run" # 주행 상태
        self.can_stop = True # 정지 가능 여부. 정지선을 연속으로 읽어대서 만든 변수다. 
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.drive_tr = "" # 주행 트리거. drive_key.key 로 넘어오는 값을 저장할 공간이다.
        self.drive_sub = rospy.Subscriber('drive', drive_key, self.drive_cb) # follow_yellow_left.py 에서 발행될 토픽을 구독. 주행 전반에 영향을 끼친다.
        self.drive_sub_2 = rospy.Subscriber('drive_2', drive_key, self.drive_cb_2) # follow_yellow_right.py 에서 발행될 토픽을 구독. 주행 전반에 영향을 끼친다.
        self.change_course_sub = rospy.Subscriber('change_course', drive_key, self.change_course_cb) # change_course.py 에서 발행될 토픽을 구독. 방향전환코스 전용
        self.bar_sub = rospy.Subscriber('bar', drive_key, self.follow_bar) # follow_bar.py 에서 발행될 토픽을 구독. 차단바 전용
        self.drive_last_course_sub = rospy.Subscriber('drive_last_course', drive_key, self.drive_last_course_cb) # follow_last_cours.py 에서 발행될 토픽을 구독
                                                                                                                 # 마지막 교차로 진입부터 도착까지를 담당.
        self.find_obstacle_sub = rospy.Subscriber("find_obstacle", drive_key, self.find_obstacle_cb) # find_obstacle.py 에서 발행될 토픽을 구독
                                                                                                     # 장애물 인식 전용
        self.sign_sub = rospy.Subscriber('red_sign', drive_key, self.sign_cb) # detect_sign.py 에서 발행될 토픽을 구독. 표지판 인식 전용
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback) # 카메라 사용을 위한 구독
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1) # 로봇에 움직임을 명령할 토픽
        self.drive_route_pub = rospy.Publisher('select_route', String, queue_size=1) # 현재 루트 변경 시 루트 변경을 여러 클래스에 알린다.
        self.speed_pub = rospy.Publisher('speed', String, queue_size=1) # 속도 변경 시 속도 변경을 여러 클래스에 알린다.
        self.change_course_pub = rospy.Publisher('change', String, queue_size=1) # 방향전환 코스 진입을 여러 클래스에 알린다.
        self.g_last_send_time = rospy.Time.now() # 속도 급변화를 막기 위한 소스에 필요함.
        self.ramp = 0.5 # 속도 급변화를 막기 위한 소스에 필요함.
        self.last_twist = Twist() # 속도 급변화를 막기 위한 소스에 필요함.
        self.twist = Twist() # 트위스트 변수 초기화
        self.end_sign = False # 마지막 표지판인지 확인하는 여부

    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate): # 급 속도변환을 막기위한 함수. 목표 속도에 도달할 때까지 소량의 속도를 가감한다.
        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        if error < step:  # we can get there within this timestep. we're done.
            return v_target
        else:
            return v_prev + sign * step  # take a step towards the target

    def ramped_twist(self, prev, target, t_prev, t_now, ramp): # 급 속도변환을 막기위한 함수. ramped_vel 함수의 결과로 나온 twist변수를 반환한다.
        tw = Twist()
        tw.linear.x = self.ramped_vel(prev.linear.x, target.linear.x, t_prev,
                                 t_now, ramp)
        tw.angular.z = target.angular.z # 방향전환은 천천히 했더니 로봇이 연석밟는 일이 너무 많아서 취소함.
        return tw

    def send_twist(self, target): # 그렇게 만들어진 twist 변수를 로봇에 전달한다.
        t_now = rospy.Time.now()
        self.last_twist = self.ramped_twist(self.last_twist, target,
                                    self.g_last_send_time, t_now, self.ramp)
        self.g_last_send_time = t_now
        self.cmd_vel_pub.publish(self.last_twist)

    def drive_cb(self, msg): # follow_yellow_left.py 에서 넘어오는 정보를 담당하는 콜백함수.
        if self.drive_route == 1: # 기본적으로 drive_route가 1일때 실행.
            self.drive_tr = msg.key # 넘어온 메시지를 저장한다.
            self.twist = msg.twist
            if self.on_bar == 1: # 차단바 탐지 중일시 정지
                self.twist.linear.x = 0
                self.send_twist(self.twist)
                return
            if self.drive_tr == "run" and self.drive_state == "run": # 넘어온 키값과 drive_state 가 run 이면 넘어온 정보대로 주행한다.
                self.send_twist(self.twist)
            elif self.drive_tr == "stop" and self.can_stop: # follow_white.py 에서 정지 신호가 날아오고 정지 가능한 상태일 경우
                self.stop_line_count += 1 # 정지선 횟수 증가
                self.drive_state = "stop" # drive_state를 "stop"으로 바꿔 주행을 막는다
                self.send_twist(self.twist) # 속도 0이 될때까지 함수 실행
                time.sleep(3) # 3초 정지
                stop_time = 3 # 정지 후에 3초간 정지못하게 할 예정 그동안 정지선은 지나친다. 이걸 안해줬더니 바로 정지선 읽고 멈춤.
                self.drive_state = "run" # drive_state 변경으로 다시 주행 시작.
                if self.stop_line_count == 1: # 첫번째 정지선을 만났을 경우
                    self.bar_sub.unregister() # 차단바 토픽 구독 변수 해제 
                    self.speed_pub.publish("middle") # 속도 중간으로 변경.
                elif self.stop_line_count == 2: # 두번째 정지선을 만났을 경우 ( 굴절코스 진입 )
                    self.speed_pub.publish("low") # 속도를 늦춘다.
                elif self.stop_line_count == 4: # 네번째 정지선을 만났을 경우 ( 첫번째 교차로 진입 )
                    self.speed_pub.publish("low") # 속도를 늦춘다.
                    self.drive_state = "run_straight" # drive_state 를 직진으로 바꾼다.
                    p = threading.Thread(target=self.cross_run_1, args=[10]) # 10초간 직진. time.sleep() 함수는 프로그램을 멈추므로 쓰레드로 돌린다.
                    p.start() # 쓰레드 시작.
                    stop_time = 8 # 직진중 8초간 흰 선을 읽지 못하게 한다. 교차로 중간의 흰색 점선을 정지선이라고 읽고 멈추는 것 방지.
                elif self.stop_line_count == 5: # 다섯번째 정지선을 만났을 경우 ( 곡선코스 진입 )
                    self.speed_pub.publish("low") # 속도를 늦춘다.
                    if self.start_route == 1: # 1번 코스
                        self.drive_route = 1 # 노란 선을 왼쪽에 두고 진행
                        self.drive_route_pub.publish("course_turning") # 곡선 코스 진입을 알린다.
                        self.stop_line_count += 1 # 1번 코스는 2번코스에 비해 정지선이 하나 적어 원활한 진행을 위해 수를 맞춰준다.
                        stop_time = 30 # 곡선코스는 굴절률이 높아 흰선이 정지선 읽는 위치에 올 때가 많아 곡선코스가 끝날 때까지 정지선 읽기를 꺼준다.
                    elif self.start_route == 2:  # 2번 코스
                        self.drive_route = 2 # 노란 선을 오쪽에 두고 진행
                        self.drive_route_pub.publish("course_turning") # 곡선 코스 진입을 알린다.
                        stop_time = 30 # 곡선코스는 굴절률이 높아 흰선이 정지선 읽는 위치에 올 때가 많아 곡선코스가 끝날 때까지 정지선 읽기를 꺼준다.
                elif self.stop_line_count == 7: # 7번째 정지선을 만났을 경우 ( 두번째 교차로 진입 )
                    self.speed_pub.publish("low")
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run_2) # 직진 후 방향 전환 코스 진입을 준비해야 하므로 따로 만든 함수를 실행한다.
                    p.start()
                    stop_time = 15 # 직진중 15초간 흰 선을 읽지 못하게 한다. 교차로 중간의 흰색 점선을 정지선이라고 읽고 멈추는 것 방지.
                elif self.stop_line_count == 8: # 8번째 정지선 인식. ( 방향 전환 코스 진입 )
                    self.drive_route = 3 # 방향 전환 코스 실행
                elif self.stop_line_count == 11: # 11번째 정지선 인식. ( 마지막 코스 진입 )
                    self.drive_route = 5 # 마지막 코스 실행
                else: # 이외의 정지선을 만날 경우
                    self.speed_pub.publish("middle") # 속도 중간으로 바꿈.
                self.can_stop = False # 정지 불가
                t = threading.Thread(target=self.wait, args=[stop_time]) # 쓰레드로 stop_time 만큼의 시간을 기다린 후 정지 가능 여부를 가능으로 바꾼다.
                t.start()
            elif self.drive_tr == "change_course_start": # 방향 전환 코스 시작 명령이 내려오면 방향 전환 코스의 시작 정지선을 찾기 위해 노란선 인식 방향을 바꿔준다.
                self.drive_route = 2
                self.drive_route_pub.publish("1")

    def drive_cb_2(self, msg): # 대부분의 follow_yellow_right.py 에서 넘어오는 명령을 처리하는 콜백 함수.
        if self.drive_route == 2: # drive_route 가 2일때 이 함수를 실행한다.
            self.drive_tr = msg.key # 넘어온 drive_key 저장
            self.twist = msg.twist
            if self.drive_tr == "not_find": # not_find 명령이 넘어올 경우
                self.drive_route = 1 # 왼쪽 노란선을 따라 주행하게 바꿔준다. ( 기본적으로 자동차는 우측통행이기 때문에 )
            else: 
                if self.on_bar == 1: # 차단바 탐지 중일시 멈춤
                    self.twist.linear.x = 0
                    self.send_twist(self.twist)
                    return
                if self.drive_tr == "run" and self.drive_state == "run": # 넘어온 키값과 drive_state 가 run 이면 넘어온 정보대로 주행한다.
                    self.send_twist(self.twist) 
                elif self.drive_tr == "stop" and self.can_stop: # follow_white.py 에서 정지 신호가 날아오고 정지 가능한 상태일 경우
                    self.drive_state = "stop" # drive_state를 "stop"으로 바꿔 주행을 막는다
                    self.stop_line_count += 1 # 정지선 횟수 증가
                    self.send_twist(self.twist) # 속도 0이 될때까지 함수 실행
                    time.sleep(3) # 3초 정지  
                    self.drive_state = "run" # drive_state 변경으로 다시 주행 시작.
                    stop_time = 3 # 정지 후에 3초간 정지못하게 할 예정 그동안 정지선은 지나친다. 이걸 안해줬더니 바로 정지선 읽고 멈춤.
                    if self.stop_line_count == 1: # 첫번째 정지선을 만났을 경우
                        self.bar_sub.unregister() # 차단바 토픽 구독 변수 해제
                        self.speed_pub.publish("middle") # 속도 중간으로 변경.
                    elif self.stop_line_count == 2: # 두번째 정지선을 만났을 경우 ( 굴절코스 진입 )
                        self.speed_pub.publish("low") # 속도를 늦춘다.
                    elif self.stop_line_count == 8:# 8번째 정지선 인식. ( 방향 전환 코스 진입 )
                        self.drive_route = 3 # 방향 전환 코스 실행
                        self.change_course_pub.publish("on") # 방향 전환 코스 실행
                        stop_time = 10 # 10초간 정지선 인식 막기 ( 방향 전환 코스에 흰선이 많아서 혹시 몰라서 )
                    else:
                        self.speed_pub.publish("middle") # 
                    self.can_stop = False # 정지 불가능
                    t = threading.Thread(target=self.wait, args=[stop_time]) #쓰레드로 stop_time 시간 후 정지 가능으로 바꾼다.
                    t.start()

    def change_course_cb(self, msg): # 방향 전환 코스에서 실행되는 콜백 함수
        if self.drive_route == 3: # 방향 전환 코스일시
            self.drive_tr = msg.key # 넘어온 drive_key 변수 저장
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run": # 문제가 없으면 그대로 주행
                self.send_twist(self.twist)
            elif self.drive_tr == "stop" and self.can_stop: # 정지선( 정해진 범위 안에 흰색이 들어왔을 때 / 방향 전환 구간(T자)에 들어왔을때)
                self.stop_line_count += 1 # 정지선 횟수 증가
                stop_time = 30 # 30초간 정지선 인식 불가 
                self.drive_route = 4 # 일정시간 회전한다.
                s = threading.Thread(target=self.turn)
                s.start()
                self.drive_state = "run" # 회전이 끝나면 주행 시작
                self.can_stop = False # 정지 불가
                t = threading.Thread(target=self.wait, args=[stop_time]) # 30초 후 정지 가능
                t.start()
            elif self.drive_tr == "change_course_end": # 방향전환코스의 마지막에 도달했을 때 ( 노란색 바닥 )
                self.drive_state = "run_straight" # 직진한다.
                p = threading.Thread(target=self.cross_run_1, args=[2]) # 2초 후 직진 종료
                p.start()
                self.drive_route = 1 # 노란선을 왼쪽에 두고 주행 시작.
                self.drive_route_pub.publish("2")
        elif self.drive_route == 4: # 방향 전환 구간에서 회전하는 소스
            self.twist.linear.x = 0 # 속도 없이
            self.twist.angular.z = -0.2 # 회전값만 준다.
            self.send_twist(self.twist)

    def drive_last_course_cb(self, msg): # follow_last_course.py 에서 명령이 올때마다 실행되는 콜백 함수
        if self.drive_route == 5: # 마지막 코스 시작
            self.drive_tr = msg.key # 넘어온 drive_key 저장
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run": # 주행한다.
                self.send_twist(self.twist)

    def find_obstacle_cb(self, msg): # 장애물 인식 콜백 함수
        if msg.key == "obstacle": # 장애물을 인식했을 시 key로 넘어온다.
            self.drive_state = "obstacle_stop" # 상태를 장애물 인식으로 인한 정지로 바꾼다.
            self.speed_pub.publish("high") # 장애물 빨리 넘어가려고 속도를 높여준다.
            t = threading.Thread(target=self.accel) # 5초간 악셀
            t.start()
            self.twist = Twist() 
            self.send_twist(self.twist) # twist 전달
        elif msg.key == "no_obstacle" and self.drive_state == "obstacle_stop": # 장애물 인식 후 장애물이 없어졌을 시
            self.drive_state = "run" # 다시 출발한다.

    def sign_cb(self, msg): # 표지판 인식 콜백 함수
        if self.can_stop: # 정지 가능한 상태일 경우 ( 표지판 인식 후 정지 불가능이 켜져서 한번만 읽는다. )
            self.drive_state = "stop" # 정지상태로 변경
            if self.end_sign: # 마지막 표지판일 경우
                self.drive_sub.unregister() # 주행에 필요한 구독 변수를 정지시켜 주행을 멈춘다.
                self.sign_sub.unregister()
                self.drive_sub_2.unregister()
                self.drive_last_course_sub.unregister()
            time.sleep(3) # 3초 정지
            stop_time = 20 # 20초 동안 정지를 막는다. ( 표지판 재 인식을 막기 위함. )
            self.drive_state = "run" # 주행 시작
            self.can_stop = False
            t = threading.Thread(target=self.wait, args=[stop_time]) 
            t.start()
            
    def follow_bar(self, msg): # 차단바인식 콜백 함수
        if msg.key == "bar": # 차단바를 읽었을 시
            self.on_bar = 1 # 변수를 줘 주행을 막는다.
        else: # 차단바르 못 읽었을 시
            self.on_bar = 0 # 변수를 줘 주행한다.

    def image_callback(self, msg): # 이미지 콜백 함수. 카메라가 이미지를 보낼때마다 실행됨. 상시 실행중
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 이미지 변수 선언
        if self.drive_state == "run_straight": # 직진 상태일 때
            self.twist.linear.x = 0.5 # 0.5의 속도로
            self.twist.angular.z = 0 # 방향은 곧게 앞으로
            self.send_twist(self.twist) # twist 변수를 발행한다.
        cv2.imshow("window", image) # 윈도우창 실행
        cv2.waitKey(3) 
            
    # 쓰레드로 돌아가는 함수들. 주로 일정 시간동안 변수값을 바뀌지 않게 하려고 만들었다.
    def wait(self, stop_time): # 정지선을 일정 시간동안 읽지 못하게 하는 함수. 쓰레드로 돌린다.
        time.sleep(stop_time) # 들어온 시간만큼 기다린다.
        self.can_stop = True # 정지 가능 여부를 True로 바꾼다.
        if stop_time == 20: # 20이 들어왔다는건 첫번째 표지판을 지나 왔다는 뜻.
            self.end_sign = True # 다음 표지판이 코스의 마지막 표지판이다.

    def accel(self): # 일정시간 가속을 위해 만들어진 함수
        time.sleep(5) # 5초간 속도 변화없음
        self.speed_pub.publish("middle") # 속도 중간으로 변경

    def cross_run_1(self, count): # 첫번째 교차로를 위한 함수.(직진), 다른 곳에서도 일정시간 직진하는데 쓰인다.
        time.sleep(count) # 일정 시간 기다린다.
        self.drive_state = "run" # drive_state 다시 run으로 변경.

    def cross_run_2(self): # 두번째 교차로를 위한 함수. 두번째 교차로 이후엔 방향전환 코스를 준비해야 해서 따로 만들었다.
        time.sleep(10) # 10초간 직진
        self.drive_state = "run"
        time.sleep(3) # 3초 후에
        self.drive_route_pub.publish("change_course") # 방향 전환 코스 준비 시작.
        
    def turn(self): # 회전 시간을 보장해주는 함수. 쓰레드로 돌아가며 함수가 끝날때까지 회전한다.
        time.sleep(12) # 12초간 회전
        self.drive_route = 3 # 회전을 멈추고 다시 방향전환코스를 진행한다.
    #쓰레드로 돌아가는 함수들 끝
    



if __name__ == "__main__": # 메인함수
    rospy.init_node('drive') # drive라는 노드 생성
    drive = Drive(sys.argv[1]) # 실행 시 입력한 매개변수를 넣어 시작 코스를 결정한다.
    rospy.spin()
