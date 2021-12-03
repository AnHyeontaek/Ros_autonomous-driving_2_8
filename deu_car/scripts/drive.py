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

    def drive_cb(self, msg): # follow_yellow_left.py 에서 넘어오는 정보를 담당한다.
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
                elif self.stop_line_count == 4:
                    self.speed_pub.publish("low")
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run_1, args=[10])
                    p.start()
                    stop_time = 8
                elif self.stop_line_count == 5:
                    self.speed_pub.publish("low")
                    if self.start_route == 1:
                        self.drive_route = 1
                        self.drive_route_pub.publish("course_turning")
                        self.stop_line_count += 1
                        stop_time = 30
                    elif self.start_route == 2:
                        self.drive_route = 2
                        self.drive_route_pub.publish("course_turning")
                        stop_time = 30
                elif self.stop_line_count == 7:
                    self.speed_pub.publish("low")
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run_2)
                    p.start()
                    stop_time = 15
                elif self.stop_line_count == 8:
                    self.drive_route = 3
                elif self.stop_line_count == 11:
                    self.drive_route = 5
                else:
                    self.speed_pub.publish("middle")
                self.can_stop = False
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_tr == "change_course_start":
                self.drive_route = 2
                self.drive_route_pub.publish("1")

    def drive_cb_2(self, msg):
        if self.drive_route == 2:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "not_find":
                self.drive_route = 1
            else:
                if self.on_bar == 1:
                    self.twist.linear.x = 0
                    self.send_twist(self.twist)
                    return
                if self.drive_tr == "run" and self.drive_state == "run":
                    self.send_twist(self.twist)
                elif self.drive_tr == "stop" and self.can_stop:
                    self.drive_state = "stop"
                    self.stop_line_count += 1
                    self.send_twist(self.twist)
                    time.sleep(3)
                    self.drive_state = "run"
                    stop_time = 3
                    if self.stop_line_count == 1:
                        self.bar_sub.unregister()
                        self.speed_pub.publish("middle")
                    elif self.stop_line_count == 2:
                        self.speed_pub.publish("low")
                    elif self.stop_line_count == 8:
                        self.drive_route = 3
                        self.change_course_pub.publish("on")
                        stop_time = 10
                    else:
                        self.speed_pub.publish("middle")
                    self.can_stop = False
                    t = threading.Thread(target=self.wait, args=[stop_time])
                    t.start()

    def change_course_cb(self, msg):
        if self.drive_route == 3:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run":
                self.send_twist(self.twist)
            elif self.drive_tr == "stop" and self.can_stop:
                self.stop_line_count += 1
                stop_time = 30
                self.drive_route = 4
                s = threading.Thread(target=self.turn)
                s.start()
                self.drive_state = "run"
                self.can_stop = False
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_tr == "change_course_end":
                self.drive_state = "run_straight"
                p = threading.Thread(target=self.cross_run_1, args=[2])
                p.start()
                self.drive_route = 1
                self.drive_route_pub.publish("2")
        elif self.drive_route == 4:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.2
            self.send_twist(self.twist)

    def drive_last_course_cb(self, msg):
        if self.drive_route == 5:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run":
                self.send_twist(self.twist)

    def find_obstacle_cb(self, msg):
        if msg.key == "obstacle":
            self.drive_state = "obstacle_stop"
            self.speed_pub.publish("high")
            t = threading.Thread(target=self.accel)
            t.start()
            self.twist = Twist()
            self.send_twist(self.twist)
        elif msg.key == "no_obstacle" and self.drive_state == "obstacle_stop":
            self.drive_state = "run"

    def sign_cb(self, msg):
        if self.can_stop:
            self.drive_state = "stop"
            if self.end_sign:
                self.drive_sub.unregister()
                self.sign_sub.unregister()
                self.drive_sub_2.unregister()
                self.drive_last_course_sub.unregister()
            time.sleep(3)
            stop_time = 20
            self.drive_state = "run"
            self.can_stop = False
            t = threading.Thread(target=self.wait, args=[stop_time])
            t.start()

    def turn(self):
        time.sleep(12)
        self.drive_route = 3

    def follow_bar(self, msg):
        if msg.key == "bar":
            self.on_bar = 1
        else:
            self.on_bar = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.drive_state == "run_straight":
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0
            self.send_twist(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def end(self):
        print("program end")

if __name__ == "__main__":
    rospy.init_node('drive')
    drive = Drive(sys.argv[1])
    rospy.spin()
