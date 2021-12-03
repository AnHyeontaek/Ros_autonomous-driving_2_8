#!/usr/bin/env python
# change_course.py
# 방향 전환 코스에서의 차선 인식 ,노란 정지선 인식 후 주행 담당
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from deu_car.msg import drive_key
class Change_course:
    def __init__(self):
        self.count = 1
        self.bridge = cv_bridge.CvBridge()
        self.change_course_pub = rospy.Publisher('change_course', drive_key, queue_size=1) #drive_key라는 메세지 타입을 갖는 change_course 토픽 발행
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback) # 터틀봇에 장착된 카메라의 영상을 구독한다.
        self.switch_sub = rospy.Subscriber('change', String, self.switch) # 방향 전환 코스에 들어왔음을 알리기 위한 토픽 구독
        self.twist = Twist()  #twist 객체 생성
        self.drive_key = drive_key() #twist랑 string drive_key를 갖는 커스텀 메세지
        self.drive_key.key = "run"  #twist를 보낸 이유 지정-방향 전환 코스 실행
        self.change_course = False #방향 전환 코스의 실행 상태 유무 변수
        self.change_course_yellow = False  #노란 정지선의 인식 유무 변수
    def switch(self, msg): #방향 전환 코스의 실행 상태 확인하는 함수
        self.change_course = True #방향 전환 코스 상태 변수를 실행으로 바꾼다
    def image_callback(self, msg): #방향 전환 코스를 처리하는 함수
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Image메시지를 OpenCV영상 형식으로 변환한다
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Image영상을 활용해 2진 영상을 제작한다.
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
		# 방향전환 코스의 차선의 흰색을 인식하기 위해 hsv형태의 흰색 범위를 지정한다.
        lower_yellow = numpy.array([0, 128, 128])
        upper_yellow = numpy.array([100, 255, 255])
		# 방향전환 코스의 정지선의 노란색을 인식하기 위해 hsv형태의 노란색 범위를 지정한다.
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 4 * h / 5 + 20
        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0
        mask_white[0:h, w/2:w] = 0
        search_top_yellow = 4 * h / 5
        search_bot_yellow = search_top_yellow + 20
        search_left_yellow = w / 2 - 10
        search_right_yellow = search_left_yellow + 20
        mask_yellow[0:search_top_yellow, 0:w] = 0
        mask_yellow[search_bot_yellow:h, 0:w] = 0
        mask_yellow[0:h, 0:search_left_yellow] = 0
        mask_yellow[0:h, search_right_yellow:w] = 0
		# 39 ~ 57은 Image에서 인식할 부분을 처리하는 부분이다.
        M = cv2.moments(mask_white)
        M_yellow = cv2.moments(mask_yellow)
        if self.change_course and M_yellow['m00'] > 0:
			# 노란색을 인식하면 방향 전환 코스의 정지선을 인식했다는 의미이므로 방향전환 코스를 종료한다.
            self.change_course = False #방향 전환 코스 상태 변수를 종료으로 바꾼다
            self.twist = Twist() #twist 객체 생성
            self.drive_key.twist = self.twist
            self.drive_key.key = "change_course_end"  #twist를 보낸 이유 지정-방향 전환 코스 종료 실행
            self.change_course_pub.publish(self.drive_key) #drive_key라는 메세지 타입을 갖는 change_course 토픽 발행
            return #방향전환 코스 실행을 종료한다. 
        if self.change_course and M['m00'] > 0: 
			# 흰색을 인식하면 방향 전환 코스의 차선을 인식했다는 의미이므로 방향전환 코스 터틀봇을 이동시킨다. 
            cx = int(M['m10']/M['m00'])
            err = (cx + 200) - w / 2
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 212.5  # 400: 0.1, 300: 0.15, 250, 0.2
            self.drive_key.twist = self.twist
            self.change_course_pub.publish(self.drive_key)
        cv2.waitKey(3)
rospy.init_node('follower_park')
follower = Change_course()
rospy.spin()
