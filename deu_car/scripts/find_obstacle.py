#!/usr/bin/env python
#find_obstacle.py
#장애물 감지 후 주행 담당
import math
import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from deu_car.msg import drive_key
# BEGIN MEASUREMENT
class DetectObtacle:
   def __init__(self):
       self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
       # Lidar 센서의 값을 사용하기 위해 scan토픽의 LaserScan메시지를 구독한다.
       self.find_obstacle_pub = rospy.Publisher('find_obstacle', drive_key, queue_size=1)
       self.drive_key = drive_key()
       self.drive_key.key = "obstacle"
       self.twist = Twist()
   def scan_callback(self, msg):
       range_center = msg.ranges[len(msg.ranges) / 2]
       # LaserScan의 값들 중 평균값을 터틀봇에 부착된 센서가 정중앙을 바라보는 값으로 한다.
       range_right = max(msg.ranges)
       # LaserScan의 값들 중 최댓값을 터틀봇에 부착된 센서가 오른쪽을 바라보는 값으로 한다.
       if range_center > 2.0 or range_right > 1.5 or \
               ((math.isnan(range_center)) and math.isnan(range_right)):
       # 중간값이 2.0초과이고 오른쪽값이 1.5초과이면 터틀봇이 주행한다.
       # 터틀봇이 장애물과 충돌 방지를 위해 정지한 후 장애물이 터틀봇과 멀어지기 시작하면 다시 주행한다.
           self.twist.linear.x = 1.0
           self.drive_key.key = "no_obstacle"
       elif range_center < 2.0 or range_right < 1.5:
       # 중간값이 2.0미만이고 오른쪽값이 1.5미만이면 정지한다.
           self.twist.linear.x = 0.0
           self.drive_key.key = "obstacle"
       self.drive_key.twist = self.twist
       self.find_obstacle_pub.publish(self.drive_key)
rospy.init_node('DetectObtacle')
wander = DetectObtacle()
rospy.spin()
