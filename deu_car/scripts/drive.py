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

class Follower:
    def __init__(self, count):
        if count == "1":
            self.start_route = 1  # 1 left or 2 right
            self.drive_route = 1
        elif count == "2":
            self.start_route = 2  # 1 left or 2 right
            self.drive_route = 2
        self.on_bar = 0
        self.cross = 0 # stop_line count
        self.drive_state = "run" # can self run state
        self.can_stop = "yes" # can stop
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.drive_tr = ""
        self.drive_sub = rospy.Subscriber('drive', drive_key, self.drive_cb)
        self.drive_sub_2 = rospy.Subscriber('drive_2', drive_key, self.drive_cb_2)
        self.parking_sub = rospy.Subscriber('parking', drive_key, self.parking_cb)
        self.bar_sub = rospy.Subscriber('bar', drive_key, self.follow_bar)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.drive_route_pub = rospy.Publisher('select_route', String, queue_size=1)
        self.speed_pub = rospy.Publisher('speed', String, queue_size=1)
        self.twist = Twist()

    def wait(self, stop_time):
        time.sleep(stop_time)
        self.can_stop = "yes"

    def cross_run_1(self):
        time.sleep(8)
        self.drive_state = "run"

    def cross_run_2(self):
        time.sleep(12)
        self.drive_state = "run"
        time.sleep(3)
        self.drive_route_pub.publish("2")


    def drive_cb(self, msg):
        if self.drive_route == 1:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.on_bar == 1:
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
            else:
                if self.drive_tr == "run" and self.drive_state == "run":
                    self.cmd_vel_pub.publish(self.twist)
                elif self.drive_tr == "stop" and self.can_stop == "yes":
                    self.cross += 1
                    self.drive_state = "stop"
                    print("stop line : ", self.cross)
                    self.cmd_vel_pub.publish(self.twist)
                    time.sleep(3)
                    stop_time = 3
                    self.drive_state = "run"
                    if self.cross == 2:
                        self.speed_pub.publish("low")
                    elif self.cross == 4:
                        self.drive_state = "run_straight"
                        p = threading.Thread(target=self.cross_run_1)
                        p.start()
                        stop_time = 8
                    elif self.cross == 5:
                        self.speed_pub.publish("low")
                        if self.start_route == 1:
                            self.drive_route = 1
                            self.drive_route_pub.publish("1")
                            self.cross += 1
                            stop_time = 30
                        elif self.start_route == 2:
                            self.drive_route = 2
                            self.drive_route_pub.publish("2")
                            stop_time = 30
                    elif self.cross == 7:
                        self.drive_state = "run_straight"
                        p = threading.Thread(target=self.cross_run_2)
                        p.start()
                        stop_time = 15
                    else:
                        self.speed_pub.publish("middle")
                    self.can_stop = "no"
                    t = threading.Thread(target=self.wait, args=[stop_time])
                    t.start()
                elif self.drive_state == "run_straight":
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                elif self.drive_tr == "parking_start":
                    print("left")
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
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if self.drive_tr == "run" and self.drive_state == "run":
                        self.cmd_vel_pub.publish(self.twist)
                    elif self.drive_tr == "stop" and self.can_stop == "yes":
                        self.drive_state = "stop"
                        self.cross += 1
                        print("%d th stop line", self.cross)
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(3)
                        self.drive_state = "run"
                        stop_time = 3
                        if self.cross == 2:
                            self.speed_pub.publish("low")
                        elif self.cross == 8:
                            self.drive_route = 3
                        else:
                            self.speed_pub.publish("middle")
                        self.can_stop = "no"
                        t = threading.Thread(target=self.wait, args=[stop_time])
                        t.start()
                    elif self.drive_state == "run_straight":
                        self.twist.linear.x = 0.5
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)

    def parking_cb(self,msg):
        if self.drive_route == 3:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run":
                self.cmd_vel_pub.publish(self.twist)
            elif self.drive_tr == "stop" and self.can_stop == "yes":
                self.cross += 1
                self.drive_state = "stop"
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(3)
                stop_time = 3
                self.drive_state = "run"
                self.can_stop = "no"
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_tr == "parking_end":
                print("parkingend")
                self.drive_route = 1
                self.drive_route_pub = "3"

    def follow_bar(self, msg):
        if msg.key == "bar":
            self.on_bar = 1
        else:
            self.on_bar = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("window", image)
        cv2.waitKey(3)


if __name__ == "__main__":
    rospy.init_node('drive')
    print(sys.argv[1])
    follower = Follower(sys.argv[1])
    rospy.spin()