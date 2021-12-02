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
        if count == "1":
            self.start_route = 1  # 1 left or 2 right
            self.drive_route = 1
        elif count == "2":
            self.start_route = 2  # 1 left or 2 right
            self.drive_route = 2
        self.on_bar = 0
        self.cross = 0 # stop_line count
        self.drive_state = "run" # can self run state
        self.can_stop = True # can stop
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.drive_tr = ""
        self.drive_sub = rospy.Subscriber('drive', drive_key, self.drive_cb)
        self.drive_sub_2 = rospy.Subscriber('drive_2', drive_key, self.drive_cb_2)
        self.parking_sub = rospy.Subscriber('parking', drive_key, self.parking_cb)
        self.bar_sub = rospy.Subscriber('bar', drive_key, self.follow_bar)
        self.drive_last_course_sub = rospy.Subscriber('drive_last_course', drive_key, self.drive_last_course_cb)
        self.find_obstacle_sub = rospy.Subscriber("find_obstacle", drive_key, self.find_obstacle_cb)
        self.sign_sub = rospy.Subscriber('red_sign', drive_key, self.sign_cb)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.drive_route_pub = rospy.Publisher('select_route', String, queue_size=1)
        self.speed_pub = rospy.Publisher('speed', String, queue_size=1)
        self.parking_pub = rospy.Publisher('park', String, queue_size=1)
        self.g_last_send_time = rospy.Time.now()
        self.ramp = 0.5
        self.last_twist = Twist()
        self.twist = Twist()
        self.end_sign = False

    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate):
        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        if error < step:  # we can get there within this timestep. we're done.
            return v_target
        else:
            return v_prev + sign * step  # take a step towards the target

    def ramped_twist(self, prev, target, t_prev, t_now, ramp):
        tw = Twist()
        tw.linear.x = self.ramped_vel(prev.linear.x, target.linear.x, t_prev,
                                 t_now, ramp)
        tw.angular.z = target.angular.z
        return tw

    def send_twist(self, target):
        t_now = rospy.Time.now()
        self.last_twist = self.ramped_twist(self.last_twist, target,
                                    self.g_last_send_time, t_now, self.ramp)
        self.g_last_send_time = t_now
        self.cmd_vel_pub.publish(self.last_twist)

    def wait(self, stop_time):
        time.sleep(stop_time)
        self.can_stop = True
        if stop_time == 20:
            self.end_sign = True

    def accel(self):
        time.sleep(5)
        self.speed_pub.publish("middle")

    def cross_run_1(self, count):
        time.sleep(count)
        self.drive_state = "run"

    def cross_run_2(self):
        time.sleep(10)
        self.drive_state = "run"
        time.sleep(3)
        self.drive_route_pub.publish("parking")

    def drive_cb(self, msg):
        if self.drive_route == 1:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.on_bar == 1:
                self.twist.linear.x = 0
                self.send_twist(self.twist)
                return
            if self.drive_tr == "run" and self.drive_state == "run":
                self.send_twist(self.twist)
            elif self.drive_tr == "stop" and self.can_stop:
                self.cross += 1
                self.drive_state = "stop"
                self.send_twist(self.twist)
                time.sleep(3)
                stop_time = 3
                self.drive_state = "run"
                if self.cross == 1:
                    self.bar_sub.unregister()
                    self.speed_pub.publish("middle")
                elif self.cross == 2:
                    self.speed_pub.publish("low")
                elif self.cross == 4:
                    self.speed_pub.publish("low")
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run_1, args=[10])
                    p.start()
                    stop_time = 8
                elif self.cross == 5:
                    self.speed_pub.publish("low")
                    if self.start_route == 1:
                        self.drive_route = 1
                        self.drive_route_pub.publish("course_turning")
                        self.cross += 1
                        stop_time = 30
                    elif self.start_route == 2:
                        self.drive_route = 2
                        self.drive_route_pub.publish("course_turning")
                        stop_time = 30
                elif self.cross == 7:
                    self.speed_pub.publish("low")
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run_2)
                    p.start()
                    stop_time = 15
                elif self.cross == 8:
                    self.drive_route = 3
                elif self.cross == 11:
                    self.drive_route = 5
                else:
                    self.speed_pub.publish("middle")
                self.can_stop = False
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_tr == "parking_start":
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
                    self.cross += 1
                    self.send_twist(self.twist)
                    time.sleep(3)
                    self.drive_state = "run"
                    stop_time = 3
                    if self.cross == 1:
                        self.bar_sub.unregister()
                        self.speed_pub.publish("middle")
                    elif self.cross == 2:
                        self.speed_pub.publish("low")
                    elif self.cross == 8:
                        self.drive_route = 3
                        self.parking_pub.publish("on")
                        stop_time = 10
                    else:
                        self.speed_pub.publish("middle")
                    self.can_stop = False
                    t = threading.Thread(target=self.wait, args=[stop_time])
                    t.start()

    def parking_cb(self, msg):
        if self.drive_route == 3:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run":
                self.send_twist(self.twist)
            elif self.drive_tr == "stop" and self.can_stop:
                self.cross += 1
                self.drive_state = "stop"
                self.send_twist(self.twist)
                time.sleep(5)
                stop_time = 3
                if self.cross == 9:
                    stop_time = 30
                    self.drive_route = 4
                    s = threading.Thread(target=self.turn)
                    s.start()
                self.drive_state = "run"
                self.can_stop = False
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_tr == "parking_end":
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