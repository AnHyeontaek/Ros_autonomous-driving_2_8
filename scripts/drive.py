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

class Follower:
    def __init__(self):
        self.cross = 0
        self.drive_route = 2
        self.drive_state="run"
        self.can_stop = "yes"
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.drive_tr = ""
        self.drive_sub = rospy.Subscriber('drive', drive_key, self.drive_cb)
        self.drive_sub_2 = rospy.Subscriber('drive_2', drive_key, self.drive_cb_2)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.drive_route_pub = rospy.Publisher('select_route', String, queue_size=1)
        self.twist = Twist()

    def wait(self, stop_time):
        time.sleep(stop_time)
        self.can_stop = "yes"

    def cross_run(self):
        time.sleep(15)
        self.drive_state = "run"

    def drive_cb(self, msg):
        if self.drive_route == 1:
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "run" and self.drive_state == "run":
                self.cmd_vel_pub.publish(self.twist)
            elif self.drive_tr == "stop" and self.can_stop == "yes":
                self.drive_state = "stop"
                self.cross += 1
                print("%d th stop line" %self.cross)
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(3)
                if self.cross == 4:
                    self.drive_state = "run_straight"
                    p = threading.Thread(target=self.cross_run)
                    p.start()
                    stop_time = 15
                elif self.cross == 5:
                    self.drive_route = 2
                    self.drive_route_pub.publish("2")
                    stop_time = 20
                else:
                    self.drive_state = "run2"
                    stop_time = 3
                self.can_stop = "no"
                t = threading.Thread(target=self.wait, args=[stop_time])
                t.start()
            elif self.drive_state == "run_straight":
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

    def drive_cb_2(self, msg):
        if self.drive_route == 2:
            print("yes i can")
            self.drive_tr = msg.key
            self.twist = msg.twist
            if self.drive_tr == "not_find":
                print("not_find")
                self.drive_route = 1
            else:
                if self.drive_tr == "run" and self.drive_state == "run":
                    self.cmd_vel_pub.publish(self.twist)
                elif self.drive_tr == "stop" and self.can_stop == "yes":
                    self.drive_state = "stop"
                    self.cross += 1
                    print("%d th stop line" % self.cross)
                    self.cmd_vel_pub.publish(self.twist)
                    time.sleep(3)
                    if self.cross == 4:
                        self.drive_state = "run_straight"
                        p = threading.Thread(target=self.cross_run)
                        p.start()
                        stop_time = 15
                    elif self.cross == 5:
                        print("where we go?")
                        route = raw_input()
                        if route == "1":
                            self.drive_route = 1
                        elif route == "2":
                            self.drive_route = 2
                            self.drive_route_pub.publish(2)
                        stop_time = 20
                    else:
                        self.drive_state = "run"
                        stop_time = 3
                    self.can_stop = "no"
                    t = threading.Thread(target=self.wait, args=[stop_time])
                    t.start()
                elif self.drive_state == "run_straight":
                    self.twist.linear.x = 0.3
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node('drive')
follower = Follower()
rospy.spin()