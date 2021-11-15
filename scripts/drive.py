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
        self.drive_state = "run"
        self.can_stop = "yes"
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.drive_tr = ""
        self.drive_sub = rospy.Subscriber('drive', drive_key, self.drive_cb)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def wait(self):
        print("process")
        time.sleep(3)
        self.can_stop = "yes"

    def cross_run(self):
        print("croos_run")
        time.sleep(3)
        self.drive_state = "run"


    def drive_cb(self, msg):
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
                self.twist.linear.x = 0.5
                self.cmd_vel_pub.publish(self.twist)
                p = threading.Thread(target=self.cross_run)
                p.start()
            else:
                self.drive_state = "run"
            self.can_stop = "no"
            t = threading.Thread(target=self.wait)
            t.start()


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node('drive')
follower = Follower()
rospy.spin()