#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, os , time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
from pathlib import Path

class DetectSign:
       
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        simp_path = '/home/rosuser/practice/catkin_ws/src/deu_ros/scripts/ch11/red_sign.png'
        abs_path = os.path.abspath(simp_path)
        self.sign_image = cv2.imread(abs_path)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top+20
        
        
        #img2 = cv2.imread('sign.png', 0)  # trainImage
        #img3 = img2
        orb = cv2.ORB_create()
        
        kp1, des1 = orb.detectAndCompute(image,None) 
        #kp1, des1 = orb.detectAndCompute(img2,None)
        kp2, des2 = orb.detectAndCompute(self.sign_image,None)

        matcher = cv2.BFMatcher_create()

        matches = matcher.knnMatch(des1, des2, 2) 
        
        good_matches = []
        for m in matches: 
            if m[0].distance / m[1].distance < 0.7: 
                good_matches.append(m[0])
                
        
        sign_image = cv2.drawMatches(image, kp1, self.sign_image, kp2, good_matches, None)
        #sign_image = cv2.drawMatches(img2, kp1, self.sign_image, kp2, good_matches, None)
        
               
        if len(matches) >= 430 and len(matches) <= 433  and len(good_matches) >= 33 and len(good_matches) <= 35:
            print('# of matches:', len(matches))
            print('# of good_matches:', len(good_matches))
            rospy.loginfo("Detecting sign")
            self.twist.linear.x = 0.0
            time.sleep(5)
            #plt.imshow(sign_image), plt.show()
        
        else:
           self.twist.linear.x = 0.5               
        self.cmd_vel_pub.publish(self.twist)
        cv2.waitKey(3)
        cv2.destroyAllWindows()
                


rospy.init_node('detect')
detect =  DetectSign()
rospy.spin()
