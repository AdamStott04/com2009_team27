#!/usr/bin/env python3
#all the imports
import rospy
import sensor_msgs 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os

class SearchAndExplore:
    def __init__(self):
        rospy.init_node('search_bot')
        #get target colour
        self.colour = rospy.get_param('~colour')
        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")

        self.rate = rospy.Rate(10)
        #get the time
        self.start_time = rospy.Time.now()

        #set up camera subscriber to get camera info
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        #velocity publisher to move wafflebot
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #construct path to the snaps directory
        self.snap_path = os.path.join(os.path.dirname(__file__), 'snaps')
        rospy.loginfo("Path to snaps directory: %s", self.snap_path)
        
        self.detected_beacon = False
        self.incident_count = 0
        self.twist_cmd = Twist()

    def image_callback(self, msg):
        #convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #implement beacon detection and capture photo logic here
        if self.detected_beacon:
            self.capture_photo(cv_image)
    
    def capture_photo(self, image):
        #Save captured photo to snaps
        file_name = os.path.join(self.snap_path, 'task4_beacon.jpg')
        cv2.imwrite(file_name, image)
        rospy.loginfo(f"Photo of {self.colour} beacon captured.")
    
    def move_robot(self):
        #todo
        pass
    
    def run(self):
        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now() - self.start_time
            if elapsed_time.to_sec() > 180:
                rospy.logwarn("More than 180 seconds have elapsed")
                break
            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = SearchAndExplore()
        explorer.run()
    except rospy.ROSInterruptException:
        pass