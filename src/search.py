#!/usr/bin/env python3

import rospy

#image proccessing modules
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

class SearchAndExplore:
    def __init__(self):
        node_name = 'search_bot'
        rospy.init_node(node_name)
        self.colour = rospy.get_param('~colour')
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.m00 = 0
        self.m00_min = 10000
        self.cvbridge_interface = CvBridge()
        rospy.on_shutdown(self.shutdown_ops)

        self.ctrl_c = False
        self.rate = rospy.Rate(5)

        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")
        
    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    #take picture and format it
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        colour = self.colour
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        #crop
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        #colour mask
        lower = (_,_,_)
        upper = (_,_,_)
        if colour == "{blue}":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
        elif colour == "{red}":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
        elif colour == "{green}":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
        elif colour == "{yellow}":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
        else:
            rospy.loginfo("enter a valid colour")
            quit

        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        #if our colour then return picture
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)


    #explore


        #check for colour

    

if __name__ == '__main__':
    SearchAndExplore()
    rospy.spin()        