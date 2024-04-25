#!/usr/bin/env python3

import rospy

#image proccessing modules
import cv2
from cv_bridge import CvBridge, CvBridgeError


from tf.transformations import quaternion_from_euler

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SearchAndExplore:
    def __init__(self):
        node_name = 'search_bot'
        rospy.init_node(node_name)
        self.goal_reached = False
        self.initial_position = None
        self.colour = rospy.get_param('~colour')
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.m00 = 0
        self.m00_min = 10000
        self.cvbridge_interface = CvBridge()
        rospy.on_shutdown(self.shutdown_ops)

        self.ctrl_c = False
        self.rate = rospy.Rate(5)
        

        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")

        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.roam()
        
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
    def odom_callback(self, msg):
        # Store robot's initial position
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position

    def move_base_status_callback(self, status):
        # Check if move_base reached the goal
        for goal_status in status.status_list:
            if goal_status.status == 3:  # SUCCEEDED
                self.goal_reached = True

        #check for colour
    def roam(self):
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # Wait for the initial position to be received
            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)

            # Define exploration as the opposite corner
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = 2
            goal.pose.position.y = 2 
            goal.pose.position.z = 0.0
            quat = quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = quat[0]
            goal.pose.orientation.y = quat[1]
            goal.pose.orientation.z = quat[2]
            goal.pose.orientation.w = quat[3]

            # Publish exploration goal
            rospy.loginfo("Exploring maze to goal: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal_pub.publish(goal)

            # Wait for the goal to be reached or for a new goal to be published
            while not self.goal_reached:
                if rospy.is_shutdown():
                    return
                self.rate.sleep()

            # Reset goal_reached flag for next exploration
            self.goal_reached = False
    

if __name__ == '__main__':
    try:
        SearchAndExplore()
        rospy.spin()        
    except rospy.ROSInterruptException:
        pass