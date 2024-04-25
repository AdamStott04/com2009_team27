#!/usr/bin/env python3
#all the imports
import rospy
import sensor_msgs 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import cv2
import os
import roslaunch


class SearchAndExplore:
    def __init__(self):
        #initialize ros node
        rospy.init_node('search_bot')
       # Start SLAM node using subprocess
        
        #initialize parameters
        self.colour = rospy.get_param('~colour')
        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")
        self.rate = rospy.Rate(10)
        self.start_time = rospy.Time.now()
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        #initialize goal state and start pos
        self.goal_reached = False
        self.goals_reached = 0
        self.initial_position = None
        
        #set up subscribers
        self.bridge = CvBridge()
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
       
        #publishers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
       
        #variables
        self.snap_path = os.path.join(os.path.dirname(__file__), 'snaps')
        rospy.loginfo("Path to snaps directory: %s", self.snap_path)
        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.detected_beacon = False
        self.twist_cmd = Twist()

        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        rospy.loginfo("Shutting down")
        # Stop the robot (set velocities to zero)
        self.twist_cmd.linear.x = 0.0
        self.twist_cmd.angular.z = 0.0
        self.twist_pub.publish(self.twist_cmd)
        cv2.destroyAllWindows()
        self.ctrl_c = True
        self.save_map()
        # Kill the SLAM process
        self.slam_process.kill()
    
    def save_map(self):
        # Save the map using map_saver tool
        map_saver_node = roslaunch.core.Node(
            package='map_server',
            node_type='map_saver',
             
            name='map_saver',
            args=['-f', os.path.join(os.path.dirname(__file__), 'maps', 'task4_map')]
        )
        self.slam_launch.launch(map_saver_node)

    def camera_callback(self, img_data):
        #take picture
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        #assign values
        colour = self.colour
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        #crop picture
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower = (0, 0, 0)
        upper = (0, 0, 0)
        #assign colour detection values
        if colour == "blue":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
        elif colour == "red":
            lower = (0, 224, 100)
            upper = (20, 255, 255)
        elif colour == "green":
            lower = (45, 224, 100)
            upper = (75, 255, 255)
        elif colour == "yellow":
            lower = (25, 224, 100)
            upper = (35, 255, 255)
        else:
            rospy.loginfo("Enter a valid colour")
            return
        #make mask
        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
        m = cv2.moments(mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', res)
        cv2.waitKey(1)

        
        if self.detected_beacon:
            file_name = os.path.join(self.snap_path, 'task4_beacon.jpg')
            cv2.imwrite(file_name, crop_img)
            rospy.loginfo(f"Photo of {self.colour} beacon captured.")
        

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
    def move_robot(self):

        while self.initial_position is None:
            rospy.logwarn("Waiting for initial position...")
            rospy.sleep(1)

        # Define exploration as the opposite corner
        goal = PoseStamped()
        goal.header.frame_id = "map"
        if self.goals_reached == 0:
            goal.pose.position.x = 1.5
            goal.pose.position.y = -1.5
            goal.pose.position.z = 0.0
        elif self.goals_reached == 1:
            goal.pose.position.y = 1.5
        elif self.goals_reached == 2:
            goal.pose.position.x = 1.5
        elif self.goals_reached == 3:
            goal.pose.position.y = -1.5

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
        self.goals_reached += 1

    def main(self):
        while not rospy.is_shutdown() and not self.ctrl_c:
            elapsed_time =  rospy.Time.now() -self.start_time 
            if elapsed_time.to_sec() > 180:
                rospy.logwarn("More than 180 seconds have elapsed")
                break
            self.move_robot()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = SearchAndExplore()
        explorer.main()
    except rospy.ROSInterruptException:
        pass