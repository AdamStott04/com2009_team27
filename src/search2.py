#!/usr/bin/env python3
#all the imports
import rospy
import sensor_msgs 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import cv2
import os
import roslaunch
import subprocess

class SearchAndExplore:
    def __init__(self):
        #initialize ros node
        rospy.init_node('search_bot')

        # Start SLAM node using subprocess
        self.slam_process = subprocess.Popen(['roslaunch', 'turtlebot3_slam', 'turtlebot3_slam.launch'])

        #initialize parameters
        self.colour = rospy.get_param('~colour')
        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")
        self.rate = rospy.Rate(10)
        self.start_time = rospy.Time.now()

        #initialize subscribers
        self.bridge = CvBridge()
        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)


        #initialize publishers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        #initialize variables
        #construct path to the snaps directory
        self.snap_path = os.path.join(os.path.dirname(__file__), 'snaps')
        rospy.loginfo("Path to snaps directory: %s", self.snap_path)
        self.detected_beacon = False
        self.twist_cmd = Twist()
        self.m00 = 0
        self.m00_min = 10000
        self.ctrl_c = False

        #SLAM variables
        self.map_data = None

        # Obstacle avoidance variables
        self.obstacle_detected = False
        self.min_distance_threshold = 0.6
        self.twist_cmd = Twist()

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_ops)

    
    def scan_callback(self, data):
        #Check if obstacle is detected
        non_empty_data = [x for x in data.ranges[0:17] + data.ranges[343:] if x]
        if non_empty_data and min(non_empty_data) < self.min_distance_threshold:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
    
    def map_callback(self, data):
        #Store map data
        self.map_data = data.data
    
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
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        colour = self.colour
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower = (0, 0, 0)
        upper = (0, 0, 0)
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

        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
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
    
    def move_robot(self):
        if self.obstacle_detected:
            # Stop forward motion
            self.twist_cmd.linear.x = 0.0
            # Perform obstacle avoidance (turning)
            self.twist_cmd.angular.z = 1.4  # Turn left
        else:
            self.twist_cmd.linear.x = 0.1
            self.twist_cmd.angular.z = 0.0

        # Publish the Twist message
        self.twist_pub.publish(self.twist_cmd)
        pass
    
    def run(self):
        while not rospy.is_shutdown() and not self.ctrl_c:
            elapsed_time = rospy.Time.now() - self.start_time
            if elapsed_time.to_sec() > 180:
                rospy.logwarn("More than 180 seconds have elapsed")
                break
            # Move the robot
            self.move_robot()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = SearchAndExplore()
        explorer.run()
    except rospy.ROSInterruptException:
        pass