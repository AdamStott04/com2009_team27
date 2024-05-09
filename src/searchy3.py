#!/usr/bin/env python3
#all the imports
import rospy
import sensor_msgs 
import cv2
import actionlib
import os
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from tb3 import Tb3Move




class SearchAndExplore:
    def __init__(self):
        #initialize ros node
        rospy.init_node('search_bot')
       
        #get target colour
        self.colour = rospy.get_param('~colour')
        rospy.loginfo(f"TASK 4 BEACON: The target is {self.colour}")

        self.goals_reached = 0
        self.move_base_client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.goal_reached = False
        self.initial_position = None
        self.current_scan = None
        self.previous_scan = None
        self.colour_detected = False
        self.cvbridge_interface = CvBridge()
        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.2
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0
        self.centered = False
        self.m00 = 0
        self.pillar_locations = []
        self.m00_min = 10000
        self.is_moving = False
        self.current_colour = ""
        self.angular_speed = 0.5  # Angular speed for turning
        self.linear_speed = 0.2   # Linear speed for moving forward
        self.desired_distance = 0.5  # Desired distance from the wall
        self.distance_threshold = 0.05  # Distance threshold for wall following
        self.wall_angle_threshold = 10  # Angle threshold to detect a wall
        self.knocking_on_heavens_door = False #checks if robot is outside 
        self.last_goals_reached = 0

        # Set the publishing rate to 10Hz
        self.rate = rospy.Rate(10)
        self.start_time = rospy.Time.now()
        self.time_limit = rospy.Duration(180)
         

        self.bridge = CvBridge()

        
        #construct path to the snaps directory
        self.snap_path = os.path.join(os.path.dirname(__file__), 'snaps')
        rospy.loginfo("Path to snaps directory: %s", self.snap_path)

        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
            
    
    def shutdown_ops(self):
        rospy.loginfo("Shutting down")
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    def scan_callback(self, scan_msg):
        # Store the latest laser scan data
        self.current_scan = scan_msg
        self.ranges = scan_msg.ranges
       
    def velocity_callback(self,velocity_msg):
        if abs(velocity_msg.linear.x) > 0 or abs(velocity_msg.angular.z) > 0:
        # Robot is moving
            self.is_moving = True            
        else: 
            self.is_moving = False
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
        colours = ["blue","red","green","yellow"]
        lowers = [(115,225,100),(25,224,100),(56,211,100),(-2,209,100)]
        uppers = [(130,255,255),(35,255,255),(61,258,255),(5,258,255)]
        

        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, lowers[i], uppers[i])
            else:
                mask += cv2.inRange(hsv_img, lowers[i], uppers[i])
            self.current_colour = colours[i]

        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
    
    def odom_callback(self, msg):
        # Store robot's initial position
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position
        #store angle
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])



    def move_base_status_callback(self, status):
        # Check if move_base reached the goal
        for goal_status in status.status_list:
            print(len(status.status_list))
            if goal_status.status == 3:  # SUCCEEDED 
                self.goals_reached += 1
                if self.goals_reached == self.last_goals_reached + 1:
                    print("KACHING")
                    self.goal_reached = True
        self.last_goals_reached = self.goals_reached
        self.goals_reached = 0

    def search_for_pillars(self):
        self.pillar_locations = [None] * 4
        self.pillar_angles = [None] * 4
        last_detection_time = rospy.Time.now()
        pillars_found = 0
        #check if pillar visible
        while  pillars_found < 4:
            if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            if (rospy.Time.now() - last_detection_time).to_sec() >= 1.0:  # 1 second grace period
                                print("pillar stored")
                                distance = self.current_scan.ranges[0]
                                self.pillar_angles[pillars_found] = self.current_yaw
                                print(self.pillar_angles[pillars_found])
                                self.pillar_locations[pillars_found] = ((distance-0.4) * np.cos(self.current_yaw), (distance-0.4)* np.sin(self.current_yaw))
                                pillars_found += 1  # Increment pillars found
                                last_detection_time = rospy.Time.now()  # Update last detection time
                    else:
                        self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
            #spin until it is centered
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour {self.colour} of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
           #is the pillar behind a wall

            #work out distance and set coordinates
            self.robot_controller.publish()
            self.rate.sleep()

        print("LOOP LEFT")
    
    def move_robot(self):
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:

            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)

            # Move to the center for improved vision
            if self.centered == False:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.pose.position.x = 0.0
                goal.pose.position.y = 0.0
                goal.pose.position.z = 0.0
                quat = quaternion_from_euler(0, 0, 3.2)
                goal.pose.orientation.x = quat[0]
                goal.pose.orientation.y = quat[1]
                goal.pose.orientation.z = quat[2]
                goal.pose.orientation.w = quat[3]
                rospy.loginfo("Exploring to goal: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))


                # Publish exploration goal
                self.goal_pub.publish(goal)


            # Wait for the goal to be reached or for a new goal to be published
            while not self.goal_reached:
                if rospy.is_shutdown():
                    return
                self.rate.sleep()

            # Reset goal_reached flag for next exploration
            self.goal_reached = False
            self.centered = True

            if len(self.pillar_locations) < 4:
                # Identify the four pillars by looking over the walls
                self.search_for_pillars()

            #Iterate through the goals
            for i in range(0,4):
                goal.pose.position.x = self.pillar_locations[i][0]
                goal.pose.position.y = self.pillar_locations[i][1]
                quat = quaternion_from_euler(0, 0, self.pillar_angles[i])
                goal.pose.orientation.x = quat[0]
                goal.pose.orientation.y = quat[1]
                goal.pose.orientation.z = quat[2]
                goal.pose.orientation.w = quat[3]
                self.goal_pub.publish(goal)    
                while not self.goal_reached:
                    if rospy.is_shutdown():
                        return
                    self.rate.sleep()
                self.goal_reached = False
                #when reached near goal explore around until the object is in full view
                print("knock knock knock")
                self.knocking_on_heavens_door = True
                self.move_base_client.cancel_goal()
                self.search_for_entrance()
        
    def search_for_entrance(self):
         # Find the index of the closest object in front of the robot
        min_range_index = self.ranges.index(min(self.ranges))
        while self.knocking_on_heavens_door == True:
            if rospy.is_shutdown():
                        return
            if min_range_index <= len(self.ranges) / 2 - self.wall_angle_threshold:
                # Wall detected on the left-hand side
                angular_velocity = 0 
                # Calculate the difference between desired distance and actual distance from the wall
                distance_error = self.desired_distance - self.ranges[min_range_index]

                # Apply proportional control to adjust the robot's angular velocity
                angular_velocity = self.angular_speed * distance_error

                # Limit the angular velocity within a certain range
                angular_velocity = max(-1.0, min(1.0, angular_velocity))

                # Set the robot's movement command
                self.robot_controller.set_move_cmd(self.linear_speed, angular_velocity)
            else:
                # Set the robot's movement command for spinning
                self.robot_controller.set_move_cmd(0.0, 0.5)

        # Publish the Twist message to control the robot's movement
        self.robot_controller.publish()


    def main(self):
        while not rospy.is_shutdown():
            # Continue running until 180 seconds have passed
            while (rospy.Time.now() - self.start_time).to_sec() < 180:
                rospy.sleep(1)  # Add a small delay to reduce CPU usage
                #Start the movement function
                self.move_robot()
            
            rospy.loginfo("180 seconds have elapsed, exiting.")
            break  # Exit the loop after 180 seconds

        rospy.loginfo("Shutting down.")
    

if __name__ == '__main__':
    try:
        explorer = SearchAndExplore()
        explorer.main()
        rospy.spin
    except rospy.ROSInterruptException:
        pass