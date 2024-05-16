#!/usr/bin/env python3
#all the imports
import rospy
import cv2
import os
import roslaunch
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_ops)

        self.camera_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
        self.initial_position = None
        self.current_scan = None
        self.colour_detected = False
        self.robot_controller = Tb3Move()
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0
        self.centered = False
        self.m00 = 0
        self.pillar_locations = []
        self.m00_min = 10000
        self.current_colour = ""
        self.pillar_seen = False
        self.pillar_found = False
        self.moving_to_edge = False
        self.take_picture = False
        self.pillars_found = 0


        # Set the publishing rate to 10Hz
        self.rate = rospy.Rate(10)
        self.start_time = rospy.Time.now()
        self.time_limit = rospy.Duration(180)

        self.turn_vel_fast = 0.5  # Adjust this value as needed
        self.turn_vel_slow = 0.2  # Adjust this value as needed
        self.forward_vel = 0.2  # Adjust this value as needed
        self.avoidance_distance_threshold = 0.4  # Adjust this value as needed
         
        self.bridge = CvBridge()
        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
            
    
    def shutdown_ops(self):
        rospy.sleep(1)
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def scan_callback(self, scan_msg):
        # Store the latest laser scan data
        self.current_scan = scan_msg
        self.ranges = scan_msg.ranges
       
    def camera_callback(self, img_data):
        self.rate = rospy.Rate(10)
        # Take picture
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")   
        except CvBridgeError as e:
            rospy.logerr(e)
            
        # Assign values
        colour = self.colour
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        
        # Crop picture
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        colours = ["blue","red","green","yellow"]
        lowers = [(115,225,100),(-2,209,100),(56,211,100),(25,224,100)]
        uppers = [(130,255,255),(5,258,255),(61,258,255),(35,255,255)]
        self.masks = {colour: (lower, upper) for colour, lower, upper in zip(colours, lowers, uppers)}

        max_area_colour = ""  # Initialize max_area_colour
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, lowers[i], uppers[i])
            else:
                mask += cv2.inRange(hsv_img, lowers[i], uppers[i])
            self.current_colour = colours[i]
        
        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        max_area = 0
        if self.m00 > self.m00_min:
            for colour, (lower, upper) in self.masks.items():
                    mask = cv2.inRange(hsv_img, lower, upper)
                    m = cv2.moments(mask)
                    area = m['m00']
                    if area > max_area:
                        max_area = area
                        max_area_colour = colour
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            self.current_colour = max_area_colour
            #Check if the detected colour matches
            if max_area_colour == self.colour:
                self.colour_detected = True
                rospy.loginfo("Correct coloured beacon detected,")        
        if max_area_colour == self.colour and self.m00 > 20000000:
            print("SNAP")
            self.pillar_found = True
            self.pillars_found += 1
            remove_index = colours.index(self.current_colour)
            colours[remove_index] = ""
            lowers[remove_index] = 0
            uppers[remove_index] = 0
            self.masks = {colour: (lower, upper) for colour, lower, upper in zip(colours, lowers, uppers)}

            #call save_image
            self.save_image(crop_img)
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
    
    def odom_callback(self, msg):
        # Store robot's initial position
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position
        #store angle
        self.current_pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
    def search_for_pillars(self):
        self.pillar_seen = False
        #check if pillar visible
        while self.pillar_seen == False:
            if rospy.is_shutdown():
                return
                
            if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.pillar_seen = True  # Increment pillars found
                    else:
                        self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
            #spin until it is centered
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, 0.5)
            elif self.move_rate == 'slow':
                if abs(self.cy - 560) < 10:  #stops it from spinnning past the pillar
                    self.robot_controller.set_move_cmd(0.0, 0.0)  # Stop moving
                else:
                    self.robot_controller.set_move_cmd(0.0, 0.1)  # Continue moving
            
            #work out distance and set coordinates
            self.robot_controller.publish()
            self.rate.sleep()
        
    
    def move_robot(self):
        self.save_map()
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # wait for initial position
            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)
            """# find farest point
            max_range_index = self.ranges.index(max(self.ranges))
            # Return the lowest distance found
            rospy.loginfo(f"angle of approach is {max_range_index}")
            #move to this point
            destination_reached = False
            while destination_reached != True:
                if rospy.is_shutdown():
                        return
                if self.current_yaw * (180/np.pi) < 0:
                    self.current_angle = int(360-(np.round(-1*(self.current_yaw * (180/np.pi)))))
                else:
                    self.current_angle = int(np.round(self.current_yaw * (180/np.pi)))
                if self.current_angle != float(max_range_index) and self.moving_to_edge == False:
                    self.robot_controller.set_move_cmd(0, 0.5)
                elif self.ranges[0] > 0.4:
                    #Slow down when obstacle is detected
                    self.robot_controller.set_move_cmd(0.1,0)
                    self.moving_to_edge = True
                else:
                    destination_reached = True
                    self.robot_controller.set_move_cmd(0,0)       
                self.robot_controller.publish()
            #initial turn
            while any(self.ranges[i] > 0.4 for i in range(60,120)):
                self.robot_controller.set_move_cmd(0,-0.5)
                self.robot_controller.publish()
            #log point and time
            self.start_point = self.current_pos
            start_time = rospy.Time.now()        """

            #loop till all four pillars are found
            while self.pillars_found < 4:
                self.pillar_found = False
                #find pillar locations
                self.search_for_pillars()
                destination_reached = False
                #drive to pillar
                while destination_reached == False:
                    if any(self.ranges[i] < 0.4 for i in range(0,15)) or any(self.ranges[j] < 0.4 for j in range(345, 360)):
                        destination_reached = True
                        self.robot_controller.set_move_cmd(0,0)
                    else:
                        self.robot_controller.set_move_cmd(0.2,0)
                    self.robot_controller.publish()
                #spin till wall is on the left
                while any(self.ranges[i] > 0.5 for i in range(60,110)):
                    self.robot_controller.set_move_cmd(0,-0.5)
                    self.robot_controller.publish()
                    if rospy.is_shutdown():
                        return

                #made it to the room now to find this pillar
                start_pos = self.current_pos
                range_count = 0
                while range_count < 360 or self.pillar_found == False:
                    range_count = np.sum(self.ranges)
                    #check if its doing a loop of the arena
                    dx = self.current_pos.x - start_pos.x
                    dy = self.current_pos.y - start_pos.y
                    distance_from_goal = np.sqrt(dx**2 + dy**2)
                    if distance_from_goal > 2:
                        self.robot_controller.set_move_cmd(0,-0.5)
                        rospy.sleep(4)
                        print("too far")
                    if rospy.is_shutdown():
                        return
                    #left
                    if all(self.ranges[l] < 0.6 for l in range(80, 100)):#if the wall is on the left
                        if any(self.ranges[f] < 0.5 for f in range(0,15)) or  any(self.ranges[f] < 0.5 for f in range(340,360)):#if the wall is in front
                            print("In front")
                            self.robot_controller.set_move_cmd(0,-0.3)
                        else:
                            print("move eet") 
                            self.robot_controller.set_move_cmd(0.15,0) 
                    elif all(self.ranges[l] > 0.6 for l in range(80, 100)):
                        if any(self.ranges[f] < 0.5 for f in range(0,15)) or any(self.ranges[f] < 0.5 for f in range(340,360)) :#if the wall is in front
                            print("In front")
                            self.robot_controller.set_move_cmd(0.1,0.1)
                        else:
                            print("lerft")
                            self.robot_controller.set_move_cmd(0,0.6)#if wall not on the left spinny spinny 
                    self.robot_controller.publish()
                    
                    """ if range_count > 500:
                        rospy.loginfo("In good space")
                        self.robot_controller.set_move_cmd(0.1,0.7) #og value was 0.1,0.3
                        print("turning left")
                        if any(self.ranges[i] < 0.4 for i in range(1, 16)) or any(self.ranges[j] < 0.4 for j in range(345, 360)):
                            self.robot_controller.set_move_cmd(0,-0.7) #og value was 0,-0.5
                            print("object ahead") 
                    else:
                        rospy.sleep(1) 
                        self.robot_controller.set_move_cmd(0.15,0) """
                    
                    self.robot_controller.publish()
                print("DONE")
        self.save_map()
    '''
    def move_robot(self):
        self.save_map()
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # Wait for initial position
            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)
            
            # Check if the correct colored beacon is detected
            if self.colour_detected:
                rospy.loginfo("Correct colored beacon detected. Navigating towards it.")
                # Implement navigation towards the beacon while avoiding obstacles
                self.navigate_to_beacon()
                break  # Exit the loop once the beacon is reached
            
            # Perform a spinning motion to detect the correct colored beacon
            rospy.loginfo("Performing a spin to detect the correct colored beacon...")
            self.robot_controller.set_move_cmd(0, 0.5)  # Spin motion
            self.robot_controller.publish()
            rospy.sleep(5)  # Adjust the duration of spinning as needed
            self.robot_controller.set_move_cmd(0, 0)  # Stop spinning
            
            # Update the color detection flag based on the latest camera data
            if self.colour_detected:
                rospy.loginfo("Correct colored beacon detected. Navigating towards it.")
                self.navigate_to_beacon()
                break  # Exit the loop once the beacon is reached

            rospy.logwarn("Correct colored beacon not detected. Continuing the search.")
            min_left_distance = min(self.ranges[270:360])  # Consider only the left side (270 to 360 degrees)
            
            if min_left_distance < 0.5:  # Adjust this threshold as needed
                # Move forward while keeping the left wall on the side
                self.robot_controller.set_move_cmd(self.forward_vel, -0.2)  # Move forward and slightly to the left
            else:
                # If the left wall is too far, turn left to approach it
                self.robot_controller.set_move_cmd(0, 0.5)  # Turn left
            
            self.robot_controller.publish()
            rospy.sleep(0.1)  # Add a small delay between iterations

    rospy.loginfo("Movement completed.")
    
    def navigate_to_beacon(self):
        while not rospy.is_shutdown():
            # Check for collision
            if self.check_collision():
                rospy.logwarn("Collision detected, attempting to avoid...")
                # Implement collision avoidance behavior here
                self.avoid_collision()
            
            # Calculate the direction to the beacon based on its position in the camera view
            direction = (self.cy - 400) / 400  # Calculate direction relative to the center
            
            # Move towards the beacon while avoiding obstacles
            # Adjust the robot's heading based on the direction to the beacon
            forward_vel = 0.2  # Adjust the forward velocity as needed
            turn_scale = 0.5  # Adjust the turn scale as needed
            turn_vel = turn_scale * direction
            
            # Limit the turn velocity within a reasonable range
            turn_vel = max(min(turn_vel, 0.5), -0.5)
            
            # Set the robot's movement command
            self.robot_controller.set_move_cmd(forward_vel, turn_vel)
            
            # Publish the movement command
            self.robot_controller.publish()
            
            rospy.sleep(0.1)  # Add a small delay between iterations
    def check_collision(self):
        ranges = self.current_scan.ranges
        obstacle_detection_range = 0.5

        for i, distance in enumerate(ranges):
            if distance < obstacle_detection_range:
                obstacle_angle = self.current_scan.angle_min + i * self.current_scan.angle_increment
                obstacle_angle = (obstacle_angle + np.pi) % (2 * np.pi) - np.pi
                if abs(obstacle_angle) < np.pi / 2:
                    lateral_distance = distance * np.sin(abs(obstacle_angle))
                    if lateral_distance < 0.2:
                        return True
        return False
    
    def avoid_collision(self):
        # Stop the robot
        self.robot_controller.set_move_cmd(0, 0)
        rospy.sleep(1)  # Wait for a brief moment
        
        # Get the laser scan data
        ranges = self.current_scan.ranges
        
        # Find the direction with the most clearance
        left_clearance = min(ranges[45:135])  # Left side clearance
        right_clearance = min(ranges[225:315])  # Right side clearance
        
        if left_clearance > right_clearance:
            # Turn left to find a clear path
            self.robot_controller.set_move_cmd(0, 0.5)  # Set turn velocity
        else:
            # Turn right to find a clear path
            self.robot_controller.set_move_cmd(0, -0.5)  # Set turn velocity
        
        rospy.sleep(1)  # Allow time for turning
        
        # Resume forward motion while checking for obstacles
        forward_vel = 0.2  # Adjust the forward velocity as needed
        self.robot_controller.set_move_cmd(forward_vel, 0)  # Set forward velocity
        self.robot_controller.publish()  # Publish the movement command
        
        # Continue checking for obstacles and adjust motion until a clear path is found
        while self.check_collision():
            rospy.sleep(0.1)  # Wait for a brief moment for obstacle detection
            pass  # Continue adjusting motion until a clear path is found
        
        # Once a clear path is found, stop turning and resume forward motion
        self.robot_controller.set_move_cmd(forward_vel, 0)  # Set forward velocity
        self.robot_controller.publish()  # Publish the movement command
    '''

    def save_map(self):
        rospy.logdebug("saving map")
        package = "map_server"
        executable = "map_saver"
        maps_folder = "/home/student/catkin_ws/src/com2009_team27/maps"
        args = f"-f {maps_folder}/task4_map"
        node = roslaunch.core.Node(package, executable, args=args, output="screen")

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)

        rospy.logdebug("Map saved succesfully.")

    def save_image(self, img):
        for _ in range(20):
            print("Saving")
        dirPath = "/home/student/catkin_ws/src/com2009_team27/snaps/"
        path = dirPath + "task4_beacon.jpg"
        print(f"Saving the image to '{path}'...")
        try:
            cv2.imwrite(str(path), img)
            
        except Exception as e:
            print(e)
        print(f"Saved image to {path}")         

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