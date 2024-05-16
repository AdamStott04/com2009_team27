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

        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
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
        self.moving_to_edge = False
        self.take_picture = False
        self.pillars_found = 0


        # Set the publishing rate to 10Hz
        self.rate = rospy.Rate(10)
        self.start_time = rospy.Time.now()
        self.time_limit = rospy.Duration(180)
         
        self.bridge = CvBridge()
        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
            
    
    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def scan_callback(self, scan_msg):
        # Store the latest laser scan data
        self.current_scan = scan_msg
        self.ranges = scan_msg.ranges
       
    def camera_callback(self, img_data):
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
            print(f"The detected color of the pillar is {max_area_colour}.")
        if self.m00 > 20000000:
            print("SNAP")
            self.pillar_seen = True
            self.masks.pop(self.current_colour, None)
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
        last_detection_time = rospy.Time.now()
        #check if pillar visible
        while self.pillars_found != 1:
            if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.pillars_found += 1  # Increment pillars found
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
                if abs(self.cy - 560) < 10:  # Adjust the threshold for being at the center
                    self.robot_controller.set_move_cmd(0.0, 0.0)  # Stop moving
                else:
                    self.robot_controller.set_move_cmd(0.0, 0.2)  # Continue moving
            
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
            # find farest point
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
                    self.robot_controller.set_move_cmd(0.2,0)
                    rospy.loginfo(self.ranges[0])
                    self.moving_to_edge = True
                else:
                    destination_reached = True
                    self.robot_controller.set_move_cmd(0,0)
                self.robot_controller.publish()
            #initial turn
            while any(self.ranges[i] > 0.5 for i in range(65,90)):
                self.robot_controller.set_move_cmd(0,-0.5)
                self.robot_controller.publish()
            #log point
            self.start_point = self.current_pos
            #loop keeping the wall on the left breaking if it sees a pillar or retutns to the original point
            while self.pillar_seen == False:
                if rospy.is_shutdown():
                    return
                if any(self.ranges[j] < 0.4 for j in range(1, 15)) or any(self.ranges[j] < 0.4 for j in range(345, 360)) or all(self.ranges[i] > 0.4 for i in range(65, 100)):  
                    self.robot_controller.set_move_cmd(0.1,0.2) #og value was 0.1,0.3
                    print("turning left")
                    if any(self.ranges[i] < 0.4 for i in range(1, 16)) or any(self.ranges[j] < 0.4 for j in range(345, 360)):
                        self.robot_controller.set_move_cmd(0,-0.7) #og value was 0,-0.5
                        print("object ahead")
                else:
                    rospy.sleep(1) 
                    self.robot_controller.set_move_cmd(0.15,0) 
                
                self.robot_controller.publish()
    
    def save_map(self):
        rospy.logdebug("saving map")
        package = "map_server"
        executable = "map_saver"
        maps_folder = "/home/student/catkin_ws/src/com2009_team27/maps"
        rate = rospy.Rate(0.5)
        args = f"-f {maps_folder}/task4_map"
        node = roslaunch.core.Node(package, executable, args=args, output="screen")

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        rate.sleep()

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