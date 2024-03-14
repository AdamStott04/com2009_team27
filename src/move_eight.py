#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion

class FigureOfEight:
    def __init__(self):
        rospy.init_node('move_eight', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz so the robot will actually move irl
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Initialize variables
        self.start_time = None  # Will be set when the first odometry message is received
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
    
    # Wait for the first odometry message to initialize start_time
        while self.start_time is None and not rospy.is_shutdown():
            self.rate.sleep()
  
    #this function is called every 0.1 seconds by a node
    def odometry_callback(self, msg):
        # Set start_time only if it hasn't been set yet
        if self.start_time is None:
            self.start_time = rospy.get_time()
        # Check if initial position and orientation have been set
        if self.initial_x == 0.0 and self.initial_y == 0.0 and self.initial_yaw == 0.0:
            # Set initial position to actual initial position of robot
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (in radians)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        #updates current coordinates
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def shutdown(self):
        # Stop the robot when the node is shutdown
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        #rospy.loginfo("Robot stopped.")

    def move_robot(self):
        log_rate = 10  # Log every 10th loop iteration
        log_counter = 0
         # Initialize start_time when the robot starts moving
        self.start_time = rospy.get_time()
        is_first_loop = True
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            # Calculate time elapsed
            current_time = rospy.get_time()
            elapsed_time = current_time - self.start_time
            # Move the robot
            twist_msg = Twist()
            
            # Transition to the second loop and to stop
            if elapsed_time >= 25.0 and is_first_loop and abs(self.current_x - self.initial_x) < 0.1 and abs(self.current_y - self.initial_y) < 0.1 :
                if is_first_loop == True:
                    is_first_loop = False
                else: #second time it comes to initial position should stop
                    is_first_loop = True
            
            # First loop: Move anti-clockwise
            if elapsed_time < 35.0 and is_first_loop == True:  
                twist_msg.linear.x = 0.11
                twist_msg.angular.z = 0.22 #angular velocity has to be double linear for robot to do circle
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()

            # Next loop: Move clockwise
            elif is_first_loop == False and elapsed_time < 59:
                twist_msg.linear.x = 0.11
                twist_msg.angular.z = -0.22
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                
            #stop robot when figure of eight is completed
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                #rospy.loginfo("Figure-of-eight trajectory completed.")
                break

                # Log odometry data to terminal at 1 Hz
            if log_counter % log_rate == 0:
                rospy.loginfo("x={:.2f} [m], y={:.2f} [m], yaw={:.1f} [degrees]".format(
                    self.current_x, self.current_y, self.current_yaw * 180.0 / pi))

            log_counter += 1


if __name__ == '__main__':
    try:
        controller = FigureOfEight()
        controller.move_robot()
    except rospy.ROSInterruptException:
        pass