#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion

class FigureOfEight:
    def __init__(self):
        rospy.init_node('move_eight', anonymous=True)
        self.rate = rospy.Rate(10)  # 1Hz

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Initialize variables
        self.start_time = rospy.get_time()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.initial_yaw = 0.0
  

    def odometry_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (in radians)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def move_robot(self):
        log_rate = 10  # Log every 10th loop iteration
        log_counter = 0
        while not rospy.is_shutdown():
            # Calculate time elapsed
            current_time = rospy.get_time()
            elapsed_time = current_time - self.start_time
            is_first_loop = True
            # Move the robot
            twist_msg = Twist()
                # First loop: Move anti-clockwise
            if (elapsed_time < 30.0 or (self.current_x >= self.initial_x and self.current_y >= self.initial_y)) and is_first_loop == True:  
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = 0.2  # Adjust angular velocity as needed
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                # Next loop: Move clockwise
            elif elapsed_time < 60 or (self.current_x <= self.initial_x and self.current_y >= self.initial_yaw):
                is_first_loop = False
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -0.2  # Adjust angular velocity as needed
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                #stop robot when figure of eight is completed
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                rospy.loginfo("Figure-of-eight trajectory completed.")
                break

                # Log odometry data to terminal at 1 Hz
            if log_counter % log_rate == 0:
                rospy.loginfo("x={:.2f} [m], y={:.2f} [m], yaw={:.1f} [degrees]".format(
                    self.current_x - self.initial_x, self.current_y - self.initial_y, self.current_yaw * 180.0 / pi))

            log_counter += 1


if __name__ == '__main__':
    #ensures that the move_robot() method of the FigureOfEight class is called
    try:
        controller = FigureOfEight()
        controller.move_robot()
    except rospy.ROSInterruptException:
        pass