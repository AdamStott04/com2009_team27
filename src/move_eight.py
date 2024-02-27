#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion

class FigureOfEight:
    def __init__(self):
        rospy.init_node('move_eight', anonymous=True)
        self.rate = rospy.Rate(1)  # 1Hz

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Initialize variables
        self.start_time = rospy.get_time()
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def odometry_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (in radians)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def move_robot(self):
        while not rospy.is_shutdown():
            # Calculate time elapsed
            current_time = rospy.get_time()
            elapsed_time = current_time - self.start_time

            # Move the robot
            twist_msg = Twist()
                # First 30 seconds: Move anti-clockwise
            if elapsed_time < 30.0:  
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.2  # Adjust angular velocity as needed
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
                # Next 30 seconds: Move clockwise
            elif 30.0 < elapsed_time < 60.0 :
                twist_msg.linear.x = 0.2
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

            # Print odometry data to terminal
            rospy.loginfo("x={:.2f} [m], y={:.2f} [m], yaw={:.1f} [degrees]".format(
                self.current_x - self.initial_x, self.current_y - self.initial_y, self.current_yaw * 180.0 / pi))

if __name__ == '__main__':
    #ensures that the move_robot() method of the FigureOfEight class is called
    try:
        controller = FigureOfEight()
        controller.move_robot()
    except rospy.ROSInterruptException:
        pass