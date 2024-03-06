#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class ExplorerBot:
    def __init__(self):
        rospy.init_node('explorer_bot')

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.twist = Twist()
        self.scan_data = []

        self.rate = rospy.Rate(10)  # 10 Hz
        self.time_limit = rospy.Duration(90)  # 90 seconds
        self.start_time = rospy.Time.now()

        # Flag to indicate if obstacle is detected
        self.obstacle_detected = False

        # Minimum distance threshold for obstacle detection
        self.min_distance_threshold = 0.7  # Adjust as needed

    def scan_callback(self, data):
        # Store the scan data
        self.scan_data = data.ranges

    def explore(self):
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # Start moving forward by default
            self.twist.linear.x = 0.3  # Forward velocity
            self.twist.angular.z = 0.0  # No angular velocity

            # Check if obstacle is detected
            if self.obstacle_detected:
                # Stop forward motion
                self.twist.linear.x = 0.0
                # Check if there are no obstacles in the 90-degree cone
                non_empty_data = [x for x in self.scan_data[0:15] + self.scan_data[345:] if x]
                if not non_empty_data or min(non_empty_data) >= self.min_distance_threshold:
                    # No obstacles in the 90-degree cone, resume forward motion
                    self.obstacle_detected = False
                else:
                    # Turn left until there are no obstacles in the 90-degree cone
                    self.twist.angular.z = 0.5  # Turn left
            else:
                # Check if obstacle is within a certain range in the 90-degree cone
                non_empty_data = [x for x in self.scan_data[0:15] + self.scan_data[345:] if x]
                if non_empty_data and min(non_empty_data) < self.min_distance_threshold:
                    self.obstacle_detected = True

            # Publish the Twist message
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping Explorer Bot...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        explorer = ExplorerBot()
        explorer.explore()
    except rospy.ROSInterruptException:
        pass
