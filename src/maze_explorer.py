#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MazeExplorer:
    def __init__(self):
        rospy.init_node('maze_explorer')

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.twist = Twist()
        self.scan_data = []

        self.rate = rospy.Rate(10)  # 10 Hz
        self.time_limit = rospy.Duration(150)  # 90 seconds
        self.start_time = rospy.Time.now()

        # Flag to indicate if obstacle is detected
        self.obstacle_detected = False

        # Minimum distance threshold for obstacle detection
        self.min_distance_threshold = 0.6

    def scan_callback(self, data):
        # Store the scan data
        self.scan_data = data.ranges