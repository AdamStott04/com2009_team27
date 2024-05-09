#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LeftWallHugger:
    def __init__(self):
        rospy.init_node('maze_explorer')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()

    def scan_callback(self, msg):
        # Determine the direction to turn based on the scan data
        min_dist = min(msg.ranges)
        min_index = msg.ranges.index(min_dist)

        if min_dist < 1.0:
            # Obstacle detected, turn right
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0.0  # Stop linear motion when turning
        else:
            # No obstacle, follow left wall
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.2  # Move forward with linear velocity

        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        explorer = LeftWallHugger()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
