#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LeftWallFollower:
    def __init__(self):
        rospy.init_node('left_wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def laser_callback(self, msg):
        # Assuming LiDAR data is arranged as an array of ranges
        # Determine the direction to turn based on the left-hand wall
        left_distances = msg.ranges[0:180]  # Assuming 180-degree field of view
        min_distance = min(left_distances)
        desired_distance = 0.25  # Adjust as needed
        if min_distance < desired_distance:
            # Turn right to keep the left-hand wall on the left side
            self.twist.angular.z = -0.5  # Adjust angular velocity as needed
        else:
            # Move forward
            self.twist.linear.x = 0.2  # Adjust linear velocity as needed

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        wall_follower = LeftWallFollower()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass
