#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LeftWallFollower:
    def __init__(self):
        rospy.init_node('leftWallHugger')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.twist = Twist()
        self.scan_data = []

        self.rate = rospy.Rate(10)  # 10 Hz
        self.time_limit = rospy.Duration(150)  # 150 seconds
        self.start_time = rospy.Time.now()

        # Flag to indicate if obstacle is detected
        self.obstacle_detected = False

        # Minimum distance threshold for obstacle detection
        self.min_distance_threshold = 0.49

    def scan_callback(self, data):
        # Store the scan data
        self.scan_data = data.ranges

    def wall_hug(self):
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # Start moving forward by default
            self.twist.linear.x = 0.34  # Forward velocity
            self.twist.angular.z = 0.0  # No angular velocity

            # Check if obstacle is detected
            if self.obstacle_detected:
                # Stop forward motion
                self.twist.linear.x = 0.0

                # Check clearance in different directions
                front_sensor_data = [x for x in self.scan_data[0:23] + self.scan_data[337:] if x]
                left_sensor_data = [x for x in self.scan_data[265:285] if x]
                right_sensor_data = [x for x in self.scan_data[75:95] if x]
								

                closest_front = min(front_sensor_data) if front_sensor_data else float('inf')
                closest_left = min(left_sensor_data) if left_sensor_data else float('inf')
                closest_right = min(right_sensor_data) if right_sensor_data else float('inf')

                rospy.loginfo("Front Distance: %f, Left Distance: %f, Right Distance: %f",
                              closest_front, closest_left, closest_right)

                if closest_front >= self.min_distance_threshold:
                    # No obstacles in the front, go forward
                    self.obstacle_detected = False
                else:
                    if closest_front < 0.2:
                        self.twist.linear.x = -0.1
                    # Check which direction has more clearance
                    if closest_left > closest_right:
                        # Turn left
                        self.twist.angular.z = -0.8
                    else:
                        # Turn right
                        self.twist.angular.z = 0.8
            else:
                # Check if obstacle is within a certain range in the 34-degree cone
                non_empty_data = [x for x in self.scan_data[0:20] + self.scan_data[340:] if x]
                behind = [x for x in self.scan_data[170:190] if x]
                if non_empty_data and min(non_empty_data) < self.min_distance_threshold:
                    self.obstacle_detected = True
                    self.twist.linear.x = 0.0
                if behind and min(behind) < 0.1:
                    self.obstacle_detected = True
                    self.twist.linear.x = 0.1

            # Publish the Twist message
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Stopping Bot...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        explorer = LeftWallFollower()
        explorer.wall_hug()
    except rospy.ROSInterruptException:
        pass
