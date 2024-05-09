#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
import actionlib

class MazeExplorer:
    def __init__(self):
        rospy.init_node('maze_explorer')

        # Initialize variables
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_reached = True  # Initially True to start exploration
        self.initial_position = None
        self.previous_position = None  # Store the previous position
        self.previous_scan = None

        # Create a SimpleActionClient for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Subscribe to robot's initial position and laser scan
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)

        # Start exploring
        self.explore()

    def explore(self):
        while not rospy.is_shutdown():
            # Wait for the initial position to be received
            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)

            # Define exploration goals
            goals = [
                (2, 2),
                # Add more exploration goals here as needed
            ]

            for goal_x, goal_y in goals:
                if self.goal_reached:
                    goal = self.create_goal(goal_x, goal_y)
                    rospy.loginfo("Exploring maze to goal: ({}, {})".format(goal_x, goal_y))
                    self.goal_pub.publish(goal)
                    self.wait_for_goal_completion()

    def wait_for_goal_completion(self):
        self.goal_reached = False
        while not self.goal_reached and not rospy.is_shutdown():
            rospy.loginfo("Waiting for goal completion...")
            rospy.sleep(1)

    def odom_callback(self, msg):
        # Store robot's initial position
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position

        # Store robot's current position
        self.previous_position = msg.pose.pose.position

    def move_base_status_callback(self, status):
        # Check if move_base reached the goal
        for goal_status in status.status_list:
            if goal_status.status == 3:  # SUCCEEDED
                self.goal_reached = True

    def scan_callback(self, scan_msg):
        # Process the scan data and trigger replanning if necessary
        if self.previous_scan is not None and self.previous_position is not None:
            significant_change = self.detect_significant_change(scan_msg)
            if significant_change:
                rospy.loginfo("Significant change detected in the environment.")

        self.previous_scan = scan_msg

    def detect_significant_change(self, current_scan):
        THRESHOLD = 1.0
        scan_diff = [abs(curr_range - prev_range) for curr_range, prev_range in zip(current_scan.ranges, self.previous_scan.ranges)]
        position_change = self.calculate_position_change()
        return any(diff > THRESHOLD for diff in scan_diff) and position_change

    def calculate_position_change(self):
        if self.previous_position is None:
            return False
        current_position = self.initial_position
        distance = ((current_position.x - self.previous_position.x) ** 2 + (current_position.y - self.previous_position.y) ** 2) ** 0.5
        return distance > 0.2  # Consider a change significant if the robot has moved more than 0.1 meters

    def create_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # No rotation
        return goal

    def shutdown(self):
        rospy.loginfo("Shutting down Maze Explorer...")
        # Stop any ongoing actions here

if __name__ == '__main__':
    try:
        maze_explorer = MazeExplorer()
        rospy.on_shutdown(maze_explorer.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
