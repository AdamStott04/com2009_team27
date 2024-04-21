#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class MazeExplorer:
    def __init__(self):
        rospy.init_node('maze_explorer')

        # Initialize variables
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.goal_reached = False
        self.initial_position = None

        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Set the publishing rate to 10Hz
        self.rate = rospy.Rate(10)
        self.time_limit = rospy.Duration(150) 
        self.start_time = rospy.Time.now()

        # Start exploring
        self.explore()

    def explore(self):
        while not rospy.is_shutdown() and rospy.Time.now() - self.start_time < self.time_limit:
            # Wait for the initial position to be received
            while self.initial_position is None:
                rospy.logwarn("Waiting for initial position...")
                rospy.sleep(1)

            # Define exploration as the opposite corner
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = 2
            goal.pose.position.y = 2 
            goal.pose.position.z = 0.0
            quat = quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = quat[0]
            goal.pose.orientation.y = quat[1]
            goal.pose.orientation.z = quat[2]
            goal.pose.orientation.w = quat[3]

            # Publish exploration goal
            rospy.loginfo("Exploring maze to goal: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal_pub.publish(goal)

            # Wait for the goal to be reached or for a new goal to be published
            while not self.goal_reached:
                if rospy.is_shutdown():
                    return
                self.rate.sleep()

            # Reset goal_reached flag for next exploration
            self.goal_reached = False

    def odom_callback(self, msg):
        # Store robot's initial position
        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position

    def move_base_status_callback(self, status):
        # Check if move_base reached the goal
        for goal_status in status.status_list:
            if goal_status.status == 3:  # SUCCEEDED
                self.goal_reached = True

if __name__ == '__main__':
    try:
        MazeExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
