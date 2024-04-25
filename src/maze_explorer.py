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
        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.goal_reached = False
        self.initial_position = None
        self.current_scan = None
        self.previous_scan = None

        # Create a SimpleActionClient for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Subscribe to robot's initial position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

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

    def scan_callback(self, scan_msg):
        # Store the latest laser scan data
        self.current_scan = scan_msg

        # Process the scan data and trigger replanning if necessary
        self.process_scan()

    def process_scan(self):
        THRESHOLD = 0.1
        # Check if the current scan data and previous scan data are available
        if self.current_scan is None:
            rospy.logwarn("No laser scan data available.")
            return
        
        # Implement logic to process the latest laser scan data and detect changes in the environment
        # For example, detect new obstacles or changes in existing obstacles
        
        # Placeholder implementation: Compare the current scan data with the previous scan data
        if self.previous_scan is not None:
            # Calculate the absolute difference between consecutive scan ranges
            scan_diff = [abs(curr_range - prev_range) for curr_range, prev_range in zip(self.current_scan.ranges, self.previous_scan.ranges)]
            
            # Determine if there are significant changes in the environment
            significant_change = any(diff > THRESHOLD for diff in scan_diff)
            
            if significant_change:
                rospy.loginfo("Significant change detected in the environment.")
                self.trigger_replanning()
        else:
            rospy.logwarn("Previous laser scan data not available for comparison.")
        
        # Store the current scan data for comparison in the next iteration
        self.previous_scan = self.current_scan

    def trigger_replanning(self):
        # Call path planning algorithm to generate a new path
        new_path = self.generate_new_path()

        # Update the current planned path
        self.update_planned_path(new_path)

    def generate_new_path(self):
        # Implement logic to generate a new path based on updated environment information
        # This may involve calling the path planning algorithm again with the updated map

        # Placeholder implementation: Generate a simple new path from the current position to a random goal
        new_path = []

        # Check if the initial position is available
        if self.initial_position is None:
            rospy.logwarn("Initial position not available.")
            return new_path

        # Define a random goal position within the map bounds
        goal_x = 2
        goal_y = 2

        # Create a PoseStamped message for the goal position
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, 0)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        # Add the goal position to the new path
        new_path.append(goal)

        # Return the newly generated path
        return new_path

    def update_planned_path(self, new_path):
        # Update the robot's planned path to follow the newly generated path
        # This may involve updating waypoints or trajectory

        # Check if the new path is empty
        if not new_path:
            rospy.logwarn("Empty new path.")
            return

        # Create a goal for each waypoint in the new path and send it to move_base
        for pose_stamped in new_path:
            goal = MoveBaseGoal()
            goal.target_pose = pose_stamped

            # Send the goal to move_base
            self.move_base_client.send_goal(goal)

        # Log a message indicating that the planned path has been updated
        rospy.loginfo("Planned path updated.")

if __name__ == '__main__':
    try:
        MazeExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
