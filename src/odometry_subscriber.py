#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomSubscriber():

    def callback(self, topic_data: Odometry):

        pose = topic_data.pose.pose

        position = pose.position
        orientation = pose.orientation 

        pos_x = position.x
        pos_y = position.y
        pos_z = position.z

        orientation_x = orientation.x
        orientation_y = orientation.y
        orientation_z = orientation.z
        orientation_w = orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')
        robot_odom = [pos_x, pos_y, pos_z, roll, pitch, yaw]
        if self.counter > 10:
            self.counter = 0
            print(f"x = {pos_x:.3f} (m), y = {pos_y:.3f}(m), theta_z = {yaw:.3f} (radians)")
        else:
            self.counter += 1

    def __init__(self):
        node_name = "odometry_subscriber"
        rospy.init_node(node_name, anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"The '{node_name}' node is active...")
        
        self.counter = 0

    def main(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    node = OdomSubscriber()
    node.main()