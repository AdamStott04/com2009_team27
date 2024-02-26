#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Circle():
    def __init__(self):
        self.node_name = "circle_publisher"

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")
    
    def shutdownhook(self):
        #make the robot stop
        #create a Twist() message instance
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0 # m/s
        vel_cmd.angular.z = 0.0 # rad/s

        self.pub.publish(vel_cmd)
        rospy.loginfo("Robot stopped")

        #if ctrl c is pressed
        self.ctrl_c = True
    
    def main(self):
        linear_velocity = 0.26 #Maximum linear velocity of TurtleBot3
        angular_velocity = linear_velocity / 0.5

        while not self.ctrl_c:
            move_msg = Twist()
            move_msg.linear.x = linear_velocity
            move_msg.angular.z = angular_velocity

            self.pub.publish(move_msg)
            self.rate.sleep()
        #After exiting the loop, call the shutdown    
        self.shutdownhook()

if __name__ == '__main__': 
    node = Circle() 
    node.main()

