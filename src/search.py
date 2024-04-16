#!/usr/bin/env python3

import rospy

class SearchAndExplore:
    def __init__(self):
        rospy.init_node('search_bot')
        colour = rospy.get_param('-colour', 'Black')
        rospy.log(f"TASK 4 BEACON: The target is {colour}")
        