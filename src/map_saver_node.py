#!/usr/bin/env python3

import os
import rospy
from subprocess import call

def save_map():
    rospy.loginfo("Saving map...")
    # Get the absolute path to the directory containing this script
    script_directory = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the "maps" directory
    maps_directory = os.path.join(script_directory, '..', '..', 'maps')
    # Ensure the "maps" directory exists, create it if necessary
    if not os.path.exists(maps_directory):
        os.makedirs(maps_directory)
    # Construct the absolute path to the map file within the "maps" directory
    map_file_path = os.path.join(maps_directory, "my_map")
    # Call the map_saver command with the full path to the map file
    return_code = call(["rosrun", "map_server", "map_saver", "-f", map_file_path])
    if return_code != 0:
        rospy.logerr("Failed to save the map. Return code: %d" % return_code)
    else:
        rospy.loginfo("Map saved successfully.")

if __name__ == '__main__':
    rospy.init_node('map_saver_node')
    save_map()