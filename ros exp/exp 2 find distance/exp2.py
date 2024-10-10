#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

# Callback function to process LaserScan messages
def scan_callback(msg):
    # Get the range directly in front of the robot
    range_ahead = msg.ranges[int(len(msg.ranges) / 2)]
    print("Range ahead: %0.1f meters" % range_ahead)

# Initialize the ROS node
rospy.init_node('range_ahead')

# Subscriber to the 'scan' topic
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Keep the program alive
rospy.spin()
