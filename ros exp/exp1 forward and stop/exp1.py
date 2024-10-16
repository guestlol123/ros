#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('turtlebot_mover', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Forward speed
    move_cmd.angular.z = 0.0

    stop_cmd = Twist()
    move_cmd.linear.x = 0.0  # Stop

    while not rospy.is_shutdown():
        # Move forward
        rospy.loginfo("Moving forward")
        cmd_pub.publish(move_cmd)
        rospy.sleep(5)  # Move forward for 5 seconds

        # Stop
        rospy.loginfo("Stopping")
        cmd_pub.publish(stop_cmd)
        rospy.sleep(2)  # Stop for 2 seconds

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
