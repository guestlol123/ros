#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_turtlebot():
    rospy.init_node('turtlebot_mover', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    move_cmd = Twist()
    stop_cmd = Twist()

    move_cmd.linear.x = 0.5  # Move forward with linear velocity 0.5
    stop_cmd.linear.x = 0.0  # Stop

    while not rospy.is_shutdown():
        # Move forward for 2 seconds
        rospy.loginfo("Moving forward")
        pub.publish(move_cmd)
        rospy.sleep(2)  # Use rospy.sleep() instead of time.sleep()

        # Stop for 2 seconds
        rospy.loginfo("Stopping")
        pub.publish(stop_cmd)
        rospy.sleep(2)  # Use rospy.sleep() instead of time.sleep()

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass
