#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Initialize global variables for LaserScan readings
front_range = 0
left_range = 0
right_range = 0

# Define thresholds for obstacle avoidance
OBSTACLE_THRESHOLD = 0.5  # meters

def laser_callback(msg):
    global front_range, left_range, right_range

    # Obtain LaserScan data for front, left, and right directions
    front_range = min(min(msg.ranges[0:10] + msg.ranges[-10:]), 3.5)
    left_range = min(msg.ranges[70:110])
    right_range = min(msg.ranges[-110:-70])

def wander():
    global front_range, left_range, right_range

    # Initialize the ROS node
    rospy.init_node('wander_node')

    # Create a Publisher for velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Create a Subscriber for the LaserScan topic
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Define Twist message
    move_cmd = Twist()

    while not rospy.is_shutdown():
        # Check for obstacles in the front, left, and right directions
        if front_range < OBSTACLE_THRESHOLD:
            # Obstacle in front: turn either left or right depending on side ranges
            if left_range < right_range:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.5  # Turn right
            else:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5   # Turn left
        else:
            # No obstacle in front, move forward
            move_cmd.linear.x = 0.3
            move_cmd.angular.z = 0.0

        # Publish the command
        cmd_vel_pub.publish(move_cmd)

        # Sleep for the remaining time
        rate.sleep()

if __name__ == '__main__':
    try:
        wander()
    except rospy.ROSInterruptException:
        pass
