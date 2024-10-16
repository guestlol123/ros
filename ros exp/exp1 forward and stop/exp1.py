# Import ROS libraries
import rospy
from sensor_msgs.msg import LaserScan

# Callback function to process laser scan data
def callback(data):
    # data.ranges contains the distances measured by the laser scanner
    min_distance = min(data.ranges)
    rospy.loginfo("Minimum distance to an obstacle: {:.2f} meters".format(min_distance))

def listener():
    # Initialize the ROS node
    rospy.init_node('obstacle_distance_listener', anonymous=True)

    # Subscribe to the LaserScan topic
    rospy.Subscriber('/scan', LaserScan, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
