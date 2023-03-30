#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from simple_robot.msg import Obstacle

# Subscriber callback method
def callback(data):
    # Access the forward laser scan distance (0-2pi -> ranged[0]-ranges[359])
    obstacle.distance = data.ranges[0]
    # Send a dummy name
    obstacle.name = 'Obstacle'
    publisher.publish(obstacle)

# Initialize node
rospy.init_node('obstacle_detector', anonymous=True)
# Define publisher on topic /obstacle_detection
publisher = rospy.Publisher('/obstacle_detection', Obstacle, queue_size=10)
# Define subscriber on topic /scan
subscriber = rospy.Subscriber('/scan', LaserScan, callback)

# Set rate to 10Hz
r = rospy.Rate(10)
# Define the message to be published
obstacle = Obstacle()

# Keep the node in spin
rospy.spin()
