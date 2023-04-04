#!/usr/bin/env python3

# obstacle_detector.py
# Author: Mohamad Moudallal
# GitHub: https://github.com/moudallal
# Email: mohamad@moudallal.xyz

import rospy
from sensor_msgs.msg import LaserScan
from simple_robot.msg import Obstacle

# Subscriber callback method
def callback(data):
    # Access the forward laser scan distance (0-2pi -> ranges[0]-ranges[359])
    range = []
    range.extend(list(data.ranges[355:360]))
    range.extend(list(data.ranges[0:5]))

    # Send the minimum distance in a 10-degree range for ensuring safety
    obstacle.distance = min(range)
    # Send a dummy name
    obstacle.name = 'Obstacle'
    publisher.publish(obstacle)

# Initialize ROS node
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
