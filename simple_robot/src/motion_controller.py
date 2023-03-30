#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from simple_robot.msg import Obstacle

# Subscriber callback method
def callback(data):
    # Move the robot forward
    move.linear.x = 0.3
    move.angular.z = 0

    # Evaluate the distance between the robot and the obstacle
    if data.distance < 1:
        rospy.loginfo('%s detected at %f meters away!' %(data.name, data.distance))
        rospy.loginfo('SEARCHING FOR SAFE PATH')
        # Rotate the robot to the left to find a safe path
        move.linear.x = 0
        move.angular.z = 0.5
    elif math.isinf(data.distance):
        rospy.loginfo('No obstacles detected.')
        rospy.loginfo('MOVING FORWARD')
    else:
        rospy.loginfo('Obstacle is still far. Distance: %f meters' %(data.distance))
        rospy.loginfo('MOVING FORWARD')

    publisher.publish(move)

# Initialize node
rospy.init_node('motion_controller', anonymous=True)
# Define publisher on topic /cmd_vel
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# Define subscriber on topic /obstacl_detection
subscriber = rospy.Subscriber('/obstacle_detection', Obstacle, callback)

# Set rate to 10Hz
r = rospy.Rate(10)
# Define the message to be published
move = Twist()

# Keep the node in spin
rospy.spin()
