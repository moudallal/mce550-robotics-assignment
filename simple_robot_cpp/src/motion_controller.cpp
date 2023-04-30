// motion_controller.cpp
// Author: Mohamad Moudallal
// GitHub: https://github.com/moudallal
// Email: mohamad@moudallal.xyz

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "simple_robot_msgs/Obstacle.h"

// Initialize publisher and subscriber
ros::Publisher publisher;
ros::Subscriber subscriber;

// Subscriber callback method
void callback(const simple_robot_msgs::Obstacle data) {
    // Move the robot forward
    geometry_msgs::Twist move;
    move.angular.z = 0;
    move.linear.x = 0.3;

    // Evaluate the distance between the robot and the obstacle
    if (data.distance < 1) {
        ROS_INFO("%s detected at %1.3f meters away!", data.name.c_str(), data.distance);
        ROS_INFO("SEARCHING FOR SAFE PATH");
        //  Rotate the robot to the left to find a safe path
        move.linear.x = 0;
        move.angular.z = 0.5;
    } else if (isinf(data.distance)) {
        ROS_INFO("No obstacles detected.");
        ROS_INFO("MOVING FORWARD");
    } else {
        ROS_INFO("Obstacle is still far. Distance: %1.3f meters", data.distance);
        ROS_INFO("MOVING FORWARD");
    }

    publisher.publish(move);
}

int main(int argc, char **argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "motion_controller");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Define publisher on topic /cmd_vel
    publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Define subscriber on topic /obstacle_detection
    subscriber = n.subscribe<simple_robot_msgs::Obstacle>("/obstacle_detection", 10, callback);

    // Keep the node in spin
    ros::spin();

    return 0;
}