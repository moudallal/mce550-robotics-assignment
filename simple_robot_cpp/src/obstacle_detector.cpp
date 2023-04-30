// obstacle_detector.cpp
// Author: Mohamad Moudallal
// GitHub: https://github.com/moudallal
// Email: mohamad@moudallal.xyz

#include "ros/ros.h"
#include <vector>
#include <algorithm>

#include <sensor_msgs/LaserScan.h>
#include "simple_robot_msgs/Obstacle.h"

// Initialize the publisher and subscriber
ros::Publisher publisher;
ros::Subscriber subscriber;

// Subscriber callback method
void callback(const sensor_msgs::LaserScan data) {
    // Access the forward laser scan distance (0-2pi -> ranges[0]-ranges[359])
    std::vector<float> range;
    range.insert(range.begin(), data.ranges.begin() + 354, data.ranges.end());
    range.insert(range.end(), data.ranges.begin(), data.ranges.begin() + 5);
    
    simple_robot_msgs::Obstacle obstacle;
    // Send the minimum distance in a 10-degree range for ensuring safety
    obstacle.distance = *std::min_element(range.begin(), range.end());
    // Send a dummy name
    obstacle.name = "Obstacle";
    publisher.publish(obstacle);
}

int main(int argc, char **argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "obstacle_detector");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Define publisher on topic /obstacle_detection
    publisher = n.advertise<simple_robot_msgs::Obstacle>("/obstacle_detection", 10);
    // Define subscriber on topic /scan
    subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, callback);

    // Keep the node in spin
    ros::spin();

    return 0;
}