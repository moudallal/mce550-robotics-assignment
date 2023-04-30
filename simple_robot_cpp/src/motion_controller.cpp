// motion_controller.cpp
// Author: Mohamad Moudallal
// GitHub: https://github.com/moudallal
// Email: mohamad@moudallal.xyz

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "simple_robot_msgs/Obstacle.h"
#include "simple_robot_msgs/ChangeVelocity.h"

// Initialize publisher and subscriber
ros::Publisher publisher;
ros::Subscriber subscriber;

float linear_x = 0.3;
float angular_z = 0.5;

// Subscriber callback method
void obstacleCallback(const simple_robot_msgs::Obstacle data) {
    // Move the robot forward
    geometry_msgs::Twist move;
    move.angular.z = 0;
    move.linear.x = linear_x;

    // Evaluate the distance between the robot and the obstacle
    if (data.distance < 1) {
        ROS_INFO("%s detected at %1.3f meters away!", data.name.c_str(), data.distance);
        ROS_INFO("SEARCHING FOR SAFE PATH");
        //  Rotate the robot to the left to find a safe path
        move.linear.x = 0;
        move.angular.z = angular_z;
    } else if (isinf(data.distance)) {
        ROS_INFO("No obstacles detected.");
        ROS_INFO("MOVING FORWARD");
    } else {
        ROS_INFO("Obstacle is still far. Distance: %1.3f meters", data.distance);
        ROS_INFO("MOVING FORWARD");
    }

    publisher.publish(move);
}

bool handleChangeVelRequest(simple_robot_msgs::ChangeVelocity::Request& req, simple_robot_msgs::ChangeVelocity::Response& res) {
    ROS_INFO("Change Velocity request received - linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

    linear_x = req.linear_x;
    angular_z = req.angular_z;

    res.success = true;

    return true;
}

int main(int argc, char **argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "motion_controller");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Define publisher on topic /cmd_vel
    publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Define subscriber on topic /obstacle_detection
    subscriber = n.subscribe<simple_robot_msgs::Obstacle>("/obstacle_detection", 10, obstacleCallback);

    // Define service server on /simple_robot_cpp/change_vel
    ros::ServiceServer change_vel_service = n.advertiseService("/simple_robot_cpp/change_vel", handleChangeVelRequest);

    // Keep the node in spin
    ros::spin();

    return 0;
}