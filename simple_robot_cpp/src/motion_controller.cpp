// motion_controller.cpp
// Author: Mohamad Moudallal
// GitHub: https://github.com/moudallal
// Email: mohamad@moudallal.xyz

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "simple_robot_msgs/Obstacle.h"
#include "simple_robot_msgs/DriveVelocity.h"
#include "simple_robot_msgs/ToggleMode.h"

// Initialize publisher and subscriber
ros::Publisher publisher;
ros::Subscriber subscriber;

// Default robot velocities
float default_linear_x = 0.3;
float default_angular_z = 0.5;

// Chase flag
bool chase = false;

// Robot actuation function
void drive_robot(float linear_x, float angular_z) {
    geometry_msgs::Twist move;
    move.angular.z = angular_z;
    move.linear.x = linear_x;

    publisher.publish(move);
}

// Subscriber callback function
void obstacleCallback(const simple_robot_msgs::Obstacle data) {
    // Check if robot is in WANDER mode
    if (!chase) {
        // Move the robot forward
        float angular_z = 0;
        float linear_x = default_linear_x;
        // Evaluate the distance between the robot and the obstacle
        if (data.distance < 1) {
            ROS_INFO("%s detected at %1.3f meters away!", data.name.c_str(), data.distance);
            ROS_INFO("SEARCHING FOR SAFE PATH");
            //  Rotate the robot to the left to find a safe path
            linear_x = 0;
            angular_z = default_angular_z;
        } else if (isinf(data.distance)) {
            ROS_INFO("No obstacles detected.");
            ROS_INFO("MOVING FORWARD");
        } else {
            ROS_INFO("Obstacle is still far. Distance: %1.3f meters", data.distance);
            ROS_INFO("MOVING FORWARD");
        }

        drive_robot(linear_x, angular_z);
    }
}

// Change Velocity service request handler function
bool handleChangeVelRequest(simple_robot_msgs::DriveVelocity::Request& req, simple_robot_msgs::DriveVelocity::Response& res) {
    // Check if robot is in WANDER mode
    if (!chase) {
        // Check if requested velocities are in the required range
        if (req.linear_x > 0  && req.linear_x <= 3 && req.angular_z <= 3 && req.angular_z >= -3) {
            ROS_INFO("Change Velocity request received - linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

            default_linear_x = req.linear_x;
            default_angular_z = req.angular_z;

            res.msg_feedback = "Velocity changed";
            res.success = true; 
        } else {
            ROS_WARN("Requested velocities are invalid. Choose velocities within the required range.");
            ROS_WARN("linear_x: ]0,3]\nangular_z: [-3,3]");
            res.msg_feedback = "Requested velocities out of range";
            res.success = false;
        }
    } else {
        res.msg_feedback = "Cannot change velocity. Robot is in CHASE mode!";
        res.success = false;
    }

    return true;
}

// Toggle Mode service request handler function
bool handleToggleModeRequest(simple_robot_msgs::ToggleMode::Request& req, simple_robot_msgs::ToggleMode::Response& res) {
    ROS_INFO("Toggle Mode request received");

    // Toggle the robot's mode
    chase = !chase;
    res.mode = chase ? "CHASE" : "WANDER";
    ROS_INFO("%s mode activated!", res.mode.c_str());

    return true;
}

// Chase service request handler function
bool handleChaseRequest(simple_robot_msgs::DriveVelocity::Request& req, simple_robot_msgs::DriveVelocity::Response& res) {
    // Check if the robot is in CHASE mode
    if (chase) {
        res.msg_feedback = "Chase request received";

        ROS_INFO("Chase request received - driving robot at: linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

        drive_robot(req.linear_x, req.angular_z);
    } else {
        res.msg_feedback = "Cannot chase. Robot is in WANDER mode!";
    }

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

    // Define service server on /simple_robot_cpp/toggle_mode
    ros::ServiceServer toggle_mode_service = n.advertiseService("/simple_robot_cpp/toggle_mode", handleToggleModeRequest);

    // Define service server on /simple_robot_cpp/chase
    ros::ServiceServer chase_service = n.advertiseService("/simple_robot_cpp/chase", handleChaseRequest);

    ROS_INFO("Initialized simple_robot in %s mode", chase ? "CHASE" : "WANDER");

    // Keep the node in spin
    ros::spin();

    return 0;
}
