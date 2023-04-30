// process_image.cpp
// Author: Mohamad Moudallal
// GitHub: https://github.com/moudallal
// Email: mohamad@moudallal.xyz

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include "simple_robot_msgs/DriveVelocity.h"

// Initialize the chase service client
ros::ServiceClient chase_client;

// Call chase service to drive the robot toward the target
void call_chase_service(float linear_x, float angular_z) {

    // Request velocities
    simple_robot_msgs::DriveVelocity srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // Call the chase service and pass the requested velocities
    if (!chase_client.call(srv))
        ROS_ERROR("Failed to call DriveVelocity service");
}

// This callback function continuously executes and reads the image data
void processImageCallback(const sensor_msgs::Image img) {

    int white_pixel = 255;

    float linear_x;
    float angular_z;

    // Loop through all the pixels in the image
    for (int i = 0; i < img.height * img.step; i += 3) {

        // Check if the pixel is white
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            // Check in which side the pixel falls in (left, mid, right)
            if (i % img.step < img.step / 3) {
                linear_x = 0;
                angular_z = 0.5;
            } else if (i % img.step < 2*img.step / 3) {
                linear_x = 0.2;
                angular_z = 0;
            } else {
                linear_x = 0;
                angular_z = -0.5;
            }
            break;
        }

        // Stop the robot if there are no white pixels in the image 
        linear_x = 0;
        angular_z = 0;
    }

    // Send the required robot velocities to the chase service
    call_chase_service(linear_x, angular_z);
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the processImageCallback function
    ros::Subscriber camera_subscriber = n.subscribe("/camera/rgb/image_raw", 10, processImageCallback);

    chase_client = n.serviceClient<simple_robot_msgs::DriveVelocity>("/simple_robot_cpp/chase");

    // Handle ROS communication events
    ros::spin();

    return 0;
}