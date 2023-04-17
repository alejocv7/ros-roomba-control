#include "ca_msgs/Bumper.h"      // Express the status on the bump sensors
#include "geometry_msgs/Twist.h" // Express linear and angular velocity
#include "iostream "
#include "ros/ros.h"
#include "sensor_msgs/Joy.h" // Express the state of the joystick
#include "std_msgs/String.h" // Allows to recieve a "string" type message

using namespace std;

geometry_msgs::Twist vel_msg;  // Velocity data
bool left_bumper_sen = false;  // Left bump sensor
bool right_bumper_sen = false; // Right bump sensor
bool joystick_sen = false;     // Joystick sensor
bool ard_sen = false;          // Arduino sensor
bool should_celebrate = false; // Celebration sensor

void joystick_callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    joystick_sen = true;
    vel_msg.angular.z = joy - > axes[0]; // Move with angular velocity according to axes
    vel_msg.linear.x = joy->axes[1];     // Move with linear velocity according to axes 1

    if (joy->buttons[0]) {
        // Joystick ceebration button was pressed
        should_celebrate = false;
    }
}

void bumper_sensor_callback(const ca_msgs::Bumper::ConstPtr &bumper)
{
    // Detects the left bump sensor
    if (bumper->is_left_pressed) {
        left_bumper_sen = true;
    }

    // Detects the right bump sensor
    if (bumper->is_right_pressed) {
        right_bumper_sen = true;
    }
}

void arduino_callback(const std_msgs::String::ConstPtr &msg)
{
    ard_sen = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, " teleopJoy ");
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub1; // joystick
    ros::Subscriber sub2; // bumper sensor
    ros::Subscriber sub3; // Arduino

    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub1 = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_callback);
    sub2 = n.subscribe<ca_msgs::Bumper>("bumper", 10, &bumper_sensor_callback);
    sub3 = n.subscribe<std_msgs::String>("chatter", 10, &arduino_callback);

    ros::Rate loop_rate(10); // Data transmission freq 10 Hz

    int count = 0;
    while (ros::ok()) {
        if (joystick_sen == 0) {
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0;

            // Right turn
            if (left_bumper_sen) {
                ++count;

                if (count < 10) {
                    // Move backward
                    vel_msg.linear.x = -0.5;
                    vel_msg.angular.z = 0;
                } else if (count >= 10 && count < 14) {
                    // Turn right
                    vel_msg.linear.x = 0;
                    vel_msg.angular.z = -2;
                } else if (count >= 14) {
                    // Restart variable states
                    left_bumper_sen = 0;
                    count = 0;
                    vel_msg.angular.z = 0;
                }
            }

            // Left turn
            if (right_bumper_sen) {
                ++count;

                if (count < 10) {
                    // Move backward
                    vel_msg.linear.x = -0.5;
                    vel_msg.angular.z = 0;
                } else if (count >= 10 && count < 14) {
                    // Turn left
                    vel_msg.linear.x = 0;
                    vel_msg.angular.z = 2;
                } else if (count >= 14) {
                    // Restart the states of the variables
                    right_bumper_sen = 0;
                    count = 0;
                    vel_msg.angular.z = 0;
                }
            }
        } else {
            // Return to free move if robot has stopped moving after joystick was used ,
            // and restart the states of the joystic sensor and count
            ++count;
            if (vel_msg.linear.x == 0 && vel_msg.linear.z == 0 && count >= 15) {
                joystick_sen = 0;
                count = 0;
            }
        }

        // Celebration
        if (should_celebrate) {
            ++count;
            // Turn left with angular velocity = 2 and until count >=35, then stop and
            // clear varibles and sensors
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 2;
            if (count >= 35) {
                count = 0;
                should_celebrate = 0;
            }
        }

        // Arduino
        if (ard_sen) {
            ++count;
            // Turn left with angular velocity = 2 and until count >=10, then stop and
            // varibles and sensors
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 2;
            if (count >= 10) {
                count = 0;
                ard_sen = 0;
            }
        }

        pub.publish(vel_msg); // Publish vel_msg
        ros::spinOnce();      // Read the topics and called the corresponding function
        loop_rate.sleep();    // Sleep for the necessary time to get a 10 Hz frecuenzy
    }
}
