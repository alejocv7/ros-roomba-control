#include "ca_msgs/Bumper.h"      // Express the status on the bump sensors
#include "geometry_msgs/Twist.h" // Express linear and angular velocity
#include "iostream "
#include "ros/ros.h"
#include "sensor_msgs/Joy.h" // Express the state of the joystick
#include "std_msgs/String.h" // Allows to recieve a "string" type message

using namespace std;

geometry_msgs::Twist vel_msg;         // Velocity data
bool is_left_bumper_pressed = false;  // Left bump sensor
bool is_right_bumper_pressed = false; // Right bump sensor
bool is_joystick_pressed = false;     // Joystick sensor
bool is_arduino_op = false;           // Arduino sensor
bool is_celebration = false;          // Celebration sensor

void joystick_callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    is_joystick_pressed = true;
    vel_msg.angular.z = joy->axes[0]; // Move with angular velocity according to axis 0
    vel_msg.linear.x = joy->axes[1];  // Move with linear velocity according to axis 1

    if (joy->buttons[0]) {
        // Joystick celebration button was pressed
        is_celebration = false;
    }
}

void bumper_sensor_callback(const ca_msgs::Bumper::ConstPtr &bumper)
{
    is_left_bumper_pressed = bumper->is_left_pressed;
    is_right_bumper_pressed = bumper->is_right_pressed;
}

void arduino_callback(const std_msgs::String::ConstPtr &msg)
{
    is_arduino_op = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, " teleopJoy ");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystick_callback);
    ros::Subscriber sub2 = n.subscribe<std_msgs::String>("chatter", 10, &arduino_callback);
    ros::Subscriber sub3 =
        n.subscribe<ca_msgs::Bumper>("bumper", 10, &bumper_sensor_callback);

    ros::Rate loop_rate(10); // Data transmission freq 10 Hz

    int count = 0;
    while (ros::ok()) {
        if (!is_joystick_pressed) {
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0;

            // Right turn
            if (is_left_bumper_pressed) {
                ++count;

                if (count < 10) {
                    // Move backward
                    vel_msg.linear.x = -0.5;
                    vel_msg.angular.z = 0;
                } else if (count < 14) {
                    // Turn right
                    vel_msg.linear.x = 0;
                    vel_msg.angular.z = -2;
                } else {
                    // Restart variable states
                    count = 0;
                    vel_msg.angular.z = 0;
                    is_left_bumper_pressed = false;
                }
            }

            // Left turn
            if (is_right_bumper_pressed) {
                ++count;

                if (count < 10) {
                    // Move backward
                    vel_msg.linear.x = -0.5;
                    vel_msg.angular.z = 0;
                } else if (count < 14) {
                    // Turn left
                    vel_msg.linear.x = 0;
                    vel_msg.angular.z = 2;
                } else {
                    // Restart the states of the variables
                    count = 0;
                    vel_msg.angular.z = 0;
                    is_right_bumper_pressed = false;
                }
            }
        } else {
            // Return to free move if robot has stopped moving after joystick was used,
            // and restart the states of the joystic sensor and count
            if (vel_msg.linear.x == 0 && vel_msg.linear.z == 0 && count++ >= 15) {
                is_joystick_pressed = false;
                count = 0;
            }
        }

        // Celebration
        if (is_celebration) {
            // Turn left with angular velocity = 2 and until count >= 35
            // then stop and restart varibles and sensors
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 2;

            if (count++ >= 35) {
                count = 0;
                is_celebration = false;
            }
        }

        // Arduino
        if (is_arduino_op) {
            // Turn left with angular velocity = 2 and until count >= 10
            // then stop and restart varibles and sensors
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 2;

            if (count++ >= 10) {
                count = 0;
                is_arduino_op = false;
            }
        }

        pub.publish(vel_msg); // Publish vel_msg
        ros::spinOnce();      // Read the topics and called the corresponding function
        loop_rate.sleep();    // Sleep for the necessary time to get a 10 Hz frecuenzy
    }
}
