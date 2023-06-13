/*
 * rosserial Publisher
 * Prints " group 6 "
 */

 # include <ros.h>
 # include <std_msgs/String.h>

 ros::NodeHandle nh;

 std_msgs::String str_msg; // Message to transmit
 ros::Publisher chatter("chatter", &str_msg);

 const int buttonPin = 2;   // Control pin
 char hello[] = " group6 "; // Output message

 void setup() {
   nh.initNode();
   nh.advertise(chatter);
   pinMode(buttonPin , INPUT);
 }

 void loop () {
   // Send the message if the control button was pressed 
   if (digitalRead(buttonPin) == HIGH) {
     str_msg.data = hello;
     chatter.publish(&str_msg); // Publish the message
     nh.spinOnce();
   }
     delay(100);
 }
