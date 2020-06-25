/*
 * rosserial Publisher
 * Prints " group 6"
 */

 # include < ros.h >
 # include < std_msgs / String.h >

 ros::NodeHandle nh ;

 std_msgs::String str_msg; // State the type of message
 ros::Publisher chatter ("chatter", &str_msg ); // Declaring the publisher .


 const int buttonPin = 2; // Pin 2 is the one going to be used .
 char hello [13] = " group6 "; // String message
 int buttonState = 0;

 void setup() {
   nh.initNode() ;
   nh.advertise( chatter ) ;
   pinMode( buttonPin , INPUT ); // Make pin 2 an input .
 }

 void loop () {
   buttonState = digitalRead( buttonPin ); // Read state of pin
   // Send the message when the state of the switch equals 1.
   
   if ( buttonState == HIGH ) {
     str_msg.data = hello ;
     chatter. ( &str_msg ); // Publish the message .
     nh.spinOnce();
   }
     delay(100) ;
 }
