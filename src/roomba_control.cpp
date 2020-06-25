# include "ros/ros.h"
# include "geometry_msgs/Twist.h" // Express linear and angular velocity
# include "sensor_msgs/Joy.h" // Express the state of the joystick
# include "ca_msgs/Bumper.h" // Express the status on the bump sensors
# include "std_msgs/String.h" // Allows to recieve a " string " type message
# include "iostream "
using namespace std ;

 geometry_msgs::Twist vel ; // Create variable " vel " to send the velocity data .
 int left_sen =0; // Left bump sensor
 int right_sen =0; // Right bump sensor
 int joy_sen = 0; // Joystick sensor
 int ard_sen = 0; // Arduino sensor
 int celeb = 0; // Celebration sensor

 void TeleopJoy_callBack ( const sensor_msgs::Joy::ConstPtr& joy )
 {
   joy_sen = 1;
   vel.angular.z = joy - > axes [0]; // Move with angular velocity according to axes

   vel.linear.x = joy -> axes [1]; // Move with linear velocity according to axes 1.

   // Celebration
   if ( joy -> buttons [0]==1) {
   celeb = 1; }
 }

 void BumperSensor_callBack ( const ca_msgs::Bumper::ConstPtr& bumper )
 {
   // Detects the left bump sensor
   if ( bumper -> is_left_pressed ==1) {
   left_sen =1; }

   // Detects the right bump sensor
   if ( bumper -> is_right_pressed ==1) {
   right_sen =1; }
 }

 void arduino_callBack ( const std_msgs::String::ConstPtr& msg )
 {
   ard_sen = 1;
 }

int main ( int argc , char ** argv )
 {
   ros::init ( argc , argv ," teleopJoy ");
   ros::NodeHandle n;
   ros::Publisher pub ;
   ros::Subscriber sub1 ; // joystick
   ros::Subscriber sub2 ; // bumper sensor
   ros::Subscriber sub3 ; // Arduino

   pub = n.advertise < geometry_msgs::Twist >("cmd_vel", 1) ;
   sub1 = n.subscribe < sensor_msgs::Joy >("joy", 10 , &TeleopJoy_callBack );
   sub2 = n.subscribe < ca_msgs::Bumper >("bumper" ,10 , &BumperSensor_callBack );
   sub3 = n.subscribe < std_msgs::String >("chatter" ,10 , &arduino_callBack ) ;

 ros::Rate loop_rate (10) ; // Set the frequency , to send the data , to 10 Hz.
 int count = 0;
 while ( ros::ok () ) { // Stop the node if Ctrl + c is pressed or if ROS stops all the nodes

   if( joy_sen == 0) {
     vel.linear.x= 0.5;
     vel.angular.z = 0;

     // Right turn
     if( left_sen ==1) {
       ++ count ;
    
    // Move backward
     if( count <10) {
       vel.linear.x = -0.5;
       vel.angular.z = 0; }
     
     // Turn right
     else if( count >=10 && count < 14) {
       vel.linear.x = 0;
       vel.angular.z = -2; }
     
     // Restart the states of the variables
     else if( count >=14) {
        left_sen = 0;
        count = 0;
        vel.angular.z = 0; }
     }
     
     // Left turn
     if( right_sen ==1) {
        ++ count ;
     // Move backward
     if( count <10) {
        vel.linear.x = -0.5;
        vel.angular.z = 0; }
     
     // Turn left
     else if( count >=10 && count < 14) {
        vel.linear.x = 0;
        vel.angular.z = 2; }

    // Restart the states of the variables
     else if( count >=14) {
        right_sen = 0;
        count = 0;
        vel.angular.z = 0; }
     }
   }

  // Return to free move if robot has stopped moving after joystick was used ,
  // and restart the states of the joystic sensor and count
   else {
     ++ count ;
     if( vel.linear.x == 0 && vel.linear.z == 0 && count >=15) {
      joy_sen = 0;
      count = 0; }
   }

 // Celebration
    if ( celeb ==1) {
      ++ count ;
      // Turn left with angular velocity = 2 and until count >=35 , then stop and clear varibles and sensors .
      vel.linear.x = 0;
      vel.angular.z = 2;
      if ( count >= 35) {
        count = 0;
        celeb = 0; }
    }

 // Arduino
    if ( ard_sen ==1) {
      ++ count ;
      // Turn left with angular velocity = 2 and until count >=10 , then stop and varibles and sensors .
      vel.linear.x = 0;
      vel.angular.z = 2;
      if ( count >= 10) {
        count = 0;
        ard_sen = 0; }
    }

  pub.publish( vel ); // Publish value of variable " vel "
  ros::spinOnce() ; // Read the topics and called the corresponding function
  loop_rate.sleep() ; // Sleep for the necessary time to get a 10 Hz frecuenzy
  }
}
