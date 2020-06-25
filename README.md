# Mobile Robot Control and Teleoperation with ROS

<p align="center">
  <img width="300" src="https://secure.img1-fg.wfcdn.com/im/53613036/resize-h800-w800%5Ecompr-r85/8436/84360235/iRobot%25AE+Roomba%25AE+e5+%25285150%2529+Wi-Fi%25AE+Connected+Robot+Vacuum.jpg">
</p>

## Skills and tools used

> ROS | C++ | Linux

## Summary 

Robot Operating System (ROS) is a powerful software tool to control robots. This project uses ROS to create a software package that can control an iRobot to complete a set of tasks as noted below.

1. Move freely in the given indoor environment.
2. Using the left and right bump sensors, turn around whenit touches an obstacle.
3. Turn around when an external switch is pressed.
4. Respond to joystick input for manual control.
5. Do a celebration ceremony motion triggered by one of the joystick buttons.

## SOME OF THE ROS PACKAGES AND LIBRARIES USED

* `create_autonomy:` This is a package that contains the necessary drivers for the iRobot. This driver contains the geometry_msgs/Twist subscriber that controls the robot's wheels according to a forward and angular velocity.

* `joy:` The joy library contains a node called "joy_node". This node publishes a message with the current state of the joystick's buttons and axes.
