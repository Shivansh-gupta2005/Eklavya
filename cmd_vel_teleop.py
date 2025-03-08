#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import signal
import time

# Terminal input settings
def getKey():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Velocity settings
linear_speed = 0.5  # m/s
angular_speed = 1.0  # rad/s
linear_increment = 0.1
angular_increment = 0.1

# Print control instructions
def print_instructions():
    msg = """
Mecanum Robot Teleop Control

Motion Controls:
    w: Forward          s: Backward
    a: Strafe Left      d: Strafe Right
    q: Rotate Left      e: Rotate Right
    z: Forward-Left     c: Forward-Right
    x: Backward-Left    v: Backward-Right

Speed Controls:
    t/g: Increase/decrease linear speed
    y/h: Increase/decrease angular speed

Other Controls:
    space: Stop
    i: Print current settings
    CTRL+C: Quit
    
Current Settings:
    Linear Speed: {:.2f} m/s
    Angular Speed: {:.2f} rad/s
    """.format(linear_speed, angular_speed)
    print(msg)

# Main function
def teleop():
    global linear_speed, angular_speed
    
    # Initialize ROS node
    rospy.init_node('teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Initialize Twist message
    twist = Twist()
    
    # Handle CTRL+C gracefully
    def signal_handler(sig, frame):
        twist = Twist()  # Zero velocity
        pub.publish(twist)
        print("\nExiting...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print_instructions()
    
    while not rospy.is_shutdown():
        key = getKey()
        
        # Process key input
        if key == 'w':  # Forward
            twist.linear.x = linear_speed
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        elif key == 's':  # Backward
            twist.linear.x = -linear_speed
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        elif key == 'a':  # Strafe left
            twist.linear.x = 0.0
            twist.linear.y = linear_speed
            twist.angular.z = 0.0
        elif key == 'd':  # Strafe right
            twist.linear.x = 0.0
            twist.linear.y = -linear_speed
            twist.angular.z = 0.0
        elif key == 'q':  # Rotate left
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = angular_speed
        elif key == 'e':  # Rotate right
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = -angular_speed
        elif key == 'z':  # Forward-left diagonal
            twist.linear.x = linear_speed
            twist.linear.y = linear_speed
            twist.angular.z = 0.0
        elif key == 'c':  # Forward-right diagonal
            twist.linear.x = linear_speed
            twist.linear.y = -linear_speed
            twist.angular.z = 0.0
        elif key == 'x':  # Backward-left diagonal
            twist.linear.x = -linear_speed
            twist.linear.y = linear_speed
            twist.angular.z = 0.0
        elif key == 'v':  # Backward-right diagonal
            twist.linear.x = -linear_speed
            twist.linear.y = -linear_speed
            twist.angular.z = 0.0
        elif key == ' ':  # Stop
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        elif key == 't':  # Increase linear speed
            linear_speed += linear_increment
            print("Linear speed: {:.2f} m/s".format(linear_speed))
            continue
        elif key == 'g':  # Decrease linear speed
            linear_speed = max(0, linear_speed - linear_increment)
            print("Linear speed: {:.2f} m/s".format(linear_speed))
            continue
        elif key == 'y':  # Increase angular speed
            angular_speed += angular_increment
            print("Angular speed: {:.2f} rad/s".format(angular_speed))
            continue
        elif key == 'h':  # Decrease angular speed
            angular_speed = max(0, angular_speed - angular_increment)
            print("Angular speed: {:.2f} rad/s".format(angular_speed))
            continue
        elif key == 'i':  # Print current settings
            print_instructions()
            continue
        else:
            continue  # Unknown key, just continue
        
        # Publish velocity command
        pub.publish(twist)
        
        # Brief pause to allow user to see effect
        time.sleep(0.1)
        
        # Auto-stop after each command for safety (optional)
        # Uncomment these lines if you want the robot to stop after each key press
        # twist = Twist()  # Zero velocity
        # pub.publish(twist)
        
        rate.sleep()

if __name__ == "__main__":
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
