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
linear_x_speed = 0.0  # m/s
linear_y_speed = 0.0  # m/s
angular_z_speed = 0.0 # rad/s
speed_increment = 0.1

# Print control instructions
def print_instructions():
    msg = """
Omni Robot Teleop Control

Motion Controls:
    w/s: Increase/decrease linear X (forward/backward)
    a/d: Increase/decrease linear Y (left/right)
    q/e: Increase/decrease angular Z (rotation)

Other Controls:
    space: Stop
    i: Print current settings
    CTRL+C: Quit
    
Current Settings:
    Linear X Speed: {:.2f} m/s
    Linear Y Speed: {:.2f} m/s
    Angular Z Speed: {:.2f} rad/s
    """.format(linear_x_speed, linear_y_speed, angular_z_speed)
    print(msg)

# Main function
def teleop():
    global linear_x_speed, linear_y_speed, angular_z_speed
    
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
        
        # Process key input for direct control of twist values
        if key == 'w':  # Increase linear X
            linear_x_speed += speed_increment
            print("Linear X speed: {:.2f} m/s".format(linear_x_speed))
            twist.linear.x = linear_x_speed
        elif key == 's':  # Decrease linear X
            linear_x_speed -= speed_increment
            print("Linear X speed: {:.2f} m/s".format(linear_x_speed))
            twist.linear.x = linear_x_speed
        elif key == 'a':  # Increase linear Y
            linear_y_speed += speed_increment
            print("Linear Y speed: {:.2f} m/s".format(linear_y_speed))
            twist.linear.y = linear_y_speed
        elif key == 'd':  # Decrease linear Y
            linear_y_speed -= speed_increment
            print("Linear Y speed: {:.2f} m/s".format(linear_y_speed))
            twist.linear.y = linear_y_speed
        elif key == 'q':  # Increase angular Z
            angular_z_speed += speed_increment
            print("Angular Z speed: {:.2f} rad/s".format(angular_z_speed))
            twist.angular.z = angular_z_speed
        elif key == 'e':  # Decrease angular Z
            angular_z_speed -= speed_increment
            print("Angular Z speed: {:.2f} rad/s".format(angular_z_speed))
            twist.angular.z = angular_z_speed
        elif key == ' ':  # Stop
            linear_x_speed = 0.0
            linear_y_speed = 0.0
            angular_z_speed = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            print("Stopped")
        elif key == 'i':  # Print current settings
            print_instructions()
            continue
        else:
            continue  # Unknown key, just continue
        
        # Publish velocity command
        pub.publish(twist)
        rate.sleep()

if __name__ == "__main__":
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
