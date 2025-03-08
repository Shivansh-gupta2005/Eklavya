#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time

class TerminalTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('terminal_teleop')
        
        # Publisher for cmd_vel
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Initial speeds
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.5  # rad/s
        self.linear_increment = 0.05
        self.angular_increment = 0.1
        
        # Max speeds
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        
        # Current movement command
        self.twist = Twist()
        self.running = True
        
        # Instructions for the user
        self.instructions = """
Terminal Teleop Control
-----------------------
Movement Controls:
  w: forward     s: backward
  a: left        d: right
  q: rotate left e: rotate right
  x: stop

Speed Controls:
  i: increase linear speed   k: decrease linear speed
  o: increase angular speed  l: decrease angular speed
  
Other:
  Ctrl+C: quit
  
Current Settings:
  Linear Speed: {:.2f} m/s (max: {:.2f})
  Angular Speed: {:.2f} rad/s (max: {:.2f})
"""

        # Start publishing thread
        self.pub_thread = threading.Thread(target=self.publish_twist)
        self.pub_thread.daemon = True
        self.pub_thread.start()
        
        # Print instructions
        self.print_instructions()

    def publish_twist(self):
        """Thread function that publishes twist messages at a fixed rate"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.running:
            self.pub.publish(self.twist)
            rate.sleep()

    def print_instructions(self):
        """Print control instructions with current settings"""
        print(self.instructions.format(
            self.linear_speed, self.max_linear_speed,
            self.angular_speed, self.max_angular_speed
        ))

    def process_key(self, key):
        """Process a keypress and update the twist message accordingly"""
        if key == 'w':
            # Forward
            self.twist.linear.x = self.linear_speed
            self.twist.linear.y = 0
            self.twist.angular.z = 0
            return "Moving forward"
        elif key == 's':
            # Backward
            self.twist.linear.x = -self.linear_speed
            self.twist.linear.y = 0
            self.twist.angular.z = 0
            return "Moving backward"
        elif key == 'a':
            # Left (strafe)
            self.twist.linear.x = 0
            self.twist.linear.y = self.linear_speed
            self.twist.angular.z = 0
            return "Moving left"
        elif key == 'd':
            # Right (strafe)
            self.twist.linear.x = 0
            self.twist.linear.y = -self.linear_speed
            self.twist.angular.z = 0
            return "Moving right"
        elif key == 'q':
            # Rotate left
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.angular.z = self.angular_speed
            return "Rotating left"
        elif key == 'e':
            # Rotate right
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.angular.z = -self.angular_speed
            return "Rotating right"
        elif key == 'x':
            # Stop
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.angular.z = 0
            return "Stopped"
        elif key == 'i':
            # Increase linear speed
            self.linear_speed = min(self.linear_speed + self.linear_increment, self.max_linear_speed)
            return f"Linear speed increased to {self.linear_speed:.2f} m/s"
        elif key == 'k':
            # Decrease linear speed
            self.linear_speed = max(self.linear_speed - self.linear_increment, 0)
            return f"Linear speed decreased to {self.linear_speed:.2f} m/s"
        elif key == 'o':
            # Increase angular speed
            self.angular_speed = min(self.angular_speed + self.angular_increment, self.max_angular_speed)
            return f"Angular speed increased to {self.angular_speed:.2f} rad/s"
        elif key == 'l':
            # Decrease angular speed
            self.angular_speed = max(self.angular_speed - self.angular_increment, 0)
            return f"Angular speed decreased to {self.angular_speed:.2f} rad/s"
        else:
            return "Unknown command"

    def get_key(self):
        """Get a keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        """Main loop to get keypresses and update movement"""
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # Check for Ctrl+C (ASCII value 3)
                if ord(key) == 3:
                    self.running = False
                    break
                
                message = self.process_key(key)
                # Clear screen and reprint instructions (works in most terminals)
                print("\033c", end="")
                self.print_instructions()
                print(f"Last command: {message}")
                
        except Exception as e:
            print(e)
        finally:
            # Make sure to stop the robot before exiting
            twist = Twist()
            self.pub.publish(twist)
            print("\nExiting teleop. Robot stopped.")

if __name__ == '__main__':
    teleop = TerminalTeleop()
    teleop.run()
