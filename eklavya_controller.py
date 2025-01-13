#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import tty
import termios

def getKey():
    """Get a single keypress from the user"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def robotController():
    pub = rospy.Publisher('robot_command', String, queue_size=10)
    rospy.init_node('robot_controller', anonymous=True)
    
    print("""
    Control Your Robot!
    ------------------
    w : move forward
    s : move backward
    a : move left
    d : move right
    q : spin left
    e : spin right
    g : diagonal forward
    h : diagonal backward
    x : stop
    
    CTRL-C to quit
    """)

    while not rospy.is_shutdown():
        key = getKey()
        
        if key in ['w', 'a', 's', 'd', 'q', 'e', 'g','h','x']:
            cmd = String()
            cmd.data = key
            pub.publish(cmd)
            
        if key == '\x03':  # CTRL-C
            break

if __name__ == "__main__":
    try:
        robotController()
    except rospy.ROSInterruptException:
        pass
