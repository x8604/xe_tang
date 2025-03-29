#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

def get_key():
    """Capture a single keypress without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def teleop():
    """Publish Twist messages based on keyboard input."""
    rospy.init_node('teleop_node', anonymous=True)
    pub = rospy.Publisher('/xe_tang/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize Twist message
    twist = Twist()
    rospy.loginfo("Use W/A/S/D to move, Q to quit")

    while not rospy.is_shutdown():
        key = get_key()

        # Reset velocities
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Map keys to movements
        if key == 'w':
            twist.linear.x = 0.5  # Forward
        elif key == 's':
            twist.linear.x = -0.5  # Backward
        elif key == 'a':
            twist.angular.z = 1.0  # Turn left
        elif key == 'd':
            twist.angular.z = -1.0  # Turn right
        elif key == 'q':
            rospy.loginfo("Shutting down teleop node")
            break

        # Publish the Twist message
        pub.publish(twist)
        rate.sleep()

    # Stop the robot when exiting
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass