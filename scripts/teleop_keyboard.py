#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        self.pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.linear_speed = 5
        self.angular_speed = 5

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        rospy.loginfo("w: Forward, s: Backward, a: Left, d: Right, x: Stop, f: Exit")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'w':
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
            elif key == 's':
                self.twist.linear.x = -self.linear_speed
                self.twist.angular.z = 0.0
            elif key == 'a':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_speed
            elif key == 'd':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed
            elif key == 'x':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            elif key == 'f':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                break
            self.pub.publish(self.twist)
            rospy.loginfo(f"Linear: {self.twist.linear.x}, Angular: {self.twist.angular.z}")
            rate.sleep()

if __name__ == "__main__":
    try:
        KeyboardControl().run()
    except rospy.ROSInterruptException:
        pass