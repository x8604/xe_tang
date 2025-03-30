#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control')

        self.pub_cmd_vel = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)

        self.max_linear_speed = 10.0
        self.max_angular_speed = 10.0
        self.speed_step = 0.5

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.twist = Twist()

    def get_key(self):
        """Read a key from the keyboard without requiring Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        rospy.loginfo("Control robot with keyboard:")
        rospy.loginfo("w: Forward | s: Backward | a: Rotate left | d: Rotate right")
        rospy.loginfo("q/z: Increase/Decrease linear speed")
        rospy.loginfo("e/c: Increase/Decrease angular speed")
        rospy.loginfo("x: Stop | f: Stop and exit")

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
                self.pub_cmd_vel.publish(self.twist)
                rospy.loginfo("Stopping robot and exiting")
                break
            elif key == 'q':
                self.linear_speed = min(self.linear_speed + self.speed_step, self.max_linear_speed)
                rospy.loginfo(f"Linear speed: {self.linear_speed:.2f} m/s")
                continue
            elif key == 'z':
                self.linear_speed = max(self.linear_speed - self.speed_step, 0.0)
                rospy.loginfo(f"Linear speed: {self.linear_speed:.2f} m/s")
                continue
            elif key == 'e':
                self.angular_speed = min(self.angular_speed + self.speed_step, self.max_angular_speed)
                rospy.loginfo(f"Angular speed: {self.angular_speed:.2f} rad/s")
                continue
            elif key == 'c':
                self.angular_speed = max(self.angular_speed - self.speed_step, 0.0)
                rospy.loginfo(f"Angular speed: {self.angular_speed:.2f} rad/s")
                continue
            else:
                continue

            self.pub_cmd_vel.publish(self.twist)

            rospy.loginfo(f"Linear: {self.twist.linear.x:.2f} m/s, Angular: {self.twist.angular.z:.2f} rad/s")

            rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass