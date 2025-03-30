#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, tty, termios

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control')
        self.pub_base = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.pub_joint1 = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=10)
        
        self.twist = Twist()
        self.linear_speed = 5.0
        self.angular_speed = 5.0
        
        self.joint1_pos = 0.0 
        self.joint2_pos = 0.0
        self.joint_step = 0.1

        self.joint1_min = 0.0
        self.joint1_max = 0.5
        self.joint2_min = -1.57
        self.joint2_max = 1.57

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def run(self):
        rospy.loginfo("Base: w: Forward, s: Backward, a: Left, d: Right, x: Stop")
        rospy.loginfo("Arm: i: Joint1 Up, k: Joint1 Down, j: Joint2 Left, l: Joint2 Right")
        rospy.loginfo("f: Exit")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.get_key()
            
            # Base control
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
            elif key == 'i':
                self.joint1_pos = self.clamp(self.joint1_pos + self.joint_step, self.joint1_min, self.joint1_max)
            elif key == 'k':
                self.joint1_pos = self.clamp(self.joint1_pos - self.joint_step, self.joint1_min, self.joint1_max)
            elif key == 'j':
                self.joint2_pos = self.clamp(self.joint2_pos - self.joint_step, self.joint2_min, self.joint2_max)
            elif key == 'l':
                self.joint2_pos = self.clamp(self.joint2_pos + self.joint_step, self.joint2_min, self.joint2_max)
            
            # Exit
            elif key == 'f':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub_base.publish(self.twist)
                break
            
            # Publish commands
            self.pub_base.publish(self.twist)
            self.pub_joint1.publish(self.joint1_pos)
            self.pub_joint2.publish(self.joint2_pos)
            
            # Log current state
            rospy.loginfo(f"Base - Linear: {self.twist.linear.x}, Angular: {self.twist.angular.z}")
            rospy.loginfo(f"Arm - Joint1: {self.joint1_pos:.2f}m, Joint2: {self.joint2_pos:.2f}rad")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        KeyboardControl().run()
    except rospy.ROSInterruptException:
        pass