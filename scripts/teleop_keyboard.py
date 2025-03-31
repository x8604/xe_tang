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
        self.linear_speed = 10.0
        self.angular_speed = 10.0
        
        self.joint1_pos = 0.0 
        self.joint2_pos = 0.0
        self.joint_step = 0.05

        self.joint1_min = 0
        self.joint1_max = 0.5

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
        rospy.loginfo("Xe tăng: w: tiến, s: lùi, a: trái, d: phải, x: dừng")
        rospy.loginfo("Tay máy: i: nâng khớp trượt, k: hạ khớp trượt, j: quay phải khớp xoay, l: quay trái khớp xoay")
        rospy.loginfo("f: thoát")
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
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            if key == 'i':
                self.joint1_pos = self.clamp(self.joint1_pos + self.joint_step, self.joint1_min, self.joint1_max)
            elif key == 'k':
                self.joint1_pos = self.clamp(self.joint1_pos - self.joint_step, self.joint1_min, self.joint1_max)
            elif key == 'j':
                self.joint2_pos -= self.joint_step
            elif key == 'l':
                self.joint2_pos += self.joint_step
            
            elif key == 'f':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub_base.publish(self.twist)
                break
            
            self.pub_base.publish(self.twist)
            self.pub_joint1.publish(self.joint1_pos)
            self.pub_joint2.publish(self.joint2_pos)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        KeyboardControl().run()
    except rospy.ROSInterruptException:
        pass