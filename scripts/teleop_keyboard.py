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

        # Store original terminal settings
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def setup_terminal(self):
        # Set terminal to raw mode and disable echo
        new = termios.tcgetattr(self.fd)
        new[3] = new[3] & ~termios.ECHO  # Disable echo
        new[3] = new[3] & ~termios.ICANON  # Disable canonical mode
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
        tty.setraw(self.fd)

    def restore_terminal(self):
        # Restore original terminal settings
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        return sys.stdin.read(1)

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def run(self):
        # Clear terminal screen
        print("\033c", end="")
        
        rospy.loginfo("Xe tăng: w: tiến, s: lùi, a: trái, d: phải, x: dừng")
        rospy.loginfo("Tay máy: i: nâng khớp trượt, k: hạ khớp trượt, j: quay phải khớp xoay, l: quay trái khớp xoay")
        rospy.loginfo("f: thoát")
        
        # Set up terminal settings once
        self.setup_terminal()
        
        rate = rospy.Rate(10)
        try:
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
        finally:
            # Ensure terminal settings are restored even if there's an error
            self.restore_terminal()

if __name__ == "__main__":
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass