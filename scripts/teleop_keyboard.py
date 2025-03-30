#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys
import tty
import termios

class KeyboardControl:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('keyboard_control', anonymous=True)

        # Publisher cho các lệnh vận tốc bánh xe
        self.pub_left = rospy.Publisher('/banh_4_joint/command', Float64, queue_size=10)
        self.pub_right = rospy.Publisher('/banh_2_joint/command', Float64, queue_size=10)

        # Tốc độ giới hạn
        self.max_speed = 5.0  # Giới hạn vận tốc tối đa (radian/s)
        self.min_speed = -5.0  # Giới hạn vận tốc tối thiểu (radian/s)
        self.speed_step = 0.5  # Mức tăng/giảm tốc độ

        # Vận tốc ban đầu
        self.linear_speed = 0.0  # Tốc độ tiến/lùi
        self.angular_speed = 0.0  # Tốc độ quay

    def get_key(self):
        """Đọc phím từ bàn phím mà không cần nhấn Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        rospy.loginfo("Điều khiển robot bằng bàn phím:")
        rospy.loginfo("w: Tiến | s: Lùi | a: Quay trái | d: Quay phải")
        rospy.loginfo("q/z: Tăng/Giảm tốc độ tiến/lùi")
        rospy.loginfo("e/c: Tăng/Giảm tốc độ quay")
        rospy.loginfo("x: Dừng | f: Dừng và thoát")

        while not rospy.is_shutdown():
            key = self.get_key()

            # Điều khiển tốc độ tuyến tính
            if key == 'w':  # Đi thẳng
                left_speed = -self.linear_speed
                right_speed = -self.linear_speed
            elif key == 's':  # Đi lùi
                left_speed = self.linear_speed
                right_speed = self.linear_speed
            elif key == 'a':  # Quay trái
                left_speed = self.angular_speed
                right_speed = -self.angular_speed
            elif key == 'd':  # Quay phải
                left_speed = -self.angular_speed
                right_speed = self.angular_speed
            elif key == 'x':  # Dừng
                left_speed = 0.0
                right_speed = 0.0
            elif key == 'f':  # Dừng và thoát
                left_speed = 0.0
                right_speed = 0.0
                self.pub_left.publish(left_speed)
                self.pub_right.publish(right_speed)
                rospy.loginfo("Dừng robot và thoát")
                break
            elif key == 'q':  # Tăng tốc độ tiến/lùi
                self.linear_speed = min(self.linear_speed + self.speed_step, self.max_speed)
                rospy.loginfo(f"Tốc độ tiến/lùi: {self.linear_speed:.2f}")
                continue  # Không cần gửi lệnh vận tốc vì không thay đổi trạng thái di chuyển
            elif key == 'z':  # Giảm tốc độ tiến/lùi
                self.linear_speed = max(self.linear_speed - self.speed_step, 0.0)
                rospy.loginfo(f"Tốc độ tiến/lùi: {self.linear_speed:.2f}")
                continue
            elif key == 'e':  # Tăng tốc độ quay
                self.angular_speed = min(self.angular_speed + self.speed_step, self.max_speed)
                rospy.loginfo(f"Tốc độ quay: {self.angular_speed:.2f}")
                continue
            elif key == 'c':  # Giảm tốc độ quay
                self.angular_speed = max(self.angular_speed - self.speed_step, 0.0)
                rospy.loginfo(f"Tốc độ quay: {self.angular_speed:.2f}")
                continue
            else:
                continue  # Nếu không nhấn phím hợp lệ, bỏ qua vòng lặp này

            # Gửi lệnh vận tốc
            self.pub_left.publish(left_speed)
            self.pub_right.publish(right_speed)

            # Hiển thị trạng thái
            rospy.loginfo(f"Left: {left_speed:.2f}, Right: {right_speed:.2f}")

            rospy.sleep(0.1)  # Tránh đọc phím quá nhanh

if __name__ == "__main__":
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
