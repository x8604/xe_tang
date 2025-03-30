#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys
import select
import tty
import termios

# Biến toàn cục để lưu vận tốc của bánh trái và bánh phải
left_wheel_vel = 0.0
right_wheel_vel = 0.0

# Hàm kiểm tra xem có phím nào được nhấn không
def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# Hàm điều khiển xe dựa trên phím nhấn
def process_key(key):
    global left_wheel_vel, right_wheel_vel
    if key == 'w':  # Tiến lên
        left_wheel_vel = 5.0
        right_wheel_vel = 5.0
    elif key == 's':  # Lùi lại
        left_wheel_vel = -5.0
        right_wheel_vel = -5.0
    elif key == 'a':  # Rẽ trái
        left_wheel_vel = -3.0
        right_wheel_vel = 3.0
    elif key == 'd':  # Rẽ phải
        left_wheel_vel = 3.0
        right_wheel_vel = -3.0
    elif key == 'q':  # Dừng lại
        left_wheel_vel = 0.0
        right_wheel_vel = 0.0
    elif key == '\x03':  # Ctrl+C để thoát
        rospy.signal_shutdown("User requested shutdown")

# Hàm chính khởi tạo node ROS và publish vận tốc
def teleop_robot():
    # Khởi tạo node ROS
    rospy.init_node('keyboard_teleop', anonymous=True)
    
    # Tạo publisher cho hai bánh xe
    pub_left = rospy.Publisher('/xe_tang/than_xe_banh_4/command', Float64, queue_size=10)
    pub_right = rospy.Publisher('/xe_tang/than_xe_banh_2/command', Float64, queue_size=10)
    
    # Tần số publish (10 Hz)
    rate = rospy.Rate(10)
    
    # Lưu cấu hình terminal ban đầu
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Chuyển terminal sang chế độ không đợi Enter
        tty.setcbreak(sys.stdin.fileno())
        
        print("Điều khiển xe bằng bàn phím:")
        print("w: Tiến lên | s: Lùi lại | a: Rẽ trái | d: Rẽ phải | q: Dừng | Ctrl+C: Thoát")
        
        # Vòng lặp ROS
        while not rospy.is_shutdown():
            if is_data():
                key = sys.stdin.read(1)
                process_key(key)
            
            # Publish vận tốc cho bánh trái và bánh phải
            pub_left.publish(left_wheel_vel)
            pub_right.publish(right_wheel_vel)
            
            # Duy trì tần số
            rate.sleep()
    
    finally:
        # Khôi phục cấu hình terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        teleop_robot()
    except rospy.ROSInterruptException:
        pass