#!/usr/bin/env python3

"""
Code điều khiển robot trong Gazebo bằng bàn phím thông qua plugin libgazebo_ros_diff_drive.so
sử dụng thư viện pynput để bắt sự kiện phím một cách hiệu quả.

Cách sử dụng:
- Phím mũi tên: điều khiển chuyển động của robot
  + MŨI TÊN LÊN/XUỐNG: tiến/lùi
  + MŨI TÊN TRÁI/PHẢI: rẽ trái/phải
- Phím w/s: tăng/giảm tốc độ tuyến tính
- Phím a/d: tăng/giảm tốc độ góc
- Phím Esc: thoát

Robot sẽ dừng lại khi không có phím mũi tên nào được nhấn.
"""

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import numpy as np
import sys

# Giá trị tốc độ
MIN_VEL = np.zeros(2)  # [tuyến tính, góc]
MAX_VEL = np.array([1.0, 3.14])  # [tuyến tính, góc]
LIN_INCREMENT = np.array([0.02, 0.0])  # Mức tăng tốc độ tuyến tính
ANG_INCREMENT = np.array([0.0, 0.1])  # Mức tăng tốc độ góc

# Biến toàn cục để theo dõi trạng thái phím
vel = np.array([0.5, 1.0])  # Tốc độ mặc định [tuyến tính, góc]
CHARS = set()  # Tập hợp các phím chữ đang được nhấn
SPECIALS = set()  # Tập hợp các phím đặc biệt đang được nhấn

# Định nghĩa hành động cho các phím chữ
CHAR_ACTIONS = {
    "w": lambda vel: np.minimum(vel + LIN_INCREMENT, MAX_VEL),
    "s": lambda vel: np.maximum(vel - LIN_INCREMENT, MIN_VEL),
    "a": lambda vel: np.minimum(vel + ANG_INCREMENT, MAX_VEL),
    "d": lambda vel: np.maximum(vel - ANG_INCREMENT, MIN_VEL),
}

# Định nghĩa hành động cho các phím đặc biệt
SPECIALS_ACTIONS = {
    keyboard.Key.up: lambda return_vel, vel: return_vel + np.array([1.0, 0]) * vel,
    keyboard.Key.down: lambda return_vel, vel: return_vel + np.array([-1.0, 0]) * vel,
    keyboard.Key.left: lambda return_vel, vel: return_vel + np.array([0.0, 1.0]) * vel,
    keyboard.Key.right: lambda return_vel, vel: return_vel + np.array([0.0, -1.0]) * vel,
}

def clear_line():
    """Xóa dòng hiện tại trên stdout để phím đã nhấn không hiển thị"""
    sys.stdout.write("\r" + " " * 80 + "\r")
    sys.stdout.flush()

def on_press(key):
    """Xử lý sự kiện phím được nhấn"""
    global CHARS, SPECIALS
    
    # Kiểm tra nếu là phím Esc để thoát
    if key == keyboard.Key.esc:
        return False  # Dừng listener
    
    # Xử lý các phím đặc biệt (mũi tên)
    if isinstance(key, keyboard.Key) and key in SPECIALS_ACTIONS:
        SPECIALS.add(key)
    
    # Xử lý các phím chữ (w, s, a, d)
    elif isinstance(key, keyboard.KeyCode) and hasattr(key, 'char') and key.char in CHAR_ACTIONS:
        CHARS.add(key.char)
    
    # Cập nhật và gửi vận tốc
    update_velocity()
    return True

def on_release(key):
    """Xử lý sự kiện phím được thả ra"""
    global CHARS, SPECIALS
    
    # Xử lý các phím đặc biệt (mũi tên)
    if isinstance(key, keyboard.Key) and key in SPECIALS_ACTIONS:
        SPECIALS.discard(key)
    
    # Xử lý các phím chữ (w, s, a, d)
    elif isinstance(key, keyboard.KeyCode) and hasattr(key, 'char') and key.char in CHAR_ACTIONS:
        CHARS.discard(key.char)
    
    # Cập nhật và gửi vận tốc
    update_velocity()
    return True

def update_velocity():
    """Cập nhật vận tốc dựa trên phím được nhấn và gửi lệnh điều khiển"""
    global vel, cmd_vel_pub
    
    # Cập nhật giá trị tốc độ dựa trên các phím chữ (w, s, a, d)
    for char, action_func in CHAR_ACTIONS.items():
        if char in CHARS:
            vel = action_func(vel)
    
    # Tính toán vận tốc dựa trên các phím mũi tên
    return_vel = np.zeros(2)
    for special, action_func in SPECIALS_ACTIONS.items():
        if special in SPECIALS:
            return_vel = action_func(return_vel, vel)
    
    # Hiển thị thông tin
    if CHARS or SPECIALS:
        print(f"Tốc độ: {vel} | Vận tốc áp dụng: {return_vel}")
    
    # Tạo và gửi thông điệp Twist
    msg = Twist()
    msg.linear.x = return_vel[0]
    msg.angular.z = return_vel[1]
    cmd_vel_pub.publish(msg)

def display_instructions():
    """Hiển thị hướng dẫn sử dụng"""
    print("""
HƯỚNG DẪN ĐIỀU KHIỂN ROBOT
---------------------------
Điều khiển chuyển động:
  MŨI TÊN LÊN/XUỐNG: tiến/lùi
  MŨI TÊN TRÁI/PHẢI: rẽ trái/phải
  
Điều chỉnh tốc độ:
  W/S: tăng/giảm tốc độ tuyến tính
  A/D: tăng/giảm tốc độ góc
  
Kết thúc:
  ESC: thoát chương trình
  
Robot sẽ dừng lại khi không có phím mũi tên nào được nhấn.
---------------------------
""")

if __name__ == "__main__":
    # Khởi tạo node ROS
    rospy.init_node('robot_teleop_keyboard_pynput')
    
    # Tạo publisher để gửi lệnh điều khiển
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=5)
    
    # Hiển thị hướng dẫn
    display_instructions()
    
    # Khởi động listener bắt sự kiện bàn phím
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        try:
            rospy.loginfo(f"Đã khởi động điều khiển bàn phím trên topic: {cmd_vel_topic}")
            
            # Vòng lặp chờ cho đến khi có tín hiệu shutdown
            rate = rospy.Rate(10)  # 10Hz
            while not rospy.is_shutdown() and listener.running:
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        
        finally:
            # Dừng robot khi thoát
            stop_msg = Twist()
            cmd_vel_pub.publish(stop_msg)
            rospy.loginfo("Đã dừng robot và thoát chương trình")