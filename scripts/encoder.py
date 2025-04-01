#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    # In vị trí của các khớp (mô phỏng encoder)
    for i, name in enumerate(msg.name):
        position = msg.position[i]  # Vị trí (radian)
        velocity = msg.velocity[i]  # Vận tốc (rad/s)
        rospy.loginfo(f"Joint: {name}, Position: {position:.3f} rad, Velocity: {velocity:.3f} rad/s")

def listener():
    # Khởi tạo node
    rospy.init_node('encoder_listener')
    
    # Đăng ký subscriber để lắng nghe /joint_states
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    
    # Giữ node chạy
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass