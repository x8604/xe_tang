#!/usr/bin/env python3

from __future__ import print_function
import threading
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Dieu khien xe:
w - tien
s - lui
a - quay trai
d - quay phai

Dieu khien tay may:
i - nang
k - ha
j - quay trai
l - quay phai

q/z : tang/giam toc do xe 10%
f : thoat chuong trinh
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

jointBindings = {
    'i': (1, 0),
    'k': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.pub_base = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.pub_joint1 = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/joint2_position_controller/command', Float64, queue_size=10)
        
        self.x = 0.0
        self.th = 0.0
        self.joint1_pos = 0.0
        self.joint2_pos = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        self.joint1_min = 0.0
        self.joint1_max = 0.5
        self.joint_step = 0.05

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        while not rospy.is_shutdown() and self.pub_base.get_num_connections() == 0:
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, th, joint1_pos, joint2_pos, speed, turn):
        self.condition.acquire()
        self.x = x
        self.th = th
        self.joint1_pos = max(self.joint1_min, min(joint1_pos, self.joint1_max))
        self.joint2_pos = joint2_pos
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, self.joint1_pos, self.joint2_pos, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist.linear.x = self.x * self.speed
            twist.angular.z = self.th * self.turn
            twist.linear.y = twist.linear.z = twist.angular.x = twist.angular.y = 0

            self.condition.release()

            self.pub_base.publish(twist)
            self.pub_joint1.publish(self.joint1_pos)
            self.pub_joint2.publish(self.joint2_pos)

        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_base.publish(twist)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return None if sys.platform == 'win32' else termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform != 'win32' and old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "Toc do hien tai:\tlinear %s\tangular %s" % (speed, turn)

if __name__ == "__main__":
    settings = saveTerminalSettings()
    rospy.init_node('keyboard_control')

    speed = rospy.get_param("~speed", 10.0)
    turn = rospy.get_param("~turn", 10.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat)
    x = 0
    th = 0
    joint1_pos = 0.0
    joint2_pos = 0.0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, th, joint1_pos, joint2_pos, speed, turn)

        print(msg)
        print(vels(speed, turn))

        while not rospy.is_shutdown():
            key = getKey(settings, key_timeout)

            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in jointBindings:
                joint1_pos += jointBindings[key][0] * pub_thread.joint_step
                joint2_pos += jointBindings[key][1] * pub_thread.joint_step
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == 'f' or key == '\x03':  # 'f' or Ctrl+C
                break
            else:
                if key == '' and x == 0 and th == 0:
                    continue
                x = 0
                th = 0

            pub_thread.update(x, th, joint1_pos, joint2_pos, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)