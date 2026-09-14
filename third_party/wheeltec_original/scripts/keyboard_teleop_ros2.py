#!/usr/bin/env python3
# coding=utf-8

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""

# 全向移动模式标志
Omni = 0

# 键值对应移动/转向(横向移动方向)方向
moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

# 键值对应速度增量
speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn}"


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        self.speed = 0.2
        self.turn = 0.5

        self.x = 0
        self.th = 0
        self.count = 0

        self.target_speed = 0.0
        self.target_turn = 0.0
        self.target_horizon_move = 0.0

        self.control_speed = 0.0
        self.control_turn = 0.0
        self.control_horizon_move = 0.0

        self.omni = 0

    def run(self):
        print(msg)
        print(vels(self.speed, self.turn))

        try:
            while rclpy.ok():
                key = getKey(self.settings)

                if key == 'b':
                    self.omni = ~self.omni
                    if self.omni:
                        print("Switch to OmniMode")
                        moveBindings['.'] = (-1, -1)
                        moveBindings['m'] = (-1, 1)
                    else:
                        print("Switch to CommonMode")
                        moveBindings['.'] = (-1, 1)
                        moveBindings['m'] = (-1, -1)

                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    self.count = 0

                elif key in speedBindings.keys():
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]
                    self.count = 0
                    print(vels(self.speed, self.turn))

                elif key == ' ' or key == 'k':
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0.0
                    self.control_turn = 0.0
                    self.control_horizon_move = 0.0

                else:
                    self.count += 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if key == '\x03':
                        break

                self.target_speed = self.speed * self.x
                self.target_turn = self.turn * self.th
                self.target_horizon_move = self.speed * self.th

                if self.target_speed > self.control_speed:
                    self.control_speed = min(self.target_speed, self.control_speed + 0.1)
                elif self.target_speed < self.control_speed:
                    self.control_speed = max(self.target_speed, self.control_speed - 0.1)
                else:
                    self.control_speed = self.target_speed

                if self.target_turn > self.control_turn:
                    self.control_turn = min(self.target_turn, self.control_turn + 0.5)
                elif self.target_turn < self.control_turn:
                    self.control_turn = max(self.target_turn, self.control_turn - 0.5)
                else:
                    self.control_turn = self.target_turn

                if self.target_horizon_move > self.control_horizon_move:
                    self.control_horizon_move = min(
                        self.target_horizon_move, self.control_horizon_move + 0.1
                    )
                elif self.target_horizon_move < self.control_horizon_move:
                    self.control_horizon_move = max(
                        self.target_horizon_move, self.control_horizon_move - 0.1
                    )
                else:
                    self.control_horizon_move = self.target_horizon_move

                twist = Twist()
                if self.omni == 0:
                    twist.linear.x = self.control_speed
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = self.control_turn
                else:
                    twist.linear.x = self.control_speed
                    twist.linear.y = self.control_horizon_move
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0

                self.pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()