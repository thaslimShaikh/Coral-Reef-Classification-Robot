#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tkinter import *

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel_msg = Twist()

    def move_forward(self):
        self.vel_msg.linear.x = 1.0  # Adjust forward speed
        self.publisher_.publish(self.vel_msg)

    def move_backward(self):
        self.vel_msg.linear.x = -1.0  # Adjust backward speed
        self.publisher_.publish(self.vel_msg)

    def move_left(self):
        self.vel_msg.angular.z = 1.0  # Turn left
        self.publisher_.publish(self.vel_msg)

    def move_right(self):
        self.vel_msg.angular.z = -1.0  # Turn right
        self.publisher_.publish(self.vel_msg)

def gui():
    root = Tk()
    root.title("Robot Controller")

    robot_controller = RobotController()

    def stop():
        robot_controller.vel_msg = Twist()  # Stop the robot
        robot_controller.publisher_.publish(robot_controller.vel_msg)

    Button(root, text="Move Forward", command=robot_controller.move_forward).pack()
    Button(root, text="Move Backward", command=robot_controller.move_backward).pack()
    Button(root, text="Move Left", command=robot_controller.move_left).pack()
    Button(root, text="Move Right", command=robot_controller.move_right).pack()
    Button(root, text="Stop", command=stop).pack()

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

