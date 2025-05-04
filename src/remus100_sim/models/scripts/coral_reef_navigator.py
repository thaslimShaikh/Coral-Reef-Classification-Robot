#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class CoralReefNavigator(Node):
    def __init__(self):
        super().__init__('coral_reef_navigator')
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_robot/cmd_vel', 10)
        self.front_camera_sub = self.create_subscription(
            Image, '/simple_robot/camera/front_image_raw', self.front_camera_callback, 10)
        self.down_camera_sub = self.create_subscription(
            Image, '/simple_robot/camera/down_image_raw', self.down_camera_callback, 10)
        self.sonar_sub = self.create_subscription(
            LaserScan, '/simple_robot/sonar/scan', self.sonar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/simple_robot/imu/data', self.imu_callback, 10)

        self.bridge = CvBridge()
        self.snapshot_taken = False
        self.coral_detected = False
        self.depth = 0.0

    def front_camera_callback(self, msg):
        # Process front camera image to detect coral reef
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Simple color-based detection (e.g., detect brown/yellow coral)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_coral = np.array([10, 50, 50])
        upper_coral = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_coral, upper_coral)
        
        if cv2.countNonZero(mask) > 1000:  # Threshold for coral detection
            self.coral_detected = True
            self.get_logger().info("Coral reef detected ahead!")
            self.move_towards_coral()
        else:
            self.coral_detected = False
            self.search_coral()

    def down_camera_callback(self, msg):
        # Take snapshot when close to coral
        if self.coral_detected and self.depth < 2.0 and not self.snapshot_taken:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('/home/thaslim/ros2_ws/src/remus100_sim/snapshot.jpg', frame)
            self.get_logger().info("Snapshot taken!")
            self.snapshot_taken = True

    def sonar_callback(self, msg):
        # Use sonar to maintain safe distance from the reef
        self.depth = min(msg.ranges)
        if self.depth < 1.0:
            self.stop_robot()
            self.get_logger().info("Too close to obstacle, stopping.")

    def imu_callback(self, msg):
        # Use IMU for orientation (optional)
        pass

    def move_towards_coral(self):
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.linear.z = -0.2  # Slight descent
        self.cmd_vel_pub.publish(cmd)

    def search_coral(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Slow forward
        cmd.angular.z = 0.2  # Rotate to search
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = CoralReefNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
