import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('submarine_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Submarine Teleop Node Started. Use arrow keys to move. Press 'q' to quit.")
        self.control_loop()

    def control_loop(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Forward
                twist.linear.x = 1.0
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = -1.0
                twist.angular.z = 0.0
            elif key == 'a':  # Turn Left
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            elif key == 'd':  # Turn Right
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            elif key == 'q':  # Quit
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher.publish(twist)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
