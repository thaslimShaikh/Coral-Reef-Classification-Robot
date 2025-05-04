import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopControl(Node):
    def __init__(self):
        super().__init__('teleop_control')
        
        # Publisher for sending velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for periodically calling the publish method
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Initialize a Twist message
        self.velocity_msg = Twist()
        
        # Define linear and angular velocity values
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def publish_velocity(self):
        # Set linear and angular velocities
        self.velocity_msg.linear.x = self.linear_velocity  # Forward/Backward
        self.velocity_msg.linear.y = 0.0                   # No lateral movement
        self.velocity_msg.linear.z = 0.0                   # No vertical movement
        
        self.velocity_msg.angular.x = 0.0                  # No roll
        self.velocity_msg.angular.y = 0.0                  # No pitch
        self.velocity_msg.angular.z = self.angular_velocity  # Yaw (rotation around z-axis)
        
        # Publish the velocity message
        self.publisher_.publish(self.velocity_msg)
        self.get_logger().info(f'Publishing: linear.x = {self.linear_velocity}, angular.z = {self.angular_velocity}')

    def set_velocity(self, linear, angular):
        # Function to set linear and angular velocity values
        self.linear_velocity = linear
        self.angular_velocity = angular

def main(args=None):
    rclpy.init(args=args)

    # Create the TeleopControl node
    teleop_control = TeleopControl()

    # Example: Moving forward and rotating
    teleop_control.set_velocity(1.0, 0.0)  # Forward, no rotation
    rclpy.spin_once(teleop_control)         # Call publish_velocity once

    # Keep the node alive to continuously send commands
    try:
        while rclpy.ok():
            # Example control (change as per your requirement)
            teleop_control.set_velocity(1.0, 0.0)  # Forward
            rclpy.spin_once(teleop_control)
    except KeyboardInterrupt:
        pass

    # Shutdown
    teleop_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
