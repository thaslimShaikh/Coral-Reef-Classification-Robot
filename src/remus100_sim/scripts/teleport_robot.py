import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class RobotTeleporter(Node):
    def __init__(self):
        super().__init__('robot_teleporter')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for Gazebo set_entity_state service...")

        # Use coordinates from get_reef_position.py (Replace with actual values)
        reef_x = 2.0  # Update with detected x
        reef_y = 3.0  # Update with detected y
        reef_z = 0.0  # Update with detected z

        self.teleport_robot(reef_x, reef_y, reef_z)

    def teleport_robot(self, x, y, z):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = "simple_robot"  # Change to your robot's name
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Robot successfully teleported to the coral reef!")
            else:
                self.get_logger().error("Failed to teleport the robot.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleporter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
