import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState

class CoralReefLocator(Node):
    def __init__(self):
        super().__init__('coral_reef_locator')
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for Gazebo get_entity_state service...")

        self.request_position()

    def request_position(self):
        request = GetEntityState.Request()
        request.name = "coral_reef"  # Change this if your model name is different
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Coral Reef Position: x={response.state.pose.position.x}, "
                                       f"y={response.state.pose.position.y}, z={response.state.pose.position.z}")
            else:
                self.get_logger().error("Failed to get coral reef position. Check model name.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CoralReefLocator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
