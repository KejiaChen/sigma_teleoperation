import rclpy
from rclpy.node import Node
from custom_msgs.srv import LogControl  # Replace with your package name
# print timestamp
from datetime import datetime

class LogControlServer(Node):
    def __init__(self):
        super().__init__('log_control_server')
        self.srv = self.create_service(LogControl, 'log_control', self.handle_request)

    def handle_request(self, request, response):
        if request.command == 'start':
            self.get_logger().info('Received start logging request.')
            response.success = True
            response.message = 'Logging started.'
            # Print timestamp and accurate to the millisecond
            self.get_logger().info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {response.message}")
        elif request.command == 'stop':
            self.get_logger().info('Received stop logging request.')
            response.success = True
            response.message = 'Logging stopped.'
            self.get_logger().info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {response.message}")
        else:
            response.success = False
            response.message = 'Invalid command.'
            self.get_logger().warn(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {response.message}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LogControlServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
