
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from custom_msgs.srv import LogControl  # Replace with your package name
from datetime import datetime
import os
os.environ['TZ'] = 'Europe/Berlin'  # Set to your desired timezone
import time
time.tzset()

# Import listener classes
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../joint_state_collection'))
from .ee_position_listener import EEPositionLogger
from .ee_velocity_listener import EEVelocityLogger
from .joint_states_listener import JointStateLogger
from .joint_velocity_cmd_listener import JointVelocityCmdLogger
from .ee_pos_cmd_listener import EEPoseCmdLogger
from .ee_velocity_cmd_listener import EEVelocityCmdLogger


class LogControlServer(Node):
    def __init__(self, listeners):
        super().__init__('log_control_server')
        self.srv = self.create_service(LogControl, 'log_control_right', self.handle_request)
        self.listeners = listeners  # List of listener node instances

    def handle_request(self, request, response):
        if request.command == 'start':
            self.get_logger().info('Received start logging request.')
            for listener in self.listeners:
                if hasattr(listener, 'start_logging'):
                    listener.start_logging()
            response.success = True
            response.message = 'Logging started.'
            self.get_logger().info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {response.message}")
        elif request.command == 'stop':
            self.get_logger().info('Received stop logging request.')
            for listener in self.listeners:
                if hasattr(listener, 'stop_logging'):
                    listener.stop_logging()
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

    # Instantiate listeners
    ee_pos_logger = EEPositionLogger()
    ee_vel_logger = EEVelocityLogger()
    joint_state_logger = JointStateLogger()
    joint_vel_cmd_logger = JointVelocityCmdLogger()
    ee_pose_cmd_logger = EEPoseCmdLogger()
    ee_vel_cmd_logger = EEVelocityCmdLogger()

    listeners = [ee_pos_logger, ee_vel_logger, joint_state_logger, joint_vel_cmd_logger, ee_pose_cmd_logger, ee_vel_cmd_logger]
    log_control_server = LogControlServer(listeners)

    # Use MultiThreadedExecutor to spin all nodes
    executor = MultiThreadedExecutor()
    executor.add_node(log_control_server)
    for listener in listeners:
        executor.add_node(listener)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for listener in listeners:
            if hasattr(listener, 'destroy_node_and_close'):
                listener.destroy_node_and_close()
            else:
                listener.destroy_node()
        log_control_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
