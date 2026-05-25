#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
import os
from pynput import keyboard

class EEVelocityLogger(Node):
    def __init__(self):
        super().__init__('ee_velocity_logger')

        self.declare_parameter("save_interval", 0.05)
        self.declare_parameter("topic_name", "/frankaRight/ee_velocity")

        self.save_interval = self.get_parameter("save_interval").get_parameter_value().double_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        self.recording = False
        self.latest_msg = None

        self.save_dir = "/docker_volume/ros2_ws/src/joint_state_collection/ee_velocity_data"
        os.makedirs(self.save_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Twist,
            self.topic_name,
            self.velocity_callback,
            10
        )

        self.timer = self.create_timer(self.save_interval, self.save_synced_data)

        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

        self.get_logger().info("[EE Velocity Logger] Ready. Press '1' to start, '2' to stop logging.")

    def velocity_callback(self, msg):
        self.latest_msg = msg
        if self.recording:
            self.write_to_file(self.realtime_file, msg)

    def save_synced_data(self):
        if not self.recording or self.latest_msg is None:
            return
        self.write_to_file(self.synced_file, self.latest_msg)

    def write_to_file(self, file_handle, msg):
        if file_handle is None or file_handle.closed:
            return
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        lin = msg.linear
        ang = msg.angular
        line = f"{timestamp},{lin.x:.6f},{lin.y:.6f},{lin.z:.6f},{ang.x:.6f},{ang.y:.6f},{ang.z:.6f}\n"
        file_handle.write(line)

    def on_key_press(self, key):
        try:
            if key.char == '1':
                self.start_logging()
            elif key.char == '2':
                self.stop_logging()
        except AttributeError:
            pass

    def start_logging(self):
        if not self.recording:
            # Ensure save_dir exists
            os.makedirs(self.save_dir, exist_ok=True)
            date_str = datetime.now().strftime("%Y_%m_%d")
            prefix = f"{date_str}_"
            suffix_full = "ee_velocity_right_full.txt"
            suffix_synced = "ee_velocity_right_synced.txt"

            existing_files = os.listdir(self.save_dir)
            max_index = 0
            for filename in existing_files:
                if filename.startswith(prefix) and filename.endswith(".txt"):
                    try:
                        parts = filename.split("_")
                        index = int(parts[3])
                        if index > max_index:
                            max_index = index
                    except (IndexError, ValueError):
                        continue
            next_index = max_index + 1

            base_filename = f"{date_str}_{next_index}_ee_velocity_right"
            self.synced_file_path = os.path.join(self.save_dir, f"{base_filename}_synced.txt")
            self.realtime_file_path = os.path.join(self.save_dir, f"{base_filename}_full.txt")

            self.synced_file = open(self.synced_file_path, 'w')
            self.realtime_file = open(self.realtime_file_path, 'w')

            header = "timestamp,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n"
            self.synced_file.write(header)
            self.realtime_file.write(header)
            self.recording = True
            self.get_logger().info(f"Started EE velocity logging: {base_filename}")

    def stop_logging(self):
        if self.recording:
            self.synced_file.close()
            self.realtime_file.close()
            self.recording = False
            self.get_logger().info("Stopped EE velocity logging.")

    def destroy_node_and_close(self):
        self.keyboard_listener.stop()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger = EEVelocityLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node_and_close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
