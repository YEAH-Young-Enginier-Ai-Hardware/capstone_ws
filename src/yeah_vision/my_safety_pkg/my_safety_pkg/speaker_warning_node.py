import os
import time
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory


class SpeakerWarningNode(Node):
    def __init__(self):
        super().__init__("speaker_warning_node")

        self.declare_parameter("cooldown_sec", 5.0)
        self.declare_parameter("helmet_file", "helmet.mp3")
        self.declare_parameter("siren_file", "siren.mp3")
        self.declare_parameter("siren_repeat_gap_sec", 0.5)

        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.helmet_file = self.get_parameter("helmet_file").value
        self.siren_file = self.get_parameter("siren_file").value
        self.siren_repeat_gap_sec = float(
            self.get_parameter("siren_repeat_gap_sec").value
        )

        self.last_helmet_time = 0.0
        self.last_fall_time = 0.0

        self.siren_active = False
        self.siren_thread = None
        self.siren_lock = threading.Lock()

        package_share = get_package_share_directory("my_safety_pkg")
        audio_dir = os.path.join(package_share, "audio")

        self.helmet_path = os.path.join(audio_dir, self.helmet_file)
        self.siren_path = os.path.join(audio_dir, self.siren_file)

        self.helmet_sub = self.create_subscription(
            Bool,
            "/helmet_warning",
            self.helmet_warning_callback,
            10,
        )

        self.fall_sub = self.create_subscription(
            Bool,
            "/fall_warning",
            self.fall_warning_callback,
            10,
        )

        self.siren_control_sub = self.create_subscription(
            Bool,
            "/siren_control",
            self.siren_control_callback,
            10,
        )

        self.get_logger().info("speaker_warning_node started")
        self.get_logger().info(f"helmet_path: {self.helmet_path}")
        self.get_logger().info(f"siren_path: {self.siren_path}")
        self.get_logger().info("topics: /helmet_warning, /fall_warning, /siren_control")

    def helmet_warning_callback(self, msg: Bool):
        if not msg.data:
            return

        now = time.time()
        if now - self.last_helmet_time < self.cooldown_sec:
            return

        self.last_helmet_time = now
        self.get_logger().warn("helmet warning audio play")
        self.play_audio_once(self.helmet_path)

    def fall_warning_callback(self, msg: Bool):
        if not msg.data:
            return

        now = time.time()
        if now - self.last_fall_time < self.cooldown_sec:
            return

        self.last_fall_time = now
        self.get_logger().warn("fall warning received, siren start")
        self.start_siren()

    def siren_control_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("siren_control=True, siren start")
            self.start_siren()
        else:
            self.get_logger().warn("siren_control=False, siren stop")
            self.stop_siren()

    def start_siren(self):
        with self.siren_lock:
            if self.siren_active:
                self.get_logger().info("siren already active")
                return

            if not os.path.exists(self.siren_path):
                self.get_logger().error(f"siren file not found: {self.siren_path}")
                return

            self.siren_active = True
            self.siren_thread = threading.Thread(
                target=self.siren_loop,
                daemon=True,
            )
            self.siren_thread.start()

    def stop_siren(self):
        with self.siren_lock:
            if not self.siren_active:
                self.get_logger().info("siren already stopped")
                return

            self.siren_active = False

    def siren_loop(self):
        self.get_logger().warn("siren loop started")

        while rclpy.ok():
            with self.siren_lock:
                if not self.siren_active:
                    break

            self.play_audio_once(self.siren_path)

            sleep_start = time.time()
            while time.time() - sleep_start < self.siren_repeat_gap_sec:
                with self.siren_lock:
                    if not self.siren_active:
                        break
                time.sleep(0.05)

            with self.siren_lock:
                if not self.siren_active:
                    break

        self.get_logger().warn("siren loop stopped")

    def play_audio_once(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"audio file not found: {file_path}")
            return

        try:
            if file_path.endswith(".mp3"):
                subprocess.run(["mpg123", "-q", file_path], check=True)
            elif file_path.endswith(".wav"):
                subprocess.run(["aplay", file_path], check=True)
            else:
                self.get_logger().error(f"unsupported audio format: {file_path}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"audio play failed: {e}")
        except FileNotFoundError as e:
            self.get_logger().error(f"audio player not found: {e}")

    def destroy_node(self):
        self.stop_siren()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerWarningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_siren()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()