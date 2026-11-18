#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, ParameterType


class StairZoneSwitch(Node):
    def __init__(self):
        super().__init__("stair_zone_switch")

        self.target_node = "/local_costmap/local_costmap"
        self.param_name = "spatio_temporal_voxel_layer.lidar_mark.min_obstacle_height"

        self.cli = self.create_client(SetParameters, f"{self.target_node}/set_parameters")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 계단 ROI (map 프레임) - 필요에 맞게 수정
        self.x_min = self.declare_parameter("stair_x_min", 0.0).value
        self.x_max = self.declare_parameter("stair_x_max", 0.0).value
        self.y_min = self.declare_parameter("stair_y_min", 0.0).value
        self.y_max = self.declare_parameter("stair_y_max", 0.0).value

        self.normal_min_h = 0.3
        self.stair_min_h = 1.0
        self.last = None

        self.timer = self.create_timer(0.2, self.tick)

    def set_min_h(self, v: float):
        if not self.cli.service_is_ready():
            self.get_logger().warn("set_parameters service not ready yet", throttle_duration_sec=2.0)
            return

        req = SetParameters.Request()
        p = ParameterMsg()
        p.name = self.param_name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(v),
        )
        req.parameters = [p]

        future = self.cli.call_async(req)
        future.add_done_callback(self._set_done)

    def _set_done(self, future):
        try:
            res = future.result()
            ok = all(r.successful for r in res.results)
            if not ok:
                reasons = [r.reason for r in res.results if not r.successful]
                self.get_logger().error(f"Failed to set param: {reasons}")
        except Exception as e:
            self.get_logger().error(f"set_parameters call failed: {e}")

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y

            inside = (self.x_min < self.x_max and self.y_min < self.y_max
                      and self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max)
            target = self.stair_min_h if inside else self.normal_min_h

            if self.last != target:
                self.set_min_h(target)
                self.last = target
                self.get_logger().info(f"{self.param_name} -> {target:.2f}")
        except Exception:
            pass


def main():
    rclpy.init()
    node = StairZoneSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
