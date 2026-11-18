#!/usr/bin/env python3
"""
Unified Dynamic STVL Height Node - No Log Version

This node replaces BOTH of these older nodes/workflows:
  1) dynamic z-based STVL height update
  2) stair_zone_switch.py ROI-based min_obstacle_height switching

No-log version:
- All get_logger().info / warn / error outputs are removed.
- Runtime behavior is kept the same.
"""

from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class UnifiedDynamicSTVLHeightNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_stvl_height_node")

        # ------------------------------------------------------------------
        # TF settings
        # ------------------------------------------------------------------
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("robot_base_frame", "base_link")

        # ------------------------------------------------------------------
        # Target STVL parameter settings
        # ------------------------------------------------------------------
        self.declare_parameter("costmap_node", "/local_costmap/local_costmap")
        self.declare_parameter("layer_name", "spatio_temporal_voxel_layer")
        self.declare_parameter("observation_source", "lidar_mark")

        # Base obstacle-height window on normal flat ground.
        self.declare_parameter("base_min_obstacle_height", 0.25)
        self.declare_parameter("base_max_obstacle_height", 1.50)

        # Update timing and sensitivity.
        self.declare_parameter("update_period", 2.0)
        self.declare_parameter("update_threshold", 0.5)

        # Clamp delta_z for safety in case TF jumps.
        self.declare_parameter("min_delta_z", -0.30)
        self.declare_parameter("max_delta_z", 2.00)

        # Reference z handling.
        self.declare_parameter("use_current_z_as_reference", True)
        self.declare_parameter("reference_z", 0.0)

        # ------------------------------------------------------------------
        # Optional stair-zone override
        # ------------------------------------------------------------------
        self.declare_parameter("stair_zone_enabled", False)
        self.declare_parameter("stair_frame", "map")
        self.declare_parameter("stair_x_min", 0.0)
        self.declare_parameter("stair_x_max", 0.0)
        self.declare_parameter("stair_y_min", 0.0)
        self.declare_parameter("stair_y_max", 0.0)
        self.declare_parameter("stair_min_obstacle_height", 1.0)
        self.declare_parameter("stair_max_obstacle_height", 1.50)

        # ------------------------------------------------------------------
        # Runtime behavior
        # ------------------------------------------------------------------
        self.declare_parameter("enabled", True)
        self.declare_parameter("restore_on_shutdown", True)
        self.declare_parameter("clear_costmap_on_update", False)
        self.declare_parameter("clear_service", "/local_costmap/clear_entirely_local_costmap")

        # Read parameters.
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.costmap_node = str(self.get_parameter("costmap_node").value)
        self.layer_name = str(self.get_parameter("layer_name").value)
        self.observation_source = str(self.get_parameter("observation_source").value)

        self.base_min = float(self.get_parameter("base_min_obstacle_height").value)
        self.base_max = float(self.get_parameter("base_max_obstacle_height").value)
        self.update_period = float(self.get_parameter("update_period").value)
        self.update_threshold = float(self.get_parameter("update_threshold").value)
        self.min_delta_z = float(self.get_parameter("min_delta_z").value)
        self.max_delta_z = float(self.get_parameter("max_delta_z").value)
        self.use_current_z_as_reference = bool(self.get_parameter("use_current_z_as_reference").value)
        self.reference_z = float(self.get_parameter("reference_z").value)

        self.stair_zone_enabled = bool(self.get_parameter("stair_zone_enabled").value)
        self.stair_frame = str(self.get_parameter("stair_frame").value)
        self.stair_x_min = float(self.get_parameter("stair_x_min").value)
        self.stair_x_max = float(self.get_parameter("stair_x_max").value)
        self.stair_y_min = float(self.get_parameter("stair_y_min").value)
        self.stair_y_max = float(self.get_parameter("stair_y_max").value)
        self.stair_min = float(self.get_parameter("stair_min_obstacle_height").value)
        self.stair_max = float(self.get_parameter("stair_max_obstacle_height").value)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.restore_on_shutdown = bool(self.get_parameter("restore_on_shutdown").value)
        self.clear_costmap_on_update = bool(self.get_parameter("clear_costmap_on_update").value)
        self.clear_service = str(self.get_parameter("clear_service").value)

        if self.base_min >= self.base_max:
            raise ValueError("base_min_obstacle_height must be smaller than base_max_obstacle_height")

        self.min_param_name = f"{self.layer_name}.{self.observation_source}.min_obstacle_height"
        self.max_param_name = f"{self.layer_name}.{self.observation_source}.max_obstacle_height"

        # TF listener.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service clients.
        set_param_service = self._join_service_name(self.costmap_node, "set_parameters")
        self.set_param_client = self.create_client(SetParameters, set_param_service)
        self.clear_client = self.create_client(Empty, self.clear_service)

        # Control services.
        self.create_service(SetBool, "~/set_enabled", self.set_enabled_callback)
        self.create_service(SetBool, "~/set_stair_zone_enabled", self.set_stair_zone_enabled_callback)
        self.create_service(Trigger, "~/reset_reference", self.reset_reference_callback)
        self.create_service(Trigger, "~/restore_base_heights", self.restore_base_heights_callback)

        # Internal state.
        self.reference_initialized = not self.use_current_z_as_reference
        self.last_robot_z: Optional[float] = None
        self.last_delta_z: Optional[float] = None
        self.last_set_min: Optional[float] = None
        self.last_set_max: Optional[float] = None
        self.last_inside_stair: Optional[bool] = None
        self.param_request_in_progress = False

        self.timer = self.create_timer(self.update_period, self.timer_callback)

    @staticmethod
    def _join_service_name(node_name: str, service_name: str) -> str:
        node_name = node_name.rstrip("/")
        service_name = service_name.lstrip("/")
        return f"{node_name}/{service_name}"

    @staticmethod
    def _double_param(name: str, value: float) -> Parameter:
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(value)
        return param

    def get_robot_z(self) -> Optional[float]:
        """Return robot z in global_frame using TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            return float(tf.transform.translation.z)
        except TransformException:
            return None

    def is_inside_stair_zone(self) -> bool:
        """Return True if robot is inside the configured stair ROI."""
        if not self.stair_zone_enabled:
            return False

        try:
            tf = self.tf_buffer.lookup_transform(
                self.stair_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )

            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)

            inside = (self.stair_x_min <= x <= self.stair_x_max) and (
                self.stair_y_min <= y <= self.stair_y_max
            )

            self.last_inside_stair = inside
            return inside

        except TransformException:
            return False

    def timer_callback(self) -> None:
        if not self.enabled:
            return

        robot_z = self.get_robot_z()
        if robot_z is None:
            return

        self.last_robot_z = robot_z

        if not self.reference_initialized:
            self.reference_z = robot_z
            self.reference_initialized = True
            self.set_obstacle_heights(self.base_min, self.base_max)
            return

        raw_delta_z = robot_z - self.reference_z
        delta_z = max(self.min_delta_z, min(self.max_delta_z, raw_delta_z))
        self.last_delta_z = delta_z

        # Basic dynamic z compensation.
        new_min = self.base_min + delta_z
        new_max = self.base_max + delta_z

        # Stair-zone override.
        inside_stair = self.is_inside_stair_zone()

        if inside_stair:
            new_min = max(new_min, self.stair_min)
            new_max = max(new_max, self.stair_max)

            if new_max <= new_min + 0.05:
                new_max = new_min + 0.50

        if self.last_set_min is not None and self.last_set_max is not None:
            if (
                abs(new_min - self.last_set_min) < self.update_threshold
                and abs(new_max - self.last_set_max) < self.update_threshold
            ):
                return

        self.set_obstacle_heights(new_min, new_max)

    def set_obstacle_heights(self, min_height: float, max_height: float) -> None:
        if self.param_request_in_progress:
            return

        if min_height >= max_height:
            return

        if not self.set_param_client.wait_for_service(timeout_sec=0.1):
            return

        req = SetParameters.Request()
        req.parameters = [
            self._double_param(self.min_param_name, min_height),
            self._double_param(self.max_param_name, max_height),
        ]

        self.param_request_in_progress = True

        future = self.set_param_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.set_parameters_done_callback(fut, min_height, max_height)
        )

    def set_parameters_done_callback(
        self,
        future,
        min_height: float,
        max_height: float,
    ) -> None:
        self.param_request_in_progress = False

        try:
            result = future.result()

            if all(item.successful for item in result.results):
                self.last_set_min = min_height
                self.last_set_max = max_height

                if self.clear_costmap_on_update:
                    self.clear_local_costmap()

        except Exception:
            return

    def clear_local_costmap(self) -> None:
        if not self.clear_client.wait_for_service(timeout_sec=0.1):
            return

        self.clear_client.call_async(Empty.Request())

    def set_enabled_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self.enabled = bool(request.data)

        response.success = True
        response.message = (
            f"Dynamic STVL height update {'enabled' if self.enabled else 'disabled'}."
        )

        if not self.enabled:
            self.set_obstacle_heights(self.base_min, self.base_max)

        return response

    def set_stair_zone_enabled_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self.stair_zone_enabled = bool(request.data)
        self.last_inside_stair = None

        response.success = True
        response.message = (
            f"Stair-zone override {'enabled' if self.stair_zone_enabled else 'disabled'}."
        )

        return response

    def reset_reference_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        robot_z = self.get_robot_z()

        if robot_z is None:
            response.success = False
            response.message = "Failed to reset reference z because TF is unavailable."
            return response

        self.reference_z = robot_z
        self.reference_initialized = True
        self.last_delta_z = 0.0

        self.set_obstacle_heights(self.base_min, self.base_max)

        response.success = True
        response.message = f"Reference z reset to current robot z: {self.reference_z:.3f} m"

        return response

    def restore_base_heights_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.set_obstacle_heights(self.base_min, self.base_max)

        response.success = True
        response.message = (
            f"Restored base heights: min={self.base_min:.2f}, max={self.base_max:.2f}"
        )

        return response

    def shutdown_restore(self) -> None:
        if not self.restore_on_shutdown:
            return

        self.set_obstacle_heights(self.base_min, self.base_max)

        end_time = self.get_clock().now() + Duration(seconds=1.0)

        while (
            rclpy.ok()
            and self.param_request_in_progress
            and self.get_clock().now() < end_time
        ):
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnifiedDynamicSTVLHeightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_restore()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
