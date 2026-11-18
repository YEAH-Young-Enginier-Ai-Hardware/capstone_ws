#!/usr/bin/env python3

"""
Waypoint Event Manager for /goal_pose-based waypoint navigation.

Purpose
-------
This node reads a waypoint YAML file and publishes one waypoint at a time to
/goal_pose. This is useful when another node, such as fall_detection_unified_node,
needs to store and restore the currently active normal goal through /goal_pose.

Scenario
--------
waypoint 1 -> waypoint 2 이동 중 낙상 이벤트 발생
  1) fall_detection_unified_node cancels current /navigate_to_pose goal
  2) fall_detection_unified_node sends fallen-person goal to /goal_pose
  3) siren starts when robot reaches fallen-person goal
  4) React turns siren off by publishing /siren_control False
  5) fall_detection_unified_node republishes the saved normal goal
  6) this waypoint manager keeps the same current waypoint index and continues
     after the robot reaches that waypoint

Services
--------
/waypoint_event/start       std_srvs/srv/Trigger
/waypoint_event/stop        std_srvs/srv/Trigger
/waypoint_event/reset       std_srvs/srv/Trigger
/waypoint_event/next        std_srvs/srv/Trigger
/waypoint_event/list        std_srvs/srv/Trigger
/waypoint_event/reload      std_srvs/srv/Trigger

Topics
------
Publishes: /goal_pose geometry_msgs/PoseStamped
Subscribes: /fall_detection/status std_msgs/String
Subscribes: /siren_control std_msgs/Bool
Publishes: /waypoint_event/status std_msgs/String (JSON text)
"""

import json
import math
import os
import yaml
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


class WaypointEventManager(Node):
    def __init__(self):
        super().__init__("waypoint_event_manager")

        self.declare_parameter("waypoints_file", os.path.expanduser("~/yeah_ws/local/waypoints.yaml"))
        self.declare_parameter("route_name", "construction_test")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("xy_tolerance", 0.55)
        self.declare_parameter("pause_between_goals_sec", 0.3)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("republish_current_goal_on_resume", False)
        self.declare_parameter("auto_start", False)

        self.waypoints_file = self.get_parameter("waypoints_file").value
        self.route_name = self.get_parameter("route_name").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.xy_tolerance = float(self.get_parameter("xy_tolerance").value)
        self.pause_between_goals_sec = float(self.get_parameter("pause_between_goals_sec").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.republish_current_goal_on_resume = bool(
            self.get_parameter("republish_current_goal_on_resume").value
        )
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.status_pub = self.create_publisher(String, "/waypoint_event/status", 10)

        self.create_subscription(String, "/fall_detection/status", self.fall_status_callback, 10)
        self.create_subscription(Bool, "/siren_control", self.siren_callback, 10)

        self.create_service(Trigger, "/waypoint_event/start", self.start_callback)
        self.create_service(Trigger, "/waypoint_event/stop", self.stop_callback)
        self.create_service(Trigger, "/waypoint_event/reset", self.reset_callback)
        self.create_service(Trigger, "/waypoint_event/next", self.next_callback)
        self.create_service(Trigger, "/waypoint_event/list", self.list_callback)
        self.create_service(Trigger, "/waypoint_event/reload", self.reload_callback)

        self.waypoints: List[Dict] = []
        self.frame_id: str = self.global_frame
        self.current_index: int = 0
        self.running: bool = False
        self.route_done: bool = False
        self.paused_by_fall: bool = False
        self.siren_on: bool = False
        self.last_fall_status: str = "NORMAL"
        self.last_goal_publish_time = None
        self.arrival_time = None

        self.load_waypoints()

        self.timer = self.create_timer(self.publish_period_sec, self.tick)

        if self.auto_start:
            self.running = True
            self.publish_current_goal(force=True)

        self.get_logger().info("WaypointEventManager started.")
        self.get_logger().info(f"waypoints_file: {self.waypoints_file}")
        self.get_logger().info(f"route_name: {self.route_name}")
        self.get_logger().info(f"goal_topic: {self.goal_topic}")

    def load_waypoints(self):
        with open(self.waypoints_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not data or "routes" not in data:
            raise RuntimeError("waypoints.yaml must contain top-level 'routes'.")

        routes = data["routes"]
        if self.route_name not in routes:
            raise RuntimeError(f"route_name '{self.route_name}' not found. Available: {list(routes.keys())}")

        route = routes[self.route_name]
        self.frame_id = route.get("frame_id", self.global_frame)
        self.waypoints = list(route.get("waypoints", []))

        if not self.waypoints:
            raise RuntimeError(f"route '{self.route_name}' has no waypoints.")

        for i, wp in enumerate(self.waypoints):
            if "x" not in wp or "y" not in wp:
                raise RuntimeError(f"waypoint index {i} must contain x and y")

    def make_pose(self, index: int) -> PoseStamped:
        wp = self.waypoints[index]
        x = float(wp["x"])
        y = float(wp["y"])

        if "yaw" in wp:
            yaw = float(wp["yaw"])
        else:
            yaw = math.radians(float(wp.get("yaw_deg", 0.0)))

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def current_waypoint_name(self) -> str:
        if not self.waypoints or self.current_index >= len(self.waypoints):
            return "none"
        return str(self.waypoints[self.current_index].get("name", f"wp_{self.current_index + 1}"))

    def publish_current_goal(self, force: bool = False):
        if not self.running or self.route_done:
            return
        if self.current_index >= len(self.waypoints):
            self.route_done = True
            self.running = False
            self.get_logger().info("Route completed.")
            return

        now = self.get_clock().now()
        if not force and self.last_goal_publish_time is not None:
            dt = (now - self.last_goal_publish_time).nanoseconds / 1e9
            if dt < 1.0:
                return

        goal = self.make_pose(self.current_index)
        self.goal_pub.publish(goal)
        self.last_goal_publish_time = now
        self.get_logger().info(
            f"Published waypoint {self.current_index + 1}/{len(self.waypoints)} "
            f"({self.current_waypoint_name()}): x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return None

    def distance_to_current_waypoint(self) -> Optional[float]:
        if self.current_index >= len(self.waypoints):
            return None
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return None
        rx, ry = robot_xy
        wp = self.waypoints[self.current_index]
        return math.hypot(float(wp["x"]) - rx, float(wp["y"]) - ry)

    def fall_status_callback(self, msg: String):
        status = msg.data.strip().upper()
        self.last_fall_status = status

        if status in {"DETECTING", "DANGEROUS", "GOAL_SENT"}:
            if not self.paused_by_fall:
                self.get_logger().warn(f"Pausing waypoint manager due to fall status: {status}")
            self.paused_by_fall = True

        if status == "NORMAL" and not self.siren_on:
            if self.paused_by_fall:
                self.get_logger().warn("Fall status NORMAL. Waypoint manager can continue.")
            self.paused_by_fall = False

    def siren_callback(self, msg: Bool):
        self.siren_on = bool(msg.data)

        if self.siren_on:
            if not self.paused_by_fall:
                self.get_logger().warn("Pausing waypoint manager because siren is ON.")
            self.paused_by_fall = True
            return

        # Siren OFF means the fall event has been handled in React/UI.
        if self.paused_by_fall:
            self.get_logger().warn("Siren OFF. Keeping same waypoint index and resuming waypoint manager.")
        self.paused_by_fall = False
        self.arrival_time = None

        # Usually fall_detection_unified_node republishes the saved normal /goal_pose.
        # Set republish_current_goal_on_resume=true only if you want this manager to do it too.
        if self.running and not self.route_done and self.republish_current_goal_on_resume:
            self.publish_current_goal(force=True)

    def tick(self):
        self.publish_status()

        if not self.running or self.route_done:
            return

        if self.paused_by_fall or self.siren_on:
            return

        if self.current_index >= len(self.waypoints):
            self.route_done = True
            self.running = False
            self.get_logger().info("Route completed.")
            return

        # Make sure the current goal has been published at least once.
        if self.last_goal_publish_time is None:
            self.publish_current_goal(force=True)
            return

        dist = self.distance_to_current_waypoint()
        if dist is None:
            return

        if dist > self.xy_tolerance:
            return

        now = self.get_clock().now()
        if self.arrival_time is None:
            self.arrival_time = now
            self.get_logger().info(
                f"Arrived near waypoint {self.current_index + 1} ({self.current_waypoint_name()}), "
                f"distance={dist:.2f}m"
            )
            return

        wait_time = (now - self.arrival_time).nanoseconds / 1e9
        if wait_time < self.pause_between_goals_sec:
            return

        self.current_index += 1
        self.arrival_time = None
        self.last_goal_publish_time = None

        if self.current_index >= len(self.waypoints):
            self.route_done = True
            self.running = False
            self.get_logger().info("Route completed.")
            return

        self.publish_current_goal(force=True)

    def publish_status(self):
        msg = String()
        dist = self.distance_to_current_waypoint() if self.running and not self.route_done else None
        payload = {
            "running": self.running,
            "route_done": self.route_done,
            "paused_by_fall": self.paused_by_fall,
            "siren_on": self.siren_on,
            "fall_status": self.last_fall_status,
            "route_name": self.route_name,
            "current_index": self.current_index,
            "total": len(self.waypoints),
            "current_name": self.current_waypoint_name(),
            "distance_to_current": dist,
        }
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def start_callback(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints loaded."
            return response

        self.running = True
        self.route_done = False
        self.paused_by_fall = False
        self.siren_on = False
        self.current_index = 0
        self.arrival_time = None
        self.last_goal_publish_time = None
        self.publish_current_goal(force=True)

        response.success = True
        response.message = f"Started route '{self.route_name}' with {len(self.waypoints)} waypoints."
        return response

    def stop_callback(self, request, response):
        self.running = False
        self.route_done = False
        self.paused_by_fall = False
        self.arrival_time = None
        response.success = True
        response.message = "Waypoint manager stopped. Current Nav2 goal was not canceled."
        return response

    def reset_callback(self, request, response):
        self.current_index = 0
        self.route_done = False
        self.arrival_time = None
        self.last_goal_publish_time = None
        response.success = True
        response.message = "Waypoint route reset to first waypoint."
        return response

    def next_callback(self, request, response):
        if self.current_index + 1 >= len(self.waypoints):
            self.route_done = True
            self.running = False
            response.success = False
            response.message = "Already at last waypoint. Route marked done."
            return response

        self.current_index += 1
        self.arrival_time = None
        self.last_goal_publish_time = None
        if self.running:
            self.publish_current_goal(force=True)
        response.success = True
        response.message = f"Skipped to waypoint index {self.current_index}."
        return response

    def list_callback(self, request, response):
        lines = [f"Route '{self.route_name}' / frame '{self.frame_id}'"]
        for i, wp in enumerate(self.waypoints):
            marker = " <== current" if i == self.current_index else ""
            lines.append(
                f"{i}: {wp.get('name', f'wp_{i + 1}')}, "
                f"x={float(wp['x']):.2f}, y={float(wp['y']):.2f}, "
                f"yaw_deg={float(wp.get('yaw_deg', 0.0)):.1f}{marker}"
            )
        response.success = True
        response.message = "\n".join(lines)
        return response

    def reload_callback(self, request, response):
        try:
            self.load_waypoints()
            self.current_index = 0
            self.route_done = False
            self.arrival_time = None
            self.last_goal_publish_time = None
            response.success = True
            response.message = f"Reloaded {len(self.waypoints)} waypoints from {self.waypoints_file}."
        except Exception as e:
            response.success = False
            response.message = f"Reload failed: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointEventManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
