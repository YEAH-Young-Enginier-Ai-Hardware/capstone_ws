#!/usr/bin/env python3

import os
import math
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalPoseRecorder(Node):
    def __init__(self):
        super().__init__("goal_pose_recorder")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("save_file", os.path.expanduser("~/yeah_ws/local/recorded_goals.yaml"))
        self.declare_parameter("auto_save", True)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.save_file = self.get_parameter("save_file").value
        self.auto_save = bool(self.get_parameter("auto_save").value)

        self.goals = []
        self.latest_goal = None

        os.makedirs(os.path.dirname(self.save_file), exist_ok=True)
        self.load_existing()

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.create_service(Trigger, "/goal_pose_recorder/save_last", self.save_last_callback)
        self.create_service(Trigger, "/goal_pose_recorder/list", self.list_callback)
        self.create_service(Trigger, "/goal_pose_recorder/clear", self.clear_callback)

        self.get_logger().info("GoalPoseRecorder started.")
        self.get_logger().info(f"Subscribing: {self.goal_topic}")
        self.get_logger().info(f"Save file: {self.save_file}")
        self.get_logger().info(f"Auto save: {self.auto_save}")

    def load_existing(self):
        if not os.path.exists(self.save_file):
            self.goals = []
            return

        try:
            with open(self.save_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            self.goals = data.get("goals", [])
        except Exception:
            self.goals = []

    def write_file(self):
        data = {
            "goals": self.goals
        }

        with open(self.save_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    def goal_to_dict(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation)

        return {
            "name": f"goal_{len(self.goals) + 1}",
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
            "yaw": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
        }

    def goal_callback(self, msg):
        self.latest_goal = msg

        self.get_logger().info(
            f"Received goal_pose: frame={msg.header.frame_id}, "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

        if self.auto_save:
            goal = self.goal_to_dict(msg)
            self.goals.append(goal)
            self.write_file()
            self.get_logger().info(f"Saved goal: {goal['name']}")

    def save_last_callback(self, request, response):
        if self.latest_goal is None:
            response.success = False
            response.message = "No goal_pose received yet."
            return response

        goal = self.goal_to_dict(self.latest_goal)
        self.goals.append(goal)
        self.write_file()

        response.success = True
        response.message = f"Saved latest goal: {goal}"
        return response

    def list_callback(self, request, response):
        if len(self.goals) == 0:
            response.success = True
            response.message = "No recorded goals."
            return response

        lines = []
        for i, g in enumerate(self.goals):
            lines.append(
                f"{i}: {g.get('name', 'goal')}, "
                f"frame={g.get('frame_id', 'map')}, "
                f"x={g['x']:.2f}, y={g['y']:.2f}, "
                f"yaw_deg={g.get('yaw_deg', 0.0):.1f}"
            )

        response.success = True
        response.message = "\n".join(lines)
        return response

    def clear_callback(self, request, response):
        self.goals = []
        self.write_file()

        response.success = True
        response.message = "Recorded goals cleared."
        return response


def main():
    rclpy.init()
    node = GoalPoseRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
