#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class GoalPoseMarker(Node):
    def __init__(self):
        super().__init__('goal_pose_marker')
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.cb, 10
        )
        self.pub = self.create_publisher(Marker, '/clicked_goal_marker', 10)

    def cb(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'clicked_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose

        marker.scale.x = 0.7
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        self.pub.publish(marker)
        self.get_logger().info(
            f'Goal marker published: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

def main():
    rclpy.init()
    node = GoalPoseMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
