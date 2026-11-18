import asyncio
import json
import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import websockets


class OdomWebSocketBridge(Node):
    def __init__(self):
        super().__init__("odom_ws_bridge")

        self.declare_parameter("odom_topic", "/rko_lio/odometry")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("websocket_host", "0.0.0.0")
        self.declare_parameter("websocket_port", 8765)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.websocket_host = self.get_parameter("websocket_host").value
        self.websocket_port = int(self.get_parameter("websocket_port").value)

        self.ws_clients = set()
        self.latest_pose = None

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Subscribed odometry topic: {self.odom_topic}")
        self.get_logger().info(
            f"WebSocket server will run at ws://{self.websocket_host}:{self.websocket_port}"
        )

        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self.start_ws_loop, daemon=True)
        self.ws_thread.start()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.latest_pose = {
            "type": "robot_pose",
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "theta": float(theta),
            "theta_deg": float(math.degrees(theta)),
        }

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def start_ws_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_main())
        self.loop.run_forever()

    async def ws_main(self):
        async def handler(websocket):
            self.ws_clients.add(websocket)
            self.get_logger().info("Web client connected")

            try:
                await websocket.wait_closed()
            finally:
                self.ws_clients.discard(websocket)
                self.get_logger().info("Web client disconnected")

        server = await websockets.serve(
            handler,
            self.websocket_host,
            self.websocket_port
        )

        self.loop.create_task(self.broadcast_loop())

        self.get_logger().info(
            f"WebSocket server started: ws://{self.websocket_host}:{self.websocket_port}"
        )

        await server.wait_closed()

    async def broadcast_loop(self):
        interval = 1.0 / max(self.publish_hz, 1.0)

        while True:
            await asyncio.sleep(interval)

            if self.latest_pose is None:
                continue

            if not self.ws_clients:
                continue

            data = json.dumps(self.latest_pose)

            disconnected = []
            for client in list(self.ws_clients):
                try:
                    await client.send(data)
                except Exception:
                    disconnected.append(client)

            for client in disconnected:
                self.ws_clients.discard(client)


def main(args=None):
    rclpy.init(args=args)
    node = OdomWebSocketBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()