#!/usr/bin/env python3

"""
Unified fall-detection navigation manager and goal publisher.

Desired behavior
----------------
1. NORMAL navigation
   - Save the latest normal /goal_pose as a recovery goal.

2. DETECTING received
   - Cancel current Nav2 goal.
   - Stop robot immediately.
   - Keep DETECTING handling active for up to 2 seconds.
   - If NORMAL arrives within 2 seconds: treat as false alarm and resume previous goal.
   - If DANGEROUS arrives within 2 seconds: immediately switch to fallen-person goal.
   - If neither NORMAL nor DANGEROUS arrives for 2 seconds: treat as false alarm and resume previous goal.

3. DANGEROUS received
   - Cancel current Nav2 goal.
   - Stop robot while computing fallen-person goal.
   - Transform /fall_detection/fallen_person_pose to map frame.
   - Publish a goal near the fallen person.
   - Ignore NORMAL / DETECTING until arrival and siren handling is complete.

4. Fallen-person goal arrival
   - Cancel Nav2 goal.
   - Keep publishing /cmd_vel zero.
   - Publish /siren_control True once.
   - Ignore every fall status until React publishes /siren_control False.

5. React siren OFF
   - React publishes /siren_control False.
   - Resume the previous normal goal if available.
   - Reset event state.
   - Ignore new DANGEROUS events for 10 seconds.
"""

import copy
import math
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401, required for PoseStamped TF transform support


class FallDetectionUnifiedNode(Node):
    """Manage fall detection, Nav2 goals, siren control, and event cooldown."""

    def __init__(self) -> None:
        super().__init__("fall_detection_unified_node")

        # ------------------------------------------------------------------
        # Log control
        # ------------------------------------------------------------------
        # False면 info/warn/error 로그를 거의 출력하지 않음.
        # 디버깅할 때만 True로 바꿔서 사용.
        self.enable_logs: bool = False

        # ------------------------------------------------------------------
        # Configuration
        # ------------------------------------------------------------------
        # 낙상자와 떨어져 정지할 거리 [m]
        self.offset: float = 1.0

        # 낙상자 goal 생성 타임아웃 [s]
        self.send_goal_timeout: float = 5.0

        # 낙상자 goal 생성 재시도 주기 [s]
        self.send_goal_retry_period: float = 0.5

        # 낙상자 goal 도착 판정 거리 [m]
        self.arrival_tolerance: float = 0.45

        # DETECTING이 들어왔을 때 최대 정지 유지 시간 [s]
        # 이 안에 NORMAL이 오면 즉시 오탐 처리, DANGEROUS가 오면 즉시 낙상 대응.
        # 아무 것도 안 오면 2초 후 오탐 처리.
        self.detecting_hold_duration: float = 2.0

        # React에서 사이렌 OFF 후 DANGEROUS 재트리거 무시 시간 [s]
        self.cooldown_duration: float = 10.0

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.latest_fallen_pose: Optional[PoseStamped] = None

        self.prev_status: str = "NORMAL"
        self.current_status: str = "NORMAL"

        # 평상시 마지막 /goal_pose 저장. 낙상 처리 완료 후 복귀용.
        self.latest_normal_goal: Optional[PoseStamped] = None
        self.resume_previous_goal_when_normal: bool = False

        self.nav_canceled_for_current_event: bool = False
        self.dangerous_detected: bool = False

        self.send_goal_in_progress: bool = False
        self.send_goal_done: bool = False

        self.dangerous_start_time = None
        self.last_send_goal_try_time = None

        self.detecting_start_time = None

        self.active_fall_goal: Optional[PoseStamped] = None
        self.siren_started: bool = False

        # 기본값은 비활성화.
        # 아래 서비스로 true를 줘야 낙상 대응 기능이 작동함.
        self.goal_publishing_enabled: bool = False

        # 사이렌 OFF 후 재트리거 쿨다운 종료 시각.
        self.retrigger_cooldown_end = None

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.siren_pub = self.create_publisher(Bool, "/siren_control", 10)

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            String,
            "/fall_detection/status",
            self.status_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/fall_detection/fallen_person_pose",
            self.fallen_pose_callback,
            10,
        )

        # React/web에서 사이렌을 끌 때 /siren_control False가 들어오는 것을 감지.
        # 이 노드가 직접 발행한 True는 callback에서 무시함.
        self.create_subscription(
            Bool,
            "/siren_control",
            self.siren_callback,
            10,
        )

        # ------------------------------------------------------------------
        # Service clients
        # ------------------------------------------------------------------
        self.cancel_client = self.create_client(
            CancelGoal,
            "/navigate_to_pose/_action/cancel_goal",
        )

        # ------------------------------------------------------------------
        # Service servers
        # ------------------------------------------------------------------
        self.create_service(
            Trigger,
            "/fall_detection/send_goal",
            self.send_goal_callback,
        )

        self.create_service(
            SetBool,
            "/fall_detection/set_goal_publishing",
            self.set_goal_publishing_callback,
        )

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self.create_timer(0.05, self.timer_callback)

        self.log_info(
            "FallDetectionUnifiedNode started. Goal publishing is disabled by default."
        )

    # ----------------------------------------------------------------------
    # Logging helpers
    # ----------------------------------------------------------------------
    def log_info(self, text: str) -> None:
        if self.enable_logs:
            self.get_logger().info(text)

    def log_warn(self, text: str) -> None:
        if self.enable_logs:
            self.get_logger().warn(text)

    def log_error(self, text: str) -> None:
        if self.enable_logs:
            self.get_logger().error(text)

    # ----------------------------------------------------------------------
    # Subscription callbacks
    # ----------------------------------------------------------------------
    def fallen_pose_callback(self, msg: PoseStamped) -> None:
        """Store latest detected fallen-person pose."""
        self.latest_fallen_pose = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        """Store latest normal navigation goal for recovery."""
        # 낙상 이벤트 처리 중이 아닐 때만 평상시 goal로 저장.
        # 이 노드가 발행한 fall goal이 latest_normal_goal을 덮어쓰면 안 됨.
        if self.current_status == "NORMAL" and not self.dangerous_detected and not self.siren_started:
            self.latest_normal_goal = copy.deepcopy(msg)

    def siren_callback(self, msg: Bool) -> None:
        """Handle siren off command from React/web."""
        # True는 이 노드가 도착 시 직접 발행하는 값이므로 여기서는 처리하지 않음.
        if msg.data:
            return

        # 실제 낙상 이벤트로 사이렌이 켜진 상태가 아니면 무시.
        if not self.siren_started:
            return

        # 사이렌 OFF가 들어왔으므로, 이전 일반 goal로 복귀.
        if self.latest_normal_goal is not None:
            goal = copy.deepcopy(self.latest_normal_goal)
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)

        # 이벤트 상태 초기화.
        self.reset_event_flags(set_status_normal=True)

        # 10초 동안 DANGEROUS 재트리거 무시.
        self.retrigger_cooldown_end = (
            self.get_clock().now() + Duration(seconds=self.cooldown_duration)
        )

    def status_callback(self, msg: String) -> None:
        """Handle fall detection status updates."""
        status = msg.data.strip().upper()

        if status not in {"NORMAL", "DETECTING", "DANGEROUS", "GOAL_SENT"}:
            self.log_warn(f"Received unknown fall status: {status}")
            return

        # 사이렌이 켜진 뒤에는 React가 False를 줄 때까지 모든 이벤트 무시.
        if self.siren_started:
            return

        # 사이렌 OFF 이후 10초 동안 DANGEROUS 재트리거 무시.
        if (
            status == "DANGEROUS"
            and self.retrigger_cooldown_end is not None
            and self.get_clock().now() < self.retrigger_cooldown_end
        ):
            self.log_warn("Ignoring DANGEROUS event during cooldown.")
            return

        # goal_pose가 이미 나갔고 아직 사이렌이 켜지기 전이면,
        # NORMAL/DETECTING으로 인해 취소되거나 초기화되지 않게 무시.
        if (
            self.send_goal_done
            and self.active_fall_goal is not None
            and not self.siren_started
            and status in {"NORMAL", "DETECTING"}
        ):
            self.log_warn(
                f"Ignoring {status} because fallen-person goal is already sent."
            )
            return

        if not self.goal_publishing_enabled:
            self.current_status = status
            self.prev_status = status
            return

        # DANGEROUS가 한 번 시작된 뒤에는 NORMAL/DETECTING을 무시하고,
        # 낙상자 goal 생성/이동/사이렌 처리까지 계속 진행.
        if self.dangerous_detected and status in {"NORMAL", "DETECTING"}:
            return

        # ------------------------------------------------------------
        # DETECTING 처리
        # ------------------------------------------------------------
        if status == "DETECTING":
            # 처음 DETECTING에 들어온 순간만 처리.
            if self.current_status != "DETECTING":
                self.log_warn("DETECTING: cancel Nav2 goal and stop for hold window.")
                self.cancel_current_navigation()
                self.nav_canceled_for_current_event = True
                self.resume_previous_goal_when_normal = True
                self.detecting_start_time = self.get_clock().now()
                self.publish_zero_cmd()

            self.current_status = "DETECTING"
            self.prev_status = "DETECTING"
            return

        # ------------------------------------------------------------
        # DETECTING -> NORMAL 처리
        # ------------------------------------------------------------
        if status == "NORMAL":
            if self.current_status == "DETECTING":
                # 2초 안에 NORMAL이 들어오면 바로 오탐 처리하고 이전 goal 복귀.
                self.log_warn("DETECTING -> NORMAL: false alarm, resume previous goal.")
                self.publish_zero_cmd()
                self.resume_previous_goal()
                self.reset_event_flags(set_status_normal=True)
                return

            self.current_status = "NORMAL"
            self.prev_status = "NORMAL"
            return

        # ------------------------------------------------------------
        # DANGEROUS 처리
        # ------------------------------------------------------------
        if status == "DANGEROUS" and not self.dangerous_detected:
            self.log_warn("DANGEROUS detected: prepare fallen-person goal.")

            self.detecting_start_time = None

            if not self.nav_canceled_for_current_event:
                self.cancel_current_navigation()
                self.nav_canceled_for_current_event = True

            self.resume_previous_goal_when_normal = False
            self.dangerous_detected = True
            self.send_goal_done = False
            self.send_goal_in_progress = False
            self.dangerous_start_time = self.get_clock().now()
            self.last_send_goal_try_time = None

            self.current_status = "DANGEROUS"
            self.prev_status = "DANGEROUS"

            self.publish_zero_cmd()
            return

        if status == "GOAL_SENT":
            self.current_status = "GOAL_SENT"
            self.prev_status = "GOAL_SENT"
            return

    # ----------------------------------------------------------------------
    # Service callbacks
    # ----------------------------------------------------------------------
    def set_goal_publishing_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Enable or disable fall-detection goal publishing."""
        self.goal_publishing_enabled = bool(request.data)

        state_str = "enabled" if self.goal_publishing_enabled else "disabled"
        response.success = True
        response.message = f"Fall detection goal publishing {state_str}."

        self.log_info(response.message)

        if not self.goal_publishing_enabled:
            # 사이렌 OFF는 React에서만 처리하게 두고, 내부 상태만 초기화.
            self.reset_event_flags(set_status_normal=True)

        return response

    def send_goal_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Compute and publish a fallen-person goal on demand."""
        if not self.goal_publishing_enabled:
            response.success = False
            response.message = "Fall detection goal publishing is disabled."
            return response

        if self.latest_fallen_pose is None:
            response.success = False
            response.message = "No fallen person pose received yet."
            return response

        success, text = self.compute_and_publish_goal()
        response.success = success
        response.message = text
        return response

    # ----------------------------------------------------------------------
    # Timer callback
    # ----------------------------------------------------------------------
    def timer_callback(self) -> None:
        if not self.goal_publishing_enabled:
            return

        # 사이렌이 켜져 있는 동안은 React False가 오기 전까지 계속 정지 유지.
        if self.siren_started:
            self.publish_zero_cmd()
            return

        # DETECTING 상태에서는 정지 유지.
        # 2초 안에 NORMAL이 오면 status_callback에서 즉시 오탐 복귀.
        # 2초 안에 DANGEROUS가 오면 status_callback에서 즉시 낙상 대응.
        # 아무 것도 안 오면 여기서 2초 후 오탐 복귀.
        if self.current_status == "DETECTING":
            self.publish_zero_cmd()

            if self.detecting_start_time is not None:
                now = self.get_clock().now()
                elapsed = (now - self.detecting_start_time).nanoseconds / 1e9

                if elapsed >= self.detecting_hold_duration:
                    self.log_warn(
                        f"DETECTING hold expired after {elapsed:.1f}s: false alarm, resume previous goal."
                    )
                    self.publish_zero_cmd()
                    self.resume_previous_goal()
                    self.reset_event_flags(set_status_normal=True)

            return

        # DANGEROUS인데 goal이 아직 안 나갔으면 정지 유지.
        if self.current_status == "DANGEROUS" and not self.send_goal_done:
            self.publish_zero_cmd()

        # goal이 이미 나간 경우 도착 여부만 확인.
        if self.send_goal_done and self.active_fall_goal is not None:
            self.check_arrival_and_start_siren()
            return

        if not self.dangerous_detected:
            return

        if self.send_goal_done:
            return

        if self.send_goal_in_progress:
            return

        now = self.get_clock().now()

        if self.dangerous_start_time is None:
            return

        elapsed = (now - self.dangerous_start_time).nanoseconds / 1e9
        if elapsed > self.send_goal_timeout:
            self.log_error(
                f"Fallen-person goal service timed out after {self.send_goal_timeout}s."
            )
            self.send_goal_done = True
            return

        if self.last_send_goal_try_time is not None:
            retry_elapsed = (now - self.last_send_goal_try_time).nanoseconds / 1e9
            if retry_elapsed < self.send_goal_retry_period:
                return

        self.last_send_goal_try_time = now

        self.send_goal_in_progress = True
        success, text = self.compute_and_publish_goal()
        self.send_goal_in_progress = False

        if success:
            self.log_warn(f"Fallen-person goal sent: {text}")
            self.send_goal_done = True
            self.current_status = "GOAL_SENT"
            self.prev_status = "GOAL_SENT"
        else:
            self.log_warn(f"send_goal failed, will retry: {text}")

    # ----------------------------------------------------------------------
    # Helper methods
    # ----------------------------------------------------------------------
    def compute_and_publish_goal(self) -> Tuple[bool, str]:
        """Compute and publish goal offset from fallen-person pose."""
        if self.latest_fallen_pose is None:
            return False, "No fallen person pose received yet."

        try:
            pose_for_tf = copy.deepcopy(self.latest_fallen_pose)

            # 최신 TF 기준으로 변환.
            pose_for_tf.header.stamp = rclpy.time.Time().to_msg()

            person_map = self.tf_buffer.transform(
                pose_for_tf,
                "map",
                timeout=Duration(seconds=0.5),
            )

            robot_tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

        except Exception as e:
            return False, f"TF failed: {e}"

        px = person_map.pose.position.x
        py = person_map.pose.position.y

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y

        dx = px - rx
        dy = py - ry
        dist = math.hypot(dx, dy)

        if dist < self.offset + 0.1:
            return False, "Robot is already too close to fallen person."

        ux = dx / dist
        uy = dy / dist

        # 사람 위치에서 로봇 방향으로 offset만큼 떨어진 goal.
        goal_x = px - ux * self.offset
        goal_y = py - uy * self.offset

        # goal 위치에서 사람을 바라보게 yaw 설정.
        yaw = math.atan2(py - goal_y, px - goal_x)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"

        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.active_fall_goal = copy.deepcopy(goal_msg)
        self.siren_started = False

        self.goal_pub.publish(goal_msg)

        return True, (
            f"Published goal: x={goal_x:.2f}, y={goal_y:.2f}, offset={self.offset:.2f}m"
        )

    def check_arrival_and_start_siren(self) -> None:
        """Start siren once robot reaches active fall goal."""
        if self.siren_started:
            return

        if self.active_fall_goal is None:
            return

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.log_warn(f"Arrival TF failed: {e}")
            return

        gx = self.active_fall_goal.pose.position.x
        gy = self.active_fall_goal.pose.position.y

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y

        distance_to_goal = math.hypot(gx - rx, gy - ry)

        if distance_to_goal > self.arrival_tolerance:
            return

        # 도착했다고 판단되면 Nav2 goal을 취소하고 확실히 정지.
        self.cancel_current_navigation()
        self.publish_zero_cmd()

        # 사이렌 ON. 실제 반복 재생은 speaker_warning_node가 담당.
        self.publish_siren(True)
        self.siren_started = True
        self.current_status = "SIREN_ACTIVE"
        self.prev_status = "SIREN_ACTIVE"

        self.log_warn(
            f"Arrived at fallen-person goal: distance={distance_to_goal:.2f}m. Siren started."
        )

    def publish_siren(self, enabled: bool) -> None:
        """Publish siren control command."""
        msg = Bool()
        msg.data = enabled
        self.siren_pub.publish(msg)

    def publish_zero_cmd(self) -> None:
        """Publish zero Twist command."""
        self.cmd_vel_pub.publish(Twist())

    def cancel_current_navigation(self) -> None:
        """Cancel active Nav2 NavigateToPose goal."""
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.log_error(
                "Nav2 cancel service unavailable: /navigate_to_pose/_action/cancel_goal"
            )
            return

        req = CancelGoal.Request()
        future = self.cancel_client.call_async(req)
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future) -> None:
        try:
            result = future.result()
            self.log_warn(
                f"Nav2 cancel response: goals_canceling={len(result.goals_canceling)}"
            )
        except Exception as e:
            self.log_error(f"Nav2 cancel request failed: {e}")

    def resume_previous_goal(self) -> None:
        """Republish previous normal goal after false alarm."""
        if not self.resume_previous_goal_when_normal:
            return

        if self.latest_normal_goal is None:
            self.log_warn("No saved goal to resume.")
            return

        goal = copy.deepcopy(self.latest_normal_goal)
        goal.header.stamp = self.get_clock().now().to_msg()

        self.goal_pub.publish(goal)

        self.log_warn(
            f"Republished previous goal: frame={goal.header.frame_id}, "
            f"x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

    def reset_event_flags(self, set_status_normal: bool = False) -> None:
        """Reset event-related flags."""
        # 사이렌 OFF는 React 확인 버튼에서만 처리. 여기서 False를 보내지 않음.
        self.nav_canceled_for_current_event = False
        self.dangerous_detected = False

        self.send_goal_in_progress = False
        self.send_goal_done = False

        self.dangerous_start_time = None
        self.last_send_goal_try_time = None
        self.detecting_start_time = None

        self.active_fall_goal = None
        self.siren_started = False

        self.resume_previous_goal_when_normal = False

        if set_status_normal:
            self.current_status = "NORMAL"
            self.prev_status = "NORMAL"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallDetectionUnifiedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
