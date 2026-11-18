import json
import math
import os
import time
from datetime import datetime
from typing import Dict, Optional, Tuple, List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


class FallDetectionNode(Node):
    def __init__(self):
        super().__init__("fall_detection_node")

        self.bridge = CvBridge()

        self.color_image = None
        self.color_header = None
        self.depth_image = None
        self.depth_header = None
        self.camera_info = None

        # 기존 /yolo/detections_3d의 3D bbox 중심은 야외 depth outlier 때문에
        # 비정상적으로 길어진 bbox 중심을 publish할 수 있어 goal pose 기준에서 제외한다.

        self.person_score_thr = 0.5
        self.kpt_score_thr = 0.3

        self.depth_min_mm = 1
        self.depth_max_mm = 6000
        self.depth_patch_size = 15
        self.min_depth_valid_count = 20

        # 테스트 단계에서는 depth가 부족해도 pose 기반 판단을 허용한다.
        # 최종 로봇 적용 시 True로 바꾸면 depth가 유효할 때만 낙상 판단한다.
        self.require_depth_for_validity = False

        self.fall_hold_time = 1.0
        self.normal_recover_time = 1.0
        self.track_timeout = 1.0
        self.match_distance_px = 80.0

        # Keypoint + aligned depth 기반 좌표 필터 설정
        # 낙상 확정 후 torso keypoint 중심에서 3D 좌표를 1개씩 수집한다.
        # 최근 5개 좌표 중 평균에서 가장 먼 1개를 버리고 남은 4개 평균을 1회 publish한다.
        self.pose_filter_window_size = 5
        self.pose_filter_drop_count = 1

        # 이벤트 이미지 저장 설정
        # 이 폴더를 python3 -m http.server 8090 으로 열면 웹에서 이미지 접근 가능
        # 런치파일에서 event_image_base_url 파라미터를 넘기면 그 값이 우선 적용된다.
        self.declare_parameter("event_image_dir", os.path.expanduser("~/safety_event_images"))
        self.declare_parameter("event_image_base_url", "http://localhost:8090")

        self.event_image_dir = str(self.get_parameter("event_image_dir").value)
        self.event_image_base_url = str(
            self.get_parameter("event_image_base_url").value
        ).rstrip("/")
        os.makedirs(self.event_image_dir, exist_ok=True)

        # 같은 낙상 상황에서 이미지가 매 프레임 저장되는 것을 막기 위한 상태 기록
        self.last_event_state = "NORMAL"

        self.person_states: Dict[str, dict] = {}
        self.next_track_id = 0

        self.color_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            10,
        )

        self.detection_sub = self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.detection_callback,
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/fall_detection/debug_image",
            10,
        )

        self.fallen_pose_pub = self.create_publisher(
            PoseStamped,
            "/fall_detection/fallen_person_pose",
            10,
        )

        self.fall_status_pub = self.create_publisher(
            String,
            "/fall_detection/status",
            10,
        )

        # 낙상/안전모 공통 이벤트 이력용 토픽
        self.safety_event_pub = self.create_publisher(
            String,
            "/safety/event",
            10,
        )


        self.get_logger().info("fall_detection_node started")
        self.get_logger().info(f"event_image_dir: {self.event_image_dir}")
        self.get_logger().info(f"event_image_base_url: {self.event_image_base_url}")

    def color_callback(self, msg: Image):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_header = msg.header
        except Exception as e:
            self.get_logger().error(f"color convert failed: {e}")

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"depth convert failed: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def detection_callback(self, msg: DetectionArray):
        if self.color_image is None:
            self.publish_fall_status("NORMAL")
            return

        if self.depth_image is None:
            self.publish_fall_status("NORMAL")
            return

        now = time.time()

        person_count = 0
        debug_img = self.color_image.copy()
        img_h, img_w = debug_img.shape[:2]

        frame_display_state = "NORMAL"

        for det in msg.detections:
            if det.class_name != "person":
                continue

            det_score = float(det.score) if hasattr(det, "score") else 1.0
            if det_score < self.person_score_thr:
                continue

            bbox = self.get_bbox_xywh(det)
            if bbox is None:
                continue

            cx, cy, w, h = bbox

            person_count += 1

            x1, y1, x2, y2 = self.xywh_to_xyxy(cx, cy, w, h, img_w, img_h)

            raw_kp_dict = self.parse_keypoints(det)
            kp_dict = self.filter_keypoints_for_pose(raw_kp_dict)

            shoulder_center = self.get_pair_center_flexible(
                kp_dict, 5, 6, self.kpt_score_thr
            )
            hip_center = self.get_pair_center_flexible(
                kp_dict, 11, 12, self.kpt_score_thr
            )

            torso_depth_m, depth_valid_count = self.get_torso_depth(
                shoulder_center, hip_center
            )

            validity, validity_reason = self.evaluate_pose_validity(
                kp_dict=kp_dict,
                shoulder_center=shoulder_center,
                hip_center=hip_center,
                depth_valid_count=depth_valid_count,
                bbox_w=w,
                bbox_h=h,
                img_w=img_w,
                img_h=img_h,
            )

            features = self.compute_pose_features(
                kp_dict=kp_dict,
                shoulder_center=shoulder_center,
                hip_center=hip_center,
                torso_depth_m=torso_depth_m,
            )

            frame_state = self.classify_frame_state(validity, features)

            track_key = self.assign_track(cx, cy, now)
            final_state = self.update_person_state(track_key, frame_state, now)
            display_state = self.map_display_state(final_state, frame_state)

            frame_display_state = self.select_more_severe_state(
                frame_display_state,
                display_state,
            )

            # 낙상 판단과 같은 2D pose detection에서 torso keypoint 중심을 저장한다.
            # 3D bbox 중심은 야외 depth outlier에 취약하므로 goal pose 기준으로 사용하지 않는다.
            self.update_track_pose_candidate(
                track_key=track_key,
                shoulder_center=shoulder_center,
                hip_center=hip_center,
                torso_depth_m=torso_depth_m,
                depth_valid_count=depth_valid_count,
            )

            if final_state == "FALL_DETECTED":
                self.collect_and_publish_keypoint_pose(track_key)

            depth_str = (
                f"depth: {torso_depth_m:.2f}m"
                if torso_depth_m is not None
                else "depth: N/A"
            )
            label_1 = f"person | {display_state}"
            label_2 = depth_str

            self.draw_person_overlay(
                img=debug_img,
                bbox=(x1, y1, x2, y2),
                kp_dict=kp_dict,
                label_1=label_1,
                label_2=label_2,
                state=display_state,
                shoulder_center=shoulder_center,
                hip_center=hip_center,
            )

        self.cleanup_tracks(now)

        if person_count == 0:
            frame_display_state = "NORMAL"

        self.publish_fall_status(frame_display_state)

        # DANGEROUS로 확정되는 순간에만 현재 debug_img를 jpg로 저장하고 /safety/event로 image_url을 보낸다.
        self.publish_fall_event_if_needed(
            status=frame_display_state,
            debug_img=debug_img,
        )

        self.publish_debug_image(debug_img)


    def select_more_severe_state(self, current_state: str, new_state: str) -> str:
        priority = {
            "NORMAL": 0,
            "DETECTING": 1,
            "DANGEROUS": 2,
        }

        current_priority = priority.get(current_state, 0)
        new_priority = priority.get(new_state, 0)

        if new_priority > current_priority:
            return new_state

        return current_state

    def publish_fall_status(self, status: str):
        msg = String()

        if status == "DANGEROUS":
            msg.data = "DANGEROUS"
        elif status == "DETECTING":
            msg.data = "DETECTING"
        else:
            msg.data = "NORMAL"

        self.fall_status_pub.publish(msg)

    def publish_fall_event_if_needed(self, status: str, debug_img: np.ndarray):
        # 이벤트 이미지는 낙상 확정 상태인 DANGEROUS에서만 저장한다.
        # DETECTING은 웹 상태 표시용으로만 사용하고 이벤트 이력에는 저장하지 않는다.
        if status != "DANGEROUS":
            if status == "NORMAL":
                self.last_event_state = "NORMAL"
            return

        # DANGEROUS 상태가 계속 유지되는 동안 매 프레임 이벤트가 쌓이는 것을 방지한다.
        # DANGEROUS -> DETECTING -> DANGEROUS처럼 흔들려도 NORMAL로 완전히 복귀하기 전까지는 중복 저장하지 않는다.
        if self.last_event_state == "DANGEROUS":
            return

        image_url = self.save_event_image(status, debug_img)

        event_data = {
            "type": "fall_detected",
            "status": "DANGEROUS",
            "confidence": 0.9,
            "image_url": image_url,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "description": "쓰러짐 위험 상태가 감지되었습니다.",
        }

        msg = String()
        msg.data = json.dumps(event_data, ensure_ascii=False)
        self.safety_event_pub.publish(msg)

        self.last_event_state = "DANGEROUS"

    def save_event_image(self, status: str, debug_img: np.ndarray) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"fall_{status}_{timestamp}.jpg"
        image_path = os.path.join(self.event_image_dir, filename)

        saved = cv2.imwrite(image_path, debug_img)

        if not saved:
            self.get_logger().error(f"event image save failed: {image_path}")
            return ""

        return f"{self.event_image_base_url}/{filename}"

    def update_track_pose_candidate(
        self,
        track_key: str,
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
        torso_depth_m: Optional[float],
        depth_valid_count: int,
    ):
        """현재 detection frame에서 계산된 torso 중심 후보를 track에 저장한다."""
        if track_key not in self.person_states:
            return

        if shoulder_center is None or hip_center is None:
            return

        if torso_depth_m is None:
            return

        if depth_valid_count < self.min_depth_valid_count:
            return

        torso_u = (shoulder_center[0] + hip_center[0]) / 2.0
        torso_v = (shoulder_center[1] + hip_center[1]) / 2.0

        point = self.deproject_pixel_to_camera_point(
            u=torso_u,
            v=torso_v,
            depth_m=torso_depth_m,
        )
        if point is None:
            return

        x, y, z, frame_id = point

        state = self.person_states[track_key]
        state["last_torso_uv"] = (float(torso_u), float(torso_v))
        state["last_torso_depth_m"] = float(torso_depth_m)
        state["last_depth_valid_count"] = int(depth_valid_count)
        state["last_camera_point"] = (float(x), float(y), float(z))
        state["last_camera_point_frame"] = frame_id
        state["last_pose_candidate_time"] = time.time()

    def deproject_pixel_to_camera_point(
        self,
        u: float,
        v: float,
        depth_m: float,
    ) -> Optional[Tuple[float, float, float, str]]:
        """aligned depth pixel + camera_info를 camera optical frame 3D point로 변환한다."""
        if self.camera_info is None:
            self.get_logger().warn(
                "camera_info not received yet: cannot compute keypoint 3D point"
            )
            return None

        if depth_m is None or not math.isfinite(depth_m):
            return None

        if depth_m <= 0.0 or depth_m > (self.depth_max_mm / 1000.0):
            return None

        k = self.camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            self.get_logger().error("invalid camera_info: fx/fy is zero")
            return None

        z = float(depth_m)
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy

        frame_id = ""
        if self.camera_info.header.frame_id:
            frame_id = self.camera_info.header.frame_id
        elif self.depth_header is not None and self.depth_header.frame_id:
            frame_id = self.depth_header.frame_id
        else:
            frame_id = "camera_color_optical_frame"

        return x, y, z, frame_id

    def collect_and_publish_keypoint_pose(self, track_key: str):
        """FALL_DETECTED track의 keypoint+depth 기반 3D 좌표를 필터링 후 1회 publish한다."""
        if track_key not in self.person_states:
            return

        state = self.person_states[track_key]

        if not state.get("fall_confirmed", False):
            return

        if state.get("fall_pose_published", False):
            return

        point = state.get("last_camera_point")
        frame_id = state.get("last_camera_point_frame")
        candidate_time = state.get("last_pose_candidate_time", 0.0)

        if point is None or frame_id is None:
            return

        # 너무 오래된 keypoint/depth 후보는 사용하지 않는다.
        if time.time() - candidate_time > 0.5:
            return

        filtered_center = self.update_pose_buffer_and_filter(track_key, point)
        if filtered_center is None:
            return

        published = self.publish_filtered_fallen_person_pose(
            center=filtered_center,
            frame_id=frame_id,
        )
        if published:
            state["fall_pose_published"] = True
            state["pose_buffer"] = []
            self.get_logger().info(
                "fallen person keypoint pose published once: "
                f"x={filtered_center[0]:.3f}, "
                f"y={filtered_center[1]:.3f}, "
                f"z={filtered_center[2]:.3f}, "
                f"frame={frame_id}"
            )

    def update_pose_buffer_and_filter(
        self,
        track_key: str,
        center: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        if track_key not in self.person_states:
            return None

        state = self.person_states[track_key]
        buffer = state.setdefault("pose_buffer", [])

        x, y, z = center
        buffer.append((float(x), float(y), float(z)))

        while len(buffer) > self.pose_filter_window_size:
            buffer.pop(0)

        if len(buffer) < self.pose_filter_window_size:
            return None

        points = np.array(buffer, dtype=np.float32)
        mean = np.mean(points, axis=0)
        dists = np.linalg.norm(points - mean, axis=1)

        drop_count = min(self.pose_filter_drop_count, len(points) - 1)
        if drop_count > 0:
            keep_indices = np.argsort(dists)[: len(points) - drop_count]
            filtered_points = points[keep_indices]
        else:
            filtered_points = points

        filtered_mean = np.mean(filtered_points, axis=0)

        return (
            float(filtered_mean[0]),
            float(filtered_mean[1]),
            float(filtered_mean[2]),
        )

    def publish_filtered_fallen_person_pose(
        self,
        center: Tuple[float, float, float],
        frame_id: str,
    ) -> bool:
        x, y, z = center

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id if frame_id else "camera_color_optical_frame"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0

        self.fallen_pose_pub.publish(pose_msg)
        return True

    def get_bbox_xywh(self, det) -> Optional[Tuple[float, float, float, float]]:
        try:
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            w = float(det.bbox.size.x)
            h = float(det.bbox.size.y)
            return cx, cy, w, h
        except Exception:
            return None

    def xywh_to_xyxy(
        self, cx: float, cy: float, w: float, h: float, img_w: int, img_h: int
    ):
        x1 = int(max(0, cx - w / 2.0))
        y1 = int(max(0, cy - h / 2.0))
        x2 = int(min(img_w - 1, cx + w / 2.0))
        y2 = int(min(img_h - 1, cy + h / 2.0))
        return x1, y1, x2, y2

    def parse_keypoints(self, det) -> Dict[int, dict]:
        kp_dict = {}
        try:
            raw_ids = [int(kp.id) for kp in det.keypoints.data]

            use_one_based = False
            if raw_ids and min(raw_ids) >= 1 and max(raw_ids) <= 17:
                use_one_based = True

            for kp in det.keypoints.data:
                kpt_id = int(kp.id)
                if use_one_based:
                    kpt_id -= 1

                kp_dict[kpt_id] = {
                    "x": float(kp.point.x),
                    "y": float(kp.point.y),
                    "score": float(kp.score),
                }
        except Exception:
            pass
        return kp_dict

    def get_kpt(
        self,
        kp_dict: Dict[int, dict],
        kpt_id: int,
        score_thr: float = 0.3,
    ) -> Optional[Tuple[float, float]]:
        if kpt_id not in kp_dict:
            return None
        if kp_dict[kpt_id]["score"] < score_thr:
            return None
        return kp_dict[kpt_id]["x"], kp_dict[kpt_id]["y"]

    def get_pair_center_flexible(
        self,
        kp_dict: Dict[int, dict],
        left_id: int,
        right_id: int,
        score_thr: float = 0.3,
    ) -> Optional[Tuple[float, float]]:
        left = self.get_kpt(kp_dict, left_id, score_thr)
        right = self.get_kpt(kp_dict, right_id, score_thr)

        if left is not None and right is not None:
            return ((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0)

        if left is not None:
            return left

        if right is not None:
            return right

        return None

    def filter_keypoints_for_pose(self, raw_kp_dict: Dict[int, dict]) -> Dict[int, dict]:
        filtered = {
            kpt_id: value.copy()
            for kpt_id, value in raw_kp_dict.items()
            if kpt_id >= 5
        }

        # 기존 코드에서는 얼굴과 어깨가 가까우면 어깨를 삭제했고,
        # 어깨/엉덩이 간격이 좁으면 해당 keypoint를 삭제했다.
        # 낙상 자세에서는 이런 조건 때문에 실제 어깨/엉덩이가 사라질 수 있어 제거했다.
        return filtered

    def get_torso_depth(
        self,
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
    ) -> Tuple[Optional[float], int]:
        if self.depth_image is None:
            return None, 0

        if shoulder_center is None or hip_center is None:
            return None, 0

        mid_x = int((shoulder_center[0] + hip_center[0]) / 2.0)
        mid_y = int((shoulder_center[1] + hip_center[1]) / 2.0)

        h, w = self.depth_image.shape[:2]
        half = self.depth_patch_size // 2

        x1 = max(0, mid_x - half)
        x2 = min(w, mid_x + half + 1)
        y1 = max(0, mid_y - half)
        y2 = min(h, mid_y + half + 1)

        patch = self.depth_image[y1:y2, x1:x2]
        if patch.size == 0:
            return None, 0

        valid = patch[(patch >= self.depth_min_mm) & (patch <= self.depth_max_mm)]
        valid_count = int(valid.size)

        if valid_count < self.min_depth_valid_count:
            return None, valid_count

        depth_mm = float(np.percentile(valid, 10))
        return depth_mm / 1000.0, valid_count

    def evaluate_pose_validity(
        self,
        kp_dict: Dict[int, dict],
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
        depth_valid_count: int,
        bbox_w: float,
        bbox_h: float,
        img_w: int,
        img_h: int,
    ) -> Tuple[bool, str]:
        valid_body_ids = [
            kpt_id for kpt_id, v in kp_dict.items()
            if v["score"] >= self.kpt_score_thr
        ]

        left_shoulder = self.get_kpt(kp_dict, 5, self.kpt_score_thr)
        right_shoulder = self.get_kpt(kp_dict, 6, self.kpt_score_thr)

        if left_shoulder is None and right_shoulder is None:
            return False, "missing_all_shoulders"

        left_hip = self.get_kpt(kp_dict, 11, self.kpt_score_thr)
        right_hip = self.get_kpt(kp_dict, 12, self.kpt_score_thr)

        if left_hip is None and right_hip is None:
            return False, "missing_all_hips"

        if shoulder_center is None:
            return False, "missing_shoulder_center"

        if hip_center is None:
            return False, "missing_hip_center"

        if len(valid_body_ids) < 4:
            return False, "too_few_body_keypoints"

        if self.require_depth_for_validity:
            if depth_valid_count < self.min_depth_valid_count:
                return False, "poor_depth"

        if bbox_w > img_w * 0.92 or bbox_h > img_h * 0.92:
            return False, "too_close_or_cropped"

        if depth_valid_count < self.min_depth_valid_count:
            return True, "valid_pose_no_depth"

        return True, "valid"

    def compute_pose_features(
        self,
        kp_dict: Dict[int, dict],
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
        torso_depth_m: Optional[float],
    ) -> Dict[str, Optional[float]]:
        features = {
            "trunk_len": None,
            "trunk_vertical_ratio": None,
            "vertical_span": None,
            "horizontal_span": None,
            "posture_compactness": None,
            "torso_depth_m": torso_depth_m,
            "abnormal_score": 0.0,
        }

        valid_points: List[Tuple[float, float]] = []
        for _, v in kp_dict.items():
            if v["score"] >= self.kpt_score_thr:
                valid_points.append((v["x"], v["y"]))

        if len(valid_points) >= 2:
            xs = [p[0] for p in valid_points]
            ys = [p[1] for p in valid_points]

            horizontal_span = max(xs) - min(xs)
            vertical_span = max(ys) - min(ys)

            features["horizontal_span"] = float(horizontal_span)
            features["vertical_span"] = float(vertical_span)

            if horizontal_span > 1e-6:
                features["posture_compactness"] = float(vertical_span / horizontal_span)

        if shoulder_center is not None and hip_center is not None:
            dx = hip_center[0] - shoulder_center[0]
            dy = hip_center[1] - shoulder_center[1]

            trunk_len = math.sqrt(dx * dx + dy * dy)
            trunk_vertical_ratio = abs(dy) / (abs(dx) + abs(dy) + 1e-6)

            features["trunk_len"] = float(trunk_len)
            features["trunk_vertical_ratio"] = float(trunk_vertical_ratio)

        abnormal_score = 0.0

        tvr = features["trunk_vertical_ratio"]
        if tvr is not None:
            if tvr < 0.45:
                abnormal_score += 2.0
            elif tvr < 0.60:
                abnormal_score += 1.0

        vspan = features["vertical_span"]
        if vspan is not None:
            if vspan < 120:
                abnormal_score += 2.0
            elif vspan < 180:
                abnormal_score += 1.0

        compactness = features["posture_compactness"]
        if compactness is not None:
            if compactness < 0.85:
                abnormal_score += 1.0

        features["abnormal_score"] = abnormal_score
        return features

    def classify_frame_state(
        self,
        validity: bool,
        features: Dict[str, Optional[float]],
    ) -> str:
        if validity and features.get("abnormal_score", 0.0) >= 2.5:
            return "ABNORMAL_POSTURE"
        return "NORMAL"

    def assign_track(self, cx: float, cy: float, now: float) -> str:
        best_key = None
        best_dist = float("inf")

        for key, state in self.person_states.items():
            if now - state["last_seen"] > self.track_timeout:
                continue

            prev_cx = state["last_cx"]
            prev_cy = state["last_cy"]
            dist = math.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)

            if dist < best_dist and dist < self.match_distance_px:
                best_dist = dist
                best_key = key

        if best_key is None:
            best_key = f"person_{self.next_track_id}"
            self.next_track_id += 1
            self.person_states[best_key] = {
                "last_cx": cx,
                "last_cy": cy,
                "last_seen": now,
                "last_state": "NORMAL",
                "abnormal_since": None,
                "normal_since": now,
                "fall_confirmed": False,
                "pose_buffer": [],
                "fall_pose_published": False,
                "last_torso_uv": None,
                "last_torso_depth_m": None,
                "last_depth_valid_count": 0,
                "last_camera_point": None,
                "last_camera_point_frame": None,
                "last_pose_candidate_time": 0.0,
            }
        else:
            self.person_states[best_key]["last_cx"] = cx
            self.person_states[best_key]["last_cy"] = cy
            self.person_states[best_key]["last_seen"] = now

        return best_key

    def update_person_state(self, person_key: str, frame_state: str, now: float) -> str:
        state = self.person_states[person_key]

        if frame_state == "ABNORMAL_POSTURE":
            if state["abnormal_since"] is None:
                state["abnormal_since"] = now

            state["normal_since"] = None

            if (now - state["abnormal_since"]) >= self.fall_hold_time:
                if not state["fall_confirmed"]:
                    state["pose_buffer"] = []
                    state["fall_pose_published"] = False

                state["fall_confirmed"] = True
                state["last_state"] = "FALL_DETECTED"
                return "FALL_DETECTED"

            state["last_state"] = "ABNORMAL_POSTURE"
            return "ABNORMAL_POSTURE"

        state["abnormal_since"] = None

        if state["fall_confirmed"]:
            if state["normal_since"] is None:
                state["normal_since"] = now

            if (now - state["normal_since"]) >= self.normal_recover_time:
                state["fall_confirmed"] = False
                state["pose_buffer"] = []
                state["fall_pose_published"] = False
                state["last_state"] = "NORMAL"
                return "NORMAL"

            state["last_state"] = "FALL_DETECTED"
            return "FALL_DETECTED"

        state["normal_since"] = now
        state["last_state"] = "NORMAL"
        return "NORMAL"

    def map_display_state(self, final_state: str, frame_state: str) -> str:
        if final_state == "FALL_DETECTED":
            return "DANGEROUS"
        if frame_state == "ABNORMAL_POSTURE":
            return "DETECTING"
        return "NORMAL"

    def publish_debug_image(self, img: np.ndarray):
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            if self.color_header is not None:
                msg.header = self.color_header
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"debug image publish failed: {e}")

    def draw_person_overlay(
        self,
        img: np.ndarray,
        bbox: Tuple[int, int, int, int],
        kp_dict: Dict[int, dict],
        label_1: str,
        label_2: str,
        state: str,
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
    ):
        x1, y1, x2, y2 = bbox
        state_color = self.get_state_color(state)

        cv2.rectangle(img, (x1, y1), (x2, y2), state_color, 2)

        self.draw_skeleton_multicolor(img, kp_dict)
        self.draw_core_pose(img, shoulder_center, hip_center)

        box_h = 42
        box_w = max(180, len(label_1) * 10)
        tx1 = x1
        ty1 = max(0, y1 - box_h - 6)
        tx2 = min(img.shape[1] - 1, tx1 + box_w)
        ty2 = min(img.shape[0] - 1, ty1 + box_h)

        cv2.rectangle(img, (tx1, ty1), (tx2, ty2), state_color, -1)
        cv2.putText(
            img,
            label_1,
            (tx1 + 6, ty1 + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            label_2,
            (tx1 + 6, ty1 + 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def get_state_color(self, state: str) -> Tuple[int, int, int]:
        if state == "DANGEROUS":
            return (0, 0, 255)
        if state == "DETECTING":
            return (0, 255, 255)
        return (0, 255, 0)

    def get_kpt_group_color(self, kpt_id: int) -> Tuple[int, int, int]:
        if kpt_id in [5, 7, 9]:
            return (255, 0, 0)
        if kpt_id in [6, 8, 10]:
            return (0, 165, 255)
        if kpt_id in [11, 12]:
            return (255, 0, 255)
        if kpt_id in [13, 15]:
            return (0, 255, 0)
        if kpt_id in [14, 16]:
            return (0, 0, 255)
        return (220, 220, 220)

    def draw_skeleton_multicolor(self, img: np.ndarray, kp_dict: Dict[int, dict]):
        skeleton_segments = [
            ((5, 6), (255, 255, 255)),
            ((5, 11), (255, 0, 255)),
            ((6, 12), (255, 0, 255)),
            ((11, 12), (255, 255, 255)),
            ((5, 7), (255, 0, 0)),
            ((7, 9), (255, 0, 0)),
            ((6, 8), (0, 165, 255)),
            ((8, 10), (0, 165, 255)),
            ((11, 13), (0, 255, 0)),
            ((13, 15), (0, 255, 0)),
            ((12, 14), (0, 0, 255)),
            ((14, 16), (0, 0, 255)),
        ]

        for (a, b), line_color in skeleton_segments:
            p1 = self.get_kpt(kp_dict, a, self.kpt_score_thr)
            p2 = self.get_kpt(kp_dict, b, self.kpt_score_thr)
            if p1 is None or p2 is None:
                continue

            cv2.line(
                img,
                (int(p1[0]), int(p1[1])),
                (int(p2[0]), int(p2[1])),
                line_color,
                2,
            )

        for kpt_id, v in kp_dict.items():
            if kpt_id < 5:
                continue
            if v["score"] < self.kpt_score_thr:
                continue

            px = int(v["x"])
            py = int(v["y"])
            point_color = self.get_kpt_group_color(kpt_id)

            cv2.circle(img, (px, py), 4, point_color, -1)
            cv2.putText(
                img,
                str(kpt_id),
                (px + 4, py - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                point_color,
                1,
                cv2.LINE_AA,
            )

    def draw_core_pose(
        self,
        img: np.ndarray,
        shoulder_center: Optional[Tuple[float, float]],
        hip_center: Optional[Tuple[float, float]],
    ):
        core_color = (255, 255, 255)

        if shoulder_center is not None:
            sx, sy = int(shoulder_center[0]), int(shoulder_center[1])
            cv2.circle(img, (sx, sy), 6, core_color, -1)
            cv2.putText(
                img,
                "SC",
                (sx + 6, sy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                core_color,
                1,
                cv2.LINE_AA,
            )

        if hip_center is not None:
            hx, hy = int(hip_center[0]), int(hip_center[1])
            cv2.circle(img, (hx, hy), 6, core_color, -1)
            cv2.putText(
                img,
                "HC",
                (hx + 6, hy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                core_color,
                1,
                cv2.LINE_AA,
            )

        if shoulder_center is not None and hip_center is not None:
            cv2.line(
                img,
                (int(shoulder_center[0]), int(shoulder_center[1])),
                (int(hip_center[0]), int(hip_center[1])),
                core_color,
                2,
            )

    def cleanup_tracks(self, now: float):
        remove_keys = []
        for key, state in self.person_states.items():
            if now - state["last_seen"] > self.track_timeout:
                remove_keys.append(key)

        for key in remove_keys:
            del self.person_states[key]


def main(args=None):
    rclpy.init(args=args)
    node = FallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
