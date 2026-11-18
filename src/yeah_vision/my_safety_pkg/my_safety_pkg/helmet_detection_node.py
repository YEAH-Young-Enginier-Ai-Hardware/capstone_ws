import json
import os
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from ultralytics import YOLO


class HelmetDetectionNode(Node):
    def __init__(self):
        super().__init__("helmet_detection_node")

        self.bridge = CvBridge()
        self.package_name = "my_safety_pkg"

        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("debug_image_topic", "/helmet_detection/debug_image")
        self.declare_parameter("result_topic", "/helmet_detection/result")
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_thres", 0.20)
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("half", True)

        # 기본 실행값:
        # process_every_n_frames=1
        # imgsz=416
        # conf_thres=0.20
        # publish_debug_image=False
        # device=cuda:0
        # half=True

        # 디버그 이미지는 기본적으로 끔
        # 웹 대시보드는 /helmet_detection/result만 사용
        self.declare_parameter("publish_debug_image", False)

        # 이벤트 이미지 저장 및 공통 이벤트 토픽
        self.declare_parameter("safety_event_topic", "/safety/event")

        # 안전모 미착용 음성 재생용 토픽
        # speaker_warning_node가 이 토픽을 받아 helmet.mp3를 재생한다.
        self.declare_parameter("helmet_warning_topic", "/helmet_warning")

        self.declare_parameter("event_image_dir", os.path.expanduser("~/safety_event_images"))

        self.declare_parameter("event_image_base_url", "http://localhost:8090")

        self.input_topic = self.get_parameter("input_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.process_every_n_frames = int(
            self.get_parameter("process_every_n_frames").value
        )
        self.device = str(self.get_parameter("device").value).strip()
        self.half = bool(self.get_parameter("half").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.safety_event_topic = str(self.get_parameter("safety_event_topic").value)
        self.helmet_warning_topic = str(
            self.get_parameter("helmet_warning_topic").value
        )
        self.event_image_dir = str(self.get_parameter("event_image_dir").value)
        self.event_image_base_url = str(
            self.get_parameter("event_image_base_url").value
        ).rstrip("/")
        os.makedirs(self.event_image_dir, exist_ok=True)

        # 안전모 미착용 이벤트 중복 방지용 상태
        # WITHOUT_HELMET 상태가 지속되는 동안은 1번만 저장/발행한다.
        # WITH_HELMET 또는 NO_HELMET_DETECTION으로 돌아오면 다시 새 이벤트를 허용한다.
        self.no_helmet_event_active = False

        if self.process_every_n_frames < 1:
            self.process_every_n_frames = 1

        param_model_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = self.resolve_model_path(param_model_path)

        self.model = YOLO(self.model_path)

        # 실시간 처리를 위해 오래된 이미지가 큐에 쌓이지 않도록 설정
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos,
        )

        # 결과 문자열은 가벼우므로 일반 QoS 사용
        self.result_pub = self.create_publisher(
            String,
            self.result_topic,
            10,
        )

        # 낙상/안전모 공통 이벤트 이력용 토픽
        self.safety_event_pub = self.create_publisher(
            String,
            self.safety_event_topic,
            10,
        )

        # 안전모 미착용 음성 경고용 토픽
        self.helmet_warning_pub = self.create_publisher(
            Bool,
            self.helmet_warning_topic,
            10,
        )

        # 디버그 이미지 publish를 사용할 때만 publisher 생성
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                1,
            )

        self.frame_count = 0

        self.get_logger().info("helmet_detection_node started")
        self.get_logger().info(f"input_topic: {self.input_topic}")
        self.get_logger().info(f"result_topic: {self.result_topic}")
        self.get_logger().info(f"safety_event_topic: {self.safety_event_topic}")
        self.get_logger().info(f"helmet_warning_topic: {self.helmet_warning_topic}")
        self.get_logger().info(f"event_image_dir: {self.event_image_dir}")
        self.get_logger().info(f"event_image_base_url: {self.event_image_base_url}")
        self.get_logger().info(f"model_path: {self.model_path}")

    def resolve_model_path(self, param_model_path: str) -> str:
        candidates = []

        if param_model_path:
            candidates.append(Path(param_model_path).expanduser())

        # 현재 프로젝트 워크스페이스를 먼저 확인한다.
        # 이전 코드처럼 ~/ros2_ws를 먼저 보면 예전 best.pt가 로드될 수 있다.
        candidates.append(
            Path.home() / "yeah_ws" / "src" / self.package_name / "models" / "best.pt"
        )

        try:
            share_dir = Path(get_package_share_directory(self.package_name))
            candidates.append(share_dir / "models" / "best.pt")
        except Exception:
            pass

        # 예전 워크스페이스는 fallback으로만 확인한다.
        candidates.append(
            Path.home() / "ros2_ws" / "src" / self.package_name / "models" / "best.pt"
        )

        for path in candidates:
            if path.exists():
                return str(path)

        candidate_text = "\n".join([str(p) for p in candidates])
        self.get_logger().error(
            "모델 파일을 찾지 못했습니다. 아래 경로들을 확인하세요:\n" + candidate_text
        )
        raise FileNotFoundError("best.pt not found")

    def normalize_label(self, label: str) -> str:
        return (
            str(label)
            .strip()
            .lower()
            .replace(" ", "_")
            .replace("-", "_")
        )

    def classify_detection(self, norm_name: str) -> str:
        if "without" in norm_name or "no_helmet" in norm_name:
            return "without"
        if "with" in norm_name or norm_name == "helmet":
            return "with"
        return "other"

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.process_every_n_frames > 1:
            if self.frame_count % self.process_every_n_frames != 0:
                return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        try:
            predict_kwargs = {
                "source": frame,
                "conf": self.conf_thres,
                "imgsz": self.imgsz,
                "verbose": False,
            }

            if self.device:
                predict_kwargs["device"] = self.device

            # CUDA 사용 시 FP16 추론을 시도해 GPU 부하를 줄인다.
            # CPU에서는 half가 동작하지 않을 수 있으므로 cuda 장치일 때만 적용한다.
            if self.half and self.device.startswith("cuda"):
                predict_kwargs["half"] = True

            results = self.model.predict(**predict_kwargs)

        except Exception as e:
            self.get_logger().error(f"모델 추론 실패: {e}")
            return

        if not results:
            self.no_helmet_event_active = False
            self.publish_result(
                overall_status="NO_HELMET_DETECTION",
                with_helmet_count=0,
                without_helmet_count=0,
                other_count=0,
            )
            return

        result = results[0]

        with_helmet_count = 0
        without_helmet_count = 0
        other_count = 0
        max_without_conf = 0.0

        # 디버그 이미지를 켠 경우에만 전체 디버그 이미지 복사본 생성
        annotated = frame.copy() if self.publish_debug_image else None

        # 이벤트 저장용 이미지는 안전모 미착용이 실제 검출될 때만 생성한다.
        event_annotated = None

        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                conf = float(box.conf[0].item())
                cls_id = int(box.cls[0].item())

                raw_name = result.names.get(cls_id, str(cls_id))
                norm_name = self.normalize_label(raw_name)

                det_type = self.classify_detection(norm_name)

                if det_type == "without":
                    display_name = "Without helmet"
                    color = (0, 0, 255)
                    without_helmet_count += 1
                    max_without_conf = max(max_without_conf, conf)
                elif det_type == "with":
                    display_name = "With helmet"
                    color = (0, 255, 0)
                    with_helmet_count += 1
                else:
                    display_name = str(raw_name).strip()
                    color = (0, 255, 255)
                    other_count += 1

                x1, y1, x2, y2 = box.xyxy[0].int().cpu().tolist()

                # 디버그 이미지를 켠 경우에만 bbox/텍스트 그림
                if self.publish_debug_image and annotated is not None:
                    self.draw_detection_box(
                        annotated,
                        x1,
                        y1,
                        x2,
                        y2,
                        color,
                        f"{display_name} {conf:.2f}",
                    )

                # 이벤트 이미지는 안전모 미착용 bbox만 빨간색으로 저장한다.
                if det_type == "without":
                    if event_annotated is None:
                        event_annotated = frame.copy()

                    self.draw_detection_box(
                        event_annotated,
                        x1,
                        y1,
                        x2,
                        y2,
                        (0, 0, 255),
                        f"{display_name} {conf:.2f}",
                    )

        if without_helmet_count > 0:
            overall_status = "WITHOUT_HELMET"
            overall_color = (0, 0, 255)
        elif with_helmet_count > 0:
            overall_status = "WITH_HELMET"
            overall_color = (0, 255, 0)
        else:
            overall_status = "NO_HELMET_DETECTION"
            overall_color = (255, 255, 0)

        summary_text = self.publish_result(
            overall_status=overall_status,
            with_helmet_count=with_helmet_count,
            without_helmet_count=without_helmet_count,
            other_count=other_count,
        )

        # 안전모 미착용이 새로 발생한 순간에만 이미지 저장, /safety/event, /helmet_warning 발행
        if overall_status == "WITHOUT_HELMET":
            if event_annotated is None:
                event_annotated = frame.copy()

            cv2.putText(
                event_annotated,
                summary_text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                overall_color,
                2,
                cv2.LINE_AA,
            )

            self.publish_helmet_event_if_needed(
                event_frame=event_annotated,
                confidence=max_without_conf,
                without_helmet_count=without_helmet_count,
                with_helmet_count=with_helmet_count,
                other_count=other_count,
            )
        else:
            self.no_helmet_event_active = False

        if self.publish_debug_image and annotated is not None:
            cv2.putText(
                annotated,
                summary_text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                overall_color,
                2,
                cv2.LINE_AA,
            )

            self.publish_debug_frame(annotated, msg.header)

    def draw_detection_box(
        self,
        img,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color,
        text: str,
    ):
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            img,
            text,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

    def publish_helmet_event_if_needed(
        self,
        event_frame,
        confidence: float,
        without_helmet_count: int,
        with_helmet_count: int,
        other_count: int,
    ):
        if self.no_helmet_event_active:
            return

        image_url = self.save_event_image(event_frame)

        event_data = {
            "type": "helmet_missing",
            "status": "WITHOUT_HELMET",
            "confidence": float(confidence),
            "image_url": image_url,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "description": "안전모 미착용 상태가 감지되었습니다.",
            "with": int(with_helmet_count),
            "without": int(without_helmet_count),
            "other": int(other_count),
        }

        msg = String()
        msg.data = json.dumps(event_data, ensure_ascii=False)
        self.safety_event_pub.publish(msg)

        warning_msg = Bool()
        warning_msg.data = True
        self.helmet_warning_pub.publish(warning_msg)

        self.get_logger().warn(f"helmet event published: {image_url}")
        self.get_logger().warn("helmet warning topic published")

        self.no_helmet_event_active = True

    def save_event_image(self, frame) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"helmet_WITHOUT_HELMET_{timestamp}.jpg"
        image_path = os.path.join(self.event_image_dir, filename)

        saved = cv2.imwrite(image_path, frame)

        if not saved:
            self.get_logger().error(f"helmet event image save failed: {image_path}")
            return ""

        return f"{self.event_image_base_url}/{filename}"

    def publish_result(
        self,
        overall_status: str,
        with_helmet_count: int,
        without_helmet_count: int,
        other_count: int,
    ) -> str:
        summary_text = (
            f"{overall_status} | "
            f"with={with_helmet_count}, "
            f"without={without_helmet_count}, "
            f"other={other_count}"
        )

        result_msg = String()
        result_msg.data = summary_text
        self.result_pub.publish(result_msg)

        return summary_text

    def publish_debug_frame(self, frame, header):
        if self.debug_image_pub is None:
            return

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"디버그 이미지 publish 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HelmetDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
