#!/usr/bin/env python3
"""Camera + YOLO Node for Obsidian Eye.

Runs YOLO on incoming camera frames while in VERIFYING or TRACKING
state, and publishes detection results + an annotated frame for the
dashboard.
"""

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from obsidian_eye_interfaces.msg import YOLOData

DRONE_CLASS_NAMES = ('drone', 'uav', 'quadcopter')


class CameraYoloNode(Node):
    """Runs YOLO detection and drives the VERIFYING/TRACKING state
    machine for vision.
    """

    def __init__(self):
        super().__init__('camera_yolo_node')

        # Configurable vision settings
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.70)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('show_camera', False)
        self.declare_parameter(
            'camera_topic',
            '/world/external_world/model/x500_depth_0/'
            'link/camera_link/sensor/IMX214/image'
        )

        # Verification / tracking tuning
        self.declare_parameter('required_detections', 3)
        self.declare_parameter('max_verification_frames', 10)
        self.declare_parameter('tracking_loss_grace_frames', 5)

        model_path = self.get_parameter('model_path').value
        camera_topic = self.get_parameter('camera_topic').value

        self.threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.device = self.get_parameter('device').value
        self.show_camera = bool(
            self.get_parameter('show_camera').value
        )
        self.required_detections = int(
            self.get_parameter('required_detections').value
        )
        self.max_verification_frames = int(
            self.get_parameter('max_verification_frames').value
        )
        self.tracking_loss_grace_frames = int(
            self.get_parameter('tracking_loss_grace_frames').value
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        # Vision state and verification counters
        self.state = 'IDLE'
        self.detection_count = 0
        self.verification_frames = 0
        self.missed_tracking_frames = 0
        self.target_verified = False

        # Receive VERIFY, TRACK, and IDLE commands
        self.create_subscription(
            String,
            '/vision/command',
            self.command_callback,
            10
        )

        # Receive camera frames from Gazebo
        self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish YOLO detection data
        self.yolo_publisher = self.create_publisher(
            YOLOData,
            '/yolo/data',
            10
        )

        # Publish annotated frames for the dashboard
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/camera/processed',
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Camera + YOLO started | Model: {model_path} | IDLE'
        )

    def command_callback(self, msg: String) -> None:
        """Change vision mode based on Decision Engine commands."""
        command = msg.data.strip().upper()

        if command == 'VERIFY':
            self.state = 'VERIFYING'
            self.detection_count = 0
            self.verification_frames = 0
            self.target_verified = False

            self.get_logger().info('Vision: VERIFYING')

        elif command == 'TRACK':
            if self.target_verified:
                self.state = 'TRACKING'
                self.missed_tracking_frames = 0
                self.get_logger().info('Vision: TRACKING')
            else:
                self.get_logger().warning(
                    'TRACK requested but target was never verified '
                    '- ignoring'
                )

        elif command == 'IDLE':
            self.reset_vision()

    def image_callback(self, image_msg: Image) -> None:
        """Process frames only during verification or tracking."""
        if self.state == 'IDLE':
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )

            result = self.model(
                frame,
                device=self.device,
                conf=self.threshold,
                verbose=False
            )[0]

            detection = self.find_target(result)

            message = self.create_yolo_message(
                image_msg,
                detection
            )

            self.update_status(message)
            self.yolo_publisher.publish(message)

            # Create the annotated frame once
            processed_frame = result.plot()

            # Publish the frame for the web dashboard
            processed_message = self.bridge.cv2_to_imgmsg(
                processed_frame,
                encoding='bgr8'
            )
            processed_message.header = image_msg.header

            self.processed_image_publisher.publish(
                processed_message
            )

            # Optional local OpenCV window
            if self.show_camera:
                cv2.imshow(
                    'Obsidian Eye - Camera + YOLO',
                    processed_frame
                )
                cv2.waitKey(1)

            if message.status in ('FAILED', 'LOST'):
                self.reset_vision()

        except Exception as error:
            self.get_logger().error(
                f'Vision error: {error}'
            )

    def find_target(self, result) -> dict | None:
        """Return the highest-confidence drone-class detection, if
        any.
        """
        best = None

        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(self.model.names[class_id])
            confidence = float(box.conf[0])

            if class_name.lower() not in DRONE_CLASS_NAMES:
                continue

            if best is not None and confidence <= best['confidence']:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            best = {
                'class_id': class_id,
                'class_name': class_name,
                'confidence': confidence,
                'center_x': (x1 + x2) / 2.0,
                'center_y': (y1 + y2) / 2.0,
                'width': x2 - x1,
                'height': y2 - y1,
            }

        return best

    def create_yolo_message(
        self,
        image_msg: Image,
        detection: dict | None
    ) -> YOLOData:
        """Convert detection results into a YOLOData message."""
        message = YOLOData()

        message.stamp = image_msg.header.stamp
        message.frame_id = image_msg.header.frame_id
        message.detected = detection is not None
        message.valid = False

        if detection:
            message.class_name = detection['class_name']
            message.class_id = detection['class_id']
            message.confidence = detection['confidence']
            message.center_x = detection['center_x']
            message.center_y = detection['center_y']
            message.width = detection['width']
            message.height = detection['height']

        else:
            message.class_name = ''
            message.class_id = -1
            message.confidence = 0.0
            message.center_x = 0.0
            message.center_y = 0.0
            message.width = 0.0
            message.height = 0.0

        return message

    def update_status(self, message: YOLOData) -> None:
        """Drive VERIFYING/TRACKING progress based on this frame's
        detection result.
        """
        if self.state == 'VERIFYING':
            self.verification_frames += 1

            if message.detected:
                self.detection_count += 1
            else:
                self.detection_count = 0

            if self.detection_count >= self.required_detections:
                self.target_verified = True
                message.valid = True
                message.status = 'VERIFIED'

                self.get_logger().warning(
                    f'Target verified '
                    f'{self.required_detections} times'
                )

            elif (
                self.verification_frames
                >= self.max_verification_frames
            ):
                message.status = 'FAILED'

                self.get_logger().info(
                    'Verification failed'
                )

            else:
                message.status = 'VERIFYING'

        elif self.state == 'TRACKING':
            message.valid = message.detected

            if message.detected:
                self.missed_tracking_frames = 0
                message.status = 'TRACKING'
            else:
                # Tolerate a few missed frames (occlusion, a bad
                # frame, etc.) before declaring the target lost -
                # a single miss used to cancel tracking immediately.
                self.missed_tracking_frames += 1

                if (
                    self.missed_tracking_frames
                    >= self.tracking_loss_grace_frames
                ):
                    message.status = 'LOST'
                    self.get_logger().warning('Target lost')
                else:
                    message.status = 'TRACKING'

    def reset_vision(self) -> None:
        """Return vision to the initial IDLE state."""
        self.state = 'IDLE'
        self.detection_count = 0
        self.verification_frames = 0
        self.missed_tracking_frames = 0
        self.target_verified = False

        if self.show_camera:
            cv2.destroyAllWindows()

        self.get_logger().info('Vision: IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node.show_camera:
            cv2.destroyAllWindows()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
