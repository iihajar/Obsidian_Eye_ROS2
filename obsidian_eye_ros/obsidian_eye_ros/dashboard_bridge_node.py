#!/usr/bin/env python3

import cv2
import requests
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from obsidian_eye_interfaces.msg import (
    RFData,
    AcousticData,
    YOLOData,
    DecisionStatus,
    MissionStatus,
    PX4Telemetry,
)


class DashboardBridgeNode(Node):

    def __init__(self):
        super().__init__('dashboard_bridge_node')

        self.dashboard_url = (
            'http://127.0.0.1:8000/api/update_dashboard'
        )

        self.camera_url = (
            'http://127.0.0.1:8000/api/update_camera'
        )

        self.cv_bridge = CvBridge()

        self.dashboard = {
            'system_status': 'Online',
            'mission_status': 'Waiting',

            'battery': 0,
            'battery_voltage': 0.0,

            'gps': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
            },

            'altitude': 0.0,
            'ground_speed': 0.0,

            'flight_mode': 'Unknown',
            'connected': False,
            'armed': False,
            'in_mission': False,
            'px4_status': 'Waiting',

            'threat_level': 'None',

            'sensor_fusion': {
                'rf': 'Waiting',
                'acoustic': 'Waiting',
                'yolo': 'Waiting',
            },

            'decision_engine': {
                'decision': 'Waiting',
                'confidence': 0,
            },

            'timeline': [
                'Dashboard bridge started'
            ],
        }

        self.create_subscription(
            RFData,
            '/rf/data',
            self.rf_callback,
            10
        )

        self.create_subscription(
            AcousticData,
            '/acoustic/data',
            self.acoustic_callback,
            10
        )

        self.create_subscription(
            YOLOData,
            '/yolo/data',
            self.yolo_callback,
            10
        )

        self.create_subscription(
            DecisionStatus,
            '/decision/status',
            self.decision_callback,
            10
        )

        self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.mission_callback,
            10
        )

        self.create_subscription(
            PX4Telemetry,
            '/px4/telemetry',
            self.telemetry_callback,
            10
        )

        self.create_subscription(
            Image,
            '/camera/processed',
            self.camera_callback,
            qos_profile_sensor_data
        )

        # Send dashboard data every second
        self.create_timer(
            1.0,
            self.send_update
        )

        self.get_logger().info(
            'Dashboard Bridge started and connected'
        )

    def rf_callback(self, msg):
        confidence = round(
            float(msg.confidence) * 100,
            1
        )

        self.dashboard['sensor_fusion']['rf'] = (
            f'{msg.source_type} - {confidence}%'
        )

        self.get_logger().info(
            f'RF received: {msg.source_type} {confidence}%'
        )

    def acoustic_callback(self, msg):
        confidence = round(
            float(msg.confidence) * 100,
            1
        )

        self.dashboard['sensor_fusion']['acoustic'] = (
            f'{msg.sound_type} - {confidence}%'
        )

        self.get_logger().info(
            f'Acoustic received: '
            f'{msg.sound_type} {confidence}%'
        )

    def yolo_callback(self, msg):
        confidence = round(
            float(msg.confidence) * 100,
            1
        )

        if msg.detected:
            self.dashboard['sensor_fusion']['yolo'] = (
                f'{msg.class_name} - {confidence}%'
            )
        else:
            self.dashboard['sensor_fusion']['yolo'] = (
                msg.status
                if msg.status
                else 'No Target'
            )

        self.get_logger().info(
            f'YOLO received: {msg.status}'
        )

    def decision_callback(self, msg):
        decision = (
            msg.action
            if msg.action
            else msg.decision
        )

        confidence = round(
            float(msg.confidence) * 100,
            1
        )

        self.dashboard['decision_engine']['decision'] = (
            decision
        )

        self.dashboard['decision_engine']['confidence'] = (
            confidence
        )

        self.dashboard['threat_level'] = msg.threat_level

        self.get_logger().info(
            f'Decision received: {decision}'
        )

    def mission_callback(self, msg):
        if hasattr(msg, 'current_mission_state'):
            self.dashboard['mission_status'] = (
                msg.current_mission_state
            )

        elif hasattr(msg, 'status'):
            self.dashboard['mission_status'] = (
                msg.status
            )

        self.get_logger().info(
            'Mission status received'
        )

    def telemetry_callback(self, msg):
        self.dashboard['battery'] = round(
            float(msg.battery_percentage),
            1
        )

        self.dashboard['flight_mode'] = msg.flight_mode
        self.dashboard['armed'] = bool(msg.armed)
        self.dashboard['in_mission'] = bool(msg.in_mission)

        self.dashboard['altitude'] = round(
            float(msg.altitude_m),
            2
        )

        self.dashboard['gps'] = {
            'x': round(float(msg.position.x), 2),
            'y': round(float(msg.position.y), 2),
            'z': round(float(msg.position.z), 2),
        }

        self.dashboard['connected'] = True
        self.dashboard['px4_status'] = 'Connected'

        self.get_logger().info(
            f'Telemetry received: '
            f'battery={self.dashboard["battery"]}%'
        )

    def camera_callback(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            success, encoded_frame = cv2.imencode(
                '.jpg',
                frame
            )

            if not success:
                return

            requests.post(
                self.camera_url,
                files={
                    'file': (
                        'camera.jpg',
                        encoded_frame.tobytes(),
                        'image/jpeg',
                    )
                },
                timeout=0.5
            )

        except Exception as error:
            self.get_logger().warning(
                f'Camera error: {error}'
            )

    def send_update(self):
        try:
            response = requests.post(
                self.dashboard_url,
                json=self.dashboard,
                timeout=0.5
            )

            if response.status_code != 200:
                self.get_logger().warning(
                    f'Dashboard returned '
                    f'{response.status_code}'
                )

        except requests.RequestException:
            self.get_logger().warning(
                'FastAPI dashboard is not running'
            )


def main(args=None):
    rclpy.init(args=args)

    node = DashboardBridgeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()