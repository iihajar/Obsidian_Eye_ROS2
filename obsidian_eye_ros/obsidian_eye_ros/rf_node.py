#!/usr/bin/env python3
"""RF Sensor Node for Obsidian Eye.

NOTE: This node currently SIMULATES RF readings using random values.
Replace `check_rf_signal` with real SDR/receiver code when hardware
is available.
"""

import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from obsidian_eye_interfaces.msg import RFData

# Common drone control/video frequencies (MHz), used for simulation only
SIMULATED_FREQUENCIES_MHZ = (2400.0, 5800.0, 915.0, 433.0)


class RFSensorNode(Node):

    def __init__(self):
        super().__init__('rf_sensor_node')

        # Configurable parameters (override via launch file)
        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('detection_threshold', 0.70)

        update_rate = self.get_parameter('update_rate_hz').value
        self.threshold = float(
            self.get_parameter('detection_threshold').value
        )

        self.publisher = self.create_publisher(
            RFData,
            '/rf/data',
            10
        )

        self.timer = self.create_timer(
            1.0 / update_rate,
            self.check_rf_signal
        )

        self.get_logger().info(
            f'RF Sensor Node started (simulated) | '
            f'rate={update_rate} Hz | threshold={self.threshold}'
        )

    def check_rf_signal(self):
        # --- SIMULATION ONLY: replace with real SDR reading ---
        frequency = random.choice(SIMULATED_FREQUENCIES_MHZ)
        signal_strength_pct = random.randint(0, 100)
        confidence = signal_strength_pct / 100.0
        # ---------------------------------------------------

        source_type = (
            'Drone' if confidence >= self.threshold else 'Unknown'
        )

        message = RFData()
        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = 'rf_sensor'

        message.frequency_mhz = frequency
        # Simulated strength as a 0-100 signal quality score.
        # NOTE: field is typed dBm in the .msg; real dBm values are
        # negative (e.g. -30 to -90). Convert properly once real
        # hardware is wired in.
        message.signal_strength_dbm = float(signal_strength_pct)
        message.confidence = confidence
        message.source_type = source_type

        message.position = Point()  # unknown until localization exists
        message.bearing_deg = 0.0   # unknown until localization exists

        self.publisher.publish(message)

        self.get_logger().info(
            f'RF: {source_type} | '
            f'{frequency:.0f} MHz | '
            f'Confidence: {confidence * 100:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RFSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
