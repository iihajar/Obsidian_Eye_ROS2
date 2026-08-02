#!/usr/bin/env python3
"""Acoustic Sensor Node for Obsidian Eye.

NOTE: This node currently SIMULATES acoustic readings using random
values. Replace `check_acoustic_sound` with real microphone + signal
processing code when hardware is available.
"""

import random

import rclpy
from rclpy.node import Node

from obsidian_eye_interfaces.msg import AcousticData

# (sound_type, min_level, max_level) used for simulation only
SIMULATED_SOUND_PROFILES = (
    ('propeller', 70, 100),
    ('car', 40, 60),
    ('wind', 20, 50),
    ('noise', 10, 55),
    ('none', 0, 0),
)


class AcousticSensorNode(Node):

    def __init__(self):
        super().__init__('acoustic_sensor_node')

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('detection_threshold', 0.70)

        update_rate = self.get_parameter('update_rate_hz').value
        self.threshold = float(
            self.get_parameter('detection_threshold').value
        )

        self.publisher = self.create_publisher(
            AcousticData,
            '/acoustic/data',
            10
        )

        self.timer = self.create_timer(
            1.0 / update_rate,
            self.check_acoustic_sound
        )

        self.get_logger().info(
            f'Acoustic Sensor Node started (simulated) | '
            f'rate={update_rate} Hz | threshold={self.threshold}'
        )

    def check_acoustic_sound(self):
        # --- SIMULATION ONLY: replace with real mic reading ---
        sound_type, min_level, max_level = random.choice(
            SIMULATED_SOUND_PROFILES
        )
        sound_level = random.randint(min_level, max_level)
        # ---------------------------------------------------

        confidence = sound_level / 100.0
        # A propeller reading is always simulated in the 70-100
        # range, so it always clears the threshold by construction.
        # Once real audio classification is wired in, this check
        # will do real work.
        detected = (
            sound_type == 'propeller'
            and confidence >= self.threshold
        )

        message = AcousticData()

        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = 'acoustic_sensor'
        message.frequency_hz = 0.0
        message.amplitude = float(sound_level)
        message.confidence = float(confidence) if detected else 0.0
        message.sound_type = sound_type

        self.publisher.publish(message)

        self.get_logger().info(
            f'Sound: {sound_type} | '
            f'Level: {sound_level} dB | '
            f'Detected: {detected} | '
            f'Confidence: {message.confidence * 100:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)

    node = AcousticSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
