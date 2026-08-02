#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from obsidian_eye_interfaces.action import TrackTarget
from obsidian_eye_interfaces.msg import (
    AcousticData,
    DecisionStatus,
    RFData,
    YOLOData,
)


class DecisionEngineNode(Node):

    def __init__(self):
        super().__init__('decision_engine_node')

        # Current system state
        self.state = 'MONITORING'
        self.tracking_goal = None
        self.state_entered_at = self.get_clock().now()

        # If VERIFYING/TRACKING gets stuck (e.g. camera never responds),
        # auto-return to MONITORING after this many seconds instead of
        # hanging forever and requiring a manual reset.
        self.declare_parameter('verification_timeout_sec', 15.0)
        self.declare_parameter('tracking_timeout_sec', 60.0)

        # Send commands to Camera + YOLO
        self.vision_publisher = self.create_publisher(
            String,
            '/vision/command',
            10
        )

        # Publish decisions for the Mission Node
        self.decision_publisher = self.create_publisher(
            DecisionStatus,
            '/decision/status',
            10
        )

        # Receive sensor and vision results
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

        # Send tracking goals to the Action Server
        self.tracking_client = ActionClient(
            self,
            TrackTarget,
            'track_target'
        )

        self.timeout_timer = self.create_timer(
            1.0,
            self.check_state_timeout
        )

        self.get_logger().info(
            'Decision Engine started | MONITORING'
        )

    # RF requests visual verification
    def rf_callback(self, msg):
        self.get_logger().info(

        f"State={self.state}, Source={msg.source_type}, Confidence={msg.confidence}")
        if (
            self.state == 'MONITORING'
            and msg.source_type == 'Drone'
            and msg.confidence >= 0.70
        ):
            self.get_logger().warning('RF Triggered')
            self.start_verification(msg.confidence, 'RF')

    # Acoustic sensor requests visual verification
    def acoustic_callback(self, msg):
        self.get_logger().info(

        f"State={self.state}, Sound={msg.sound_type}, Confidence={msg.confidence}")
        if (
            self.state == 'MONITORING'
            and msg.sound_type == 'propeller'
            and msg.confidence >= 0.70
        ):
            self.get_logger().warning('Acoustic Triggered')
            self.start_verification(
                msg.confidence,
                'Acoustic'
            )

    # Process Camera + YOLO results
    def yolo_callback(self, msg):
        if (
            self.state == 'VERIFYING'
            and msg.valid
            and msg.status == 'VERIFIED'
            and msg.confidence >= 0.70
        ):
            self.start_tracking(msg)

        elif (
            self.state == 'VERIFYING'
            and msg.status == 'FAILED'
        ):
            self.return_to_monitoring(
                'Verification failed'
            )

        elif (
            self.state == 'TRACKING'
            and msg.status == 'LOST'
        ):
            self.stop_tracking('Target lost')

    # Safety net: auto-recover if VERIFYING/TRACKING never resolves
    # (e.g. camera never publishes, message dropped, etc.)
    def check_state_timeout(self):
        if self.state == 'MONITORING':
            return

        elapsed = (
            self.get_clock().now() - self.state_entered_at
        ).nanoseconds / 1e9

        if self.state == 'VERIFYING':
            limit = self.get_parameter(
                'verification_timeout_sec'
            ).value
        elif self.state == 'TRACKING':
            limit = self.get_parameter(
                'tracking_timeout_sec'
            ).value
        else:
            return

        if elapsed >= limit:
            if self.tracking_goal is not None:
                self.tracking_goal.cancel_goal_async()
                self.tracking_goal = None

            self.return_to_monitoring(
                f'{self.state} timed out after {elapsed:.1f}s'
            )

    # Enable Camera + YOLO verification
    def start_verification(self, confidence, source):
        self.state = 'VERIFYING'
        self.state_entered_at = self.get_clock().now()

        self.send_vision_command('VERIFY')

        self.publish_decision(
            decision='SUSPICIOUS_OBJECT',
            confidence=confidence,
            threat_level='MEDIUM',
            action='VERIFY_TARGET'
        )

        self.get_logger().warning(
            f'{source}: {confidence * 100:.1f}% | VERIFYING'
        )

    # Start tracking after YOLO confirmation
    def start_tracking(self, yolo_msg):
        if not self.tracking_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.return_to_monitoring(
                'Tracking server unavailable'
            )
            return

        self.state = 'TRACKING'
        self.state_entered_at = self.get_clock().now()

        self.send_vision_command('TRACK')

        self.publish_decision(
            decision='CONFIRMED_DRONE',
            confidence=yolo_msg.confidence,
            threat_level='HIGH',
            action='PAUSE_MISSION_AND_TRACK'
        )

        goal = TrackTarget.Goal()
        goal.target_id = 'target_1'
        goal.class_name = yolo_msg.class_name or 'drone'

        future = self.tracking_client.send_goal_async(goal)
        future.add_done_callback(
            self.goal_response_callback
        )

        self.get_logger().warning(
            'Target confirmed | TRACKING'
        )

    # Check whether the tracking goal was accepted
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.return_to_monitoring(
                f'Tracking request failed: {error}'
            )
            return

        if not goal_handle.accepted:
            self.return_to_monitoring(
                'Tracking goal rejected'
            )
            return

        self.tracking_goal = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.tracking_result_callback
        )

        self.get_logger().info(
            'Tracking goal accepted'
        )

    # Return to monitoring when tracking finishes
    def tracking_result_callback(self, future):
        self.tracking_goal = None
        self.return_to_monitoring(
            'Tracking finished'
        )

    # Cancel active tracking when the target is lost
    def stop_tracking(self, reason):
        if self.tracking_goal is not None:
            self.tracking_goal.cancel_goal_async()
            self.tracking_goal = None

        self.return_to_monitoring(reason)

    # Stop vision processing and resume the mission
    def return_to_monitoring(self, reason):
        self.state = 'MONITORING'
        self.state_entered_at = self.get_clock().now()

        self.send_vision_command('IDLE')

        self.publish_decision(
            decision='NO_CONFIRMED_THREAT',
            confidence=0.0,
            threat_level='LOW',
            action='RESUME_MISSION'
        )

        self.get_logger().info(
            f'{reason} | MONITORING'
        )

    # Send a command to Camera + YOLO
    def send_vision_command(self, command):
        message = String()
        message.data = command

        self.vision_publisher.publish(message)

        self.get_logger().info(
            f'Vision command: {command}'
        )

    # Publish the current decision to the Mission Node
    def publish_decision(
        self,
        decision,
        confidence,
        threat_level,
        action
    ):
        message = DecisionStatus()

        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = 'decision_engine'
        message.decision = decision
        message.confidence = float(confidence * 100.0)
        message.threat_level = threat_level
        message.action = action

        self.decision_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionEngineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()