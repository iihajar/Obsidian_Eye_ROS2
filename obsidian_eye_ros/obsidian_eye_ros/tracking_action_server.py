#!/usr/bin/env python3

import asyncio
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from obsidian_eye_interfaces.action import TrackTarget
from obsidian_eye_interfaces.msg import YOLOData


class TrackingActionServer(Node):

    def __init__(self):
        super().__init__('tracking_action_server')

        self.target = None

        # Receive the latest YOLO detection
        self.create_subscription(
            YOLOData,
            '/yolo/data',
            self.yolo_callback,
            10
        )

        # Receive tracking requests
        self.action_server = ActionServer(
            self,
            TrackTarget,
            'track_target',
            self.execute_callback
        )

        self.get_logger().info(
            'Tracking Action Server started'
        )

    def yolo_callback(self, msg):
        self.target = msg

    # Send tracking feedback to the Decision Engine
    async def execute_callback(self, goal_handle):
        start_time = time.time()
        feedback = TrackTarget.Feedback()

        for _ in range(10):

            if self.target and self.target.detected:
                feedback.center_x = self.target.center_x
                feedback.center_y = self.target.center_y
                feedback.bbox_width = self.target.width
                feedback.bbox_height = self.target.height
                feedback.confidence = self.target.confidence
                feedback.tracking_status = 'TRACKING'

                goal_handle.publish_feedback(feedback)

            await asyncio.sleep(1)

        goal_handle.succeed()

        result = TrackTarget.Result()
        result.success = True
        result.message = 'Tracking finished'
        result.tracking_duration_sec = (
            time.time() - start_time
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = TrackingActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()