#!/usr/bin/env python3
"""Mission Planner Node for Obsidian Eye.

Connects to PX4 via MAVSDK, uploads and runs a patrol mission, and
pauses/resumes it when the Decision Engine requests target tracking.
"""

import asyncio
import math
import threading

import rclpy
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import LandedState
from rclpy.node import Node

from obsidian_eye_interfaces.msg import (
    DecisionStatus,
    MissionStatus,
    PX4Telemetry,
)

from .flight_safety_node import FlightSafety

# --- Connection ---
PX4_SYSTEM_ADDRESS = 'udp://:14540'

# --- Mission profile ---
MISSION_ALTITUDE_M = 20.0
MISSION_SPEED_MPS = 40.0

# Rough conversion used for small local offsets: 1 unit ~= 1e-5 deg
# (~1.1 m near the equator). Good enough for a local patrol pattern,
# not for long-range navigation.
DEGREES_PER_UNIT = 1e-5

# Patrol waypoints as (x, y) offsets from the home position, in the
# same "units" as DEGREES_PER_UNIT above. Forms a closed loop.
MISSION_WAYPOINT_OFFSETS = (
    (0, 0),
    (0, -37),
    (37, -37),
    (37, -87),
    (25, -87),
    (25, -50),
    (0, -25),
    (0, 0),
)

DASHBOARD_PUBLISH_RATE_HZ = 1.0


class MissionPlannerNode(Node):
    """Runs the patrol mission and bridges PX4 telemetry to ROS 2."""

    def __init__(self):
        super().__init__('mission_node')

        self.drone = System()
        self.loop = None

        # Current PX4 values
        self.connected = False
        self.armed = False
        self.in_mission = False
        self.flight_mode = 'UNKNOWN'
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.altitude_m = 0.0
        self.ground_speed_mps = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0

        # Mission state values
        self.mission_state = 'WAITING'
        self.mission_progress = 0
        self.status_message = 'Waiting for PX4 connection'

        self.mission_publisher = self.create_publisher(
            MissionStatus,
            '/mission/status',
            10
        )

        self.telemetry_publisher = self.create_publisher(
            PX4Telemetry,
            '/px4/telemetry',
            10
        )

        # Receive commands from the Decision Engine
        self.create_subscription(
            DecisionStatus,
            '/decision/status',
            self.decision_callback,
            10
        )

        # Publish dashboard information every second
        self.create_timer(
            1.0 / DASHBOARD_PUBLISH_RATE_HZ,
            self.publish_dashboard_data
        )

        # Monitor battery and altitude during the mission
        self.flight_safety = FlightSafety(
            self.drone,
            self.get_logger()
        )

        # Run MAVSDK asyncio separately from ROS 2
        threading.Thread(
            target=self.run_async_loop,
            daemon=True
        ).start()

    def run_async_loop(self) -> None:
        """Start the asyncio loop that drives all MAVSDK calls."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.loop.create_task(self.start_mission())
        self.loop.run_forever()

    async def start_mission(self) -> None:
        """Entry point for the mission sequence.

        Wraps `_run_mission_sequence` so that any PX4/MAVSDK error
        (arm rejected, geofence violation, connection drop, etc.) is
        logged and reflected in mission_state instead of silently
        killing the asyncio task.
        """
        try:
            await self._run_mission_sequence()
        except Exception as error:
            self.get_logger().error(
                f'Mission sequence failed: {error}'
            )
            self.update_mission_state(
                'ERROR',
                f'Mission failed: {error}'
            )

    async def _run_mission_sequence(self) -> None:
        self.update_mission_state(
            'CONNECTING',
            'Connecting to PX4'
        )

        self.get_logger().info('Connecting to PX4...')

        await self.drone.connect(
            system_address=PX4_SYSTEM_ADDRESS
        )

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True

                self.update_mission_state(
                    'CONNECTED',
                    'PX4 connected'
                )

                self.get_logger().info('PX4 connected')
                break

        # Start continuous telemetry readers
        self.loop.create_task(self.monitor_battery())
        self.loop.create_task(self.monitor_flight_mode())
        self.loop.create_task(self.monitor_position())
        self.loop.create_task(self.monitor_velocity())
        self.loop.create_task(self.monitor_armed_state())
        self.loop.create_task(self.monitor_in_air())

        # Start safety monitoring
        self.loop.create_task(
            self.flight_safety.monitor()
        )

        self.update_mission_state(
            'UPLOADING',
            'Creating and uploading mission'
        )

        mission_plan = await self.create_mission()

        await self.drone.mission.set_return_to_launch_after_mission(
            True
        )

        await self.drone.mission.upload_mission(
            mission_plan
        )

        self.get_logger().info('Mission uploaded')

        self.update_mission_state(
            'ARMING',
            'Mission uploaded, arming drone'
        )

        await self.drone.action.arm()

        self.armed = True

        await self.drone.mission.start_mission()

        self.in_mission = True

        self.update_mission_state(
            'ACTIVE',
            'Mission started'
        )

        self.get_logger().info('Mission started')

        last_progress = None

        async for progress in self.drone.mission.mission_progress():
            if progress.total > 0:
                self.mission_progress = int(
                    progress.current / progress.total * 100
                )

            if progress.current != last_progress:
                self.get_logger().info(
                    f'Mission: {progress.current}/{progress.total}'
                )

                self.status_message = (
                    f'Mission waypoint '
                    f'{progress.current}/{progress.total}'
                )

                last_progress = progress.current

            if (
                progress.total > 0
                and progress.current == progress.total
            ):
                break

        self.in_mission = False

        self.update_mission_state(
            'RTL',
            'Mission finished, returning to launch'
        )

        async for landed_state in self.drone.telemetry.landed_state():
            if landed_state == LandedState.ON_GROUND:
                self.armed = False

                self.update_mission_state(
                    'COMPLETED',
                    'Mission completed'
                )

                self.get_logger().info('Mission completed')
                break

    async def monitor_battery(self) -> None:
        async for battery in self.drone.telemetry.battery():
            self.battery_percentage = (
                float(battery.remaining_percent) * 100.0
            )

            self.battery_voltage = float(
                battery.voltage_v
            )

    async def monitor_flight_mode(self) -> None:
        async for mode in self.drone.telemetry.flight_mode():
            self.flight_mode = str(mode).split('.')[-1]

    async def monitor_position(self) -> None:
        async for position in self.drone.telemetry.position():
            self.latitude_deg = float(
                position.latitude_deg
            )

            self.longitude_deg = float(
                position.longitude_deg
            )

            self.altitude_m = float(
                position.relative_altitude_m
            )

    async def monitor_velocity(self) -> None:
        async for velocity in self.drone.telemetry.velocity_ned():
            self.ground_speed_mps = math.sqrt(
                velocity.north_m_s ** 2
                + velocity.east_m_s ** 2
            )

    async def monitor_armed_state(self) -> None:
        async for armed in self.drone.telemetry.armed():
            self.armed = bool(armed)

    async def monitor_in_air(self) -> None:
        async for in_air in self.drone.telemetry.in_air():
            if not in_air and self.mission_state == 'ACTIVE':
                self.in_mission = False

    async def create_mission(self) -> MissionPlan:
        """Build the patrol mission relative to the current home
        position.
        """
        home = await anext(
            self.drone.telemetry.home()
        )

        items = [
            MissionItem(
                home.latitude_deg + x * DEGREES_PER_UNIT,
                home.longitude_deg + y * DEGREES_PER_UNIT,
                MISSION_ALTITUDE_M,
                MISSION_SPEED_MPS,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            )
            for x, y in MISSION_WAYPOINT_OFFSETS
        ]

        return MissionPlan(items)

    def decision_callback(self, msg: DecisionStatus) -> None:
        if self.loop is None:
            return

        if msg.action == 'PAUSE_MISSION_AND_TRACK':
            asyncio.run_coroutine_threadsafe(
                self.pause_mission(),
                self.loop
            )

        elif msg.action == 'RESUME_MISSION':
            asyncio.run_coroutine_threadsafe(
                self.resume_mission(),
                self.loop
            )

    async def pause_mission(self) -> None:
        await self.drone.mission.pause_mission()

        self.in_mission = False

        self.update_mission_state(
            'PAUSED',
            'Mission paused for target tracking'
        )

        self.get_logger().warning(
            'Mission paused for tracking'
        )

    async def resume_mission(self) -> None:
        await self.drone.mission.start_mission()

        self.in_mission = True

        self.update_mission_state(
            'ACTIVE',
            'Mission resumed'
        )

        self.get_logger().info(
            'Mission resumed'
        )

    def update_mission_state(self, state: str, message: str) -> None:
        self.mission_state = state
        self.status_message = message

    def publish_dashboard_data(self) -> None:
        stamp = self.get_clock().now().to_msg()

        mission_msg = MissionStatus()
        mission_msg.stamp = stamp
        mission_msg.frame_id = 'mission'
        mission_msg.current_mission_state = self.mission_state
        mission_msg.target_id = ''
        mission_msg.mission_progress = max(
            0,
            min(100, self.mission_progress)
        )
        mission_msg.mission_active = self.in_mission
        mission_msg.emergency_landing = False
        mission_msg.status_message = self.status_message

        self.mission_publisher.publish(
            mission_msg
        )

        telemetry_msg = PX4Telemetry()
        telemetry_msg.stamp = stamp
        telemetry_msg.frame_id = 'px4'

        telemetry_msg.position.x = self.longitude_deg
        telemetry_msg.position.y = self.latitude_deg
        telemetry_msg.position.z = self.altitude_m

        telemetry_msg.altitude_m = self.altitude_m
        telemetry_msg.ground_speed_mps = self.ground_speed_mps
        telemetry_msg.battery_voltage = self.battery_voltage
        telemetry_msg.battery_percentage = self.battery_percentage
        telemetry_msg.flight_mode = self.flight_mode
        telemetry_msg.connected = self.connected
        telemetry_msg.armed = self.armed
        telemetry_msg.in_mission = self.in_mission
        telemetry_msg.status = self.status_message

        self.telemetry_publisher.publish(
            telemetry_msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = MissionPlannerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node.loop is not None:
            node.loop.call_soon_threadsafe(
                node.loop.stop
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()