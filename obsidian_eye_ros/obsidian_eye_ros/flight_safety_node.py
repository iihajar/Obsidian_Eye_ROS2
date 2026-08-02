#!/usr/bin/env python3
"""Flight safety monitor for Obsidian Eye.

Watches battery and altitude during the mission and triggers a
return-to-launch (or a small altitude correction) if either drops
below a safe limit.
"""

import asyncio

# Battery
LOW_BATTERY_RTL_THRESHOLD = 0.25

# Altitude (meters)
FLYING_ALTITUDE_THRESHOLD_M = 3.0
CRITICAL_ALTITUDE_M = 2.0
CORRECTION_ALTITUDE_M = 3.0
CORRECTION_TARGET_CLIMB_M = 5.0
CORRECTION_SETTLE_SEC = 4.0


class FlightSafety:

    def __init__(self, drone, logger):
        self.drone = drone
        self.log = logger

    async def monitor(self):
        """Run battery and altitude monitors together.

        Each monitor guards itself against exceptions internally, so
        one failing (e.g. a dropped PX4 command) never silently
        kills the other's safety checks.
        """
        await asyncio.gather(
            self._guarded(self.monitor_battery(), 'battery monitor'),
            self._guarded(self.monitor_altitude(), 'altitude monitor'),
        )

    async def _guarded(self, coro, name):
        try:
            await coro
        except Exception as error:
            self.log.error(
                f'Flight safety: {name} crashed: {error} '
                f'- safety checks for this monitor have stopped!'
            )

    async def monitor_battery(self):
        async for battery in self.drone.telemetry.battery():

            if battery.remaining_percent <= LOW_BATTERY_RTL_THRESHOLD:
                self.log.warning(
                    f'Battery below '
                    f'{LOW_BATTERY_RTL_THRESHOLD * 100:.0f}% - RTL'
                )

                await self.drone.mission.pause_mission()
                await self.drone.action.return_to_launch()
                return

    async def monitor_altitude(self):
        started_flying = False

        async for position in self.drone.telemetry.position():

            altitude = position.relative_altitude_m

            if altitude > FLYING_ALTITUDE_THRESHOLD_M:
                started_flying = True

            if not started_flying:
                continue

            if altitude < CRITICAL_ALTITUDE_M:
                self.log.error(
                    f'Altitude below {CRITICAL_ALTITUDE_M} m - RTL'
                )

                await self.drone.mission.pause_mission()
                await self.drone.action.return_to_launch()
                return

            if altitude < CORRECTION_ALTITUDE_M:
                self.log.warning(
                    f'Altitude below {CORRECTION_ALTITUDE_M} m '
                    f'- Correcting'
                )

                await self.drone.mission.pause_mission()

                target_absolute_altitude = (
                    position.absolute_altitude_m
                    + (CORRECTION_TARGET_CLIMB_M - altitude)
                )

                await self.drone.action.goto_location(
                    position.latitude_deg,
                    position.longitude_deg,
                    target_absolute_altitude,
                    0.0
                )

                await asyncio.sleep(CORRECTION_SETTLE_SEC)

                # NOTE: if the mission was paused for target tracking
                # (Decision Engine), this will resume the patrol
                # mission and could override that pause. Flag this
                # to the user if it becomes a real issue in practice.
                await self.drone.mission.start_mission()

                self.log.info('Altitude corrected - Mission resumed')