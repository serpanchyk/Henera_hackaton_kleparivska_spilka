import asyncio
import math
import os
import sys

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

# Repo root on the path so the shared swarm speed profile is the single source
# of truth (drone_sdk/swarm_speeds.py) rather than a duplicated local constant.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
from drone_sdk.config import CONFIG
from drone_sdk.swarm_speeds import LEADER_CRUISE_SPEED_M_S

# --- Configuration Constants ---
TAKEOFF_ALT_M = 1.0     # Take off exactly 1 meter above the platform
STRAIGHT_DISTANCE_M = 12.0
# Leader cruise speed comes from the shared profile so it stays matched to the
# follower chase speed. Allow generous travel time so the leg always completes.
STRAIGHT_FLIGHT_S = max(7, math.ceil(STRAIGHT_DISTANCE_M / LEADER_CRUISE_SPEED_M_S) + 4)
FINAL_HOVER_S = 5
DEFAULT_CONNECTION = f"udpin://0.0.0.0:{CONFIG.ports.mavsdk_udp_base}"

async def run():
    drone = System()
    await drone.connect(system_address=DEFAULT_CONNECTION)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    # Cap the leader's horizontal speed so it does not outrun the followers,
    # which are limited to LEADER_CRUISE_SPEED_M_S by FollowerControllerConfig.
    # MPC_XY_VEL_MAX bounds offboard position-setpoint speed; MPC_XY_CRUISE is
    # set to match for auto/mission consistency.
    print(f"-- Capping leader cruise speed to {LEADER_CRUISE_SPEED_M_S} m/s")
    await drone.param.set_param_float("MPC_XY_VEL_MAX", LEADER_CRUISE_SPEED_M_S)
    await drone.param.set_param_float("MPC_XY_CRUISE", LEADER_CRUISE_SPEED_M_S)

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(5) 

    print(f"-- Taking off to {TAKEOFF_ALT_M}m above platform")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    await asyncio.sleep(10) # Let it reach a completely stable hover

# =========================================================================
    # DYNAMIC TELEMETRY CAPTURE (FIXED)
    # Capture the exact heading and altitude the drone holds right now in hover.
    # =========================================================================
    print("-- Capturing baseline orientation...")
    
    # 1. Grab the current absolute altitude (Z-axis reference)
    async for position in drone.telemetry.position():
        hover_absolute_alt = position.relative_altitude_m
        break # Stop listening after getting the first stable reading
        
    # 2. Grab the current absolute heading (Yaw reference)
    async for heading in drone.telemetry.heading():
        hover_yaw_deg = heading.heading_deg  # <-- Fixed attribute name here!
        break # Stop listening after getting the first stable reading
        
    hover_yaw_rad = math.radians(hover_yaw_deg)
    print(f"Captured Base - Altitude: {hover_absolute_alt:.2f}m, Heading: {hover_yaw_deg:.2f}°")
    # Helper function using our freshly captured real-time bases
    def get_ned_position(forward, left, altitude_change):
        # Translate local forward/left steps based on the drone's true active heading
        north = (forward * math.cos(hover_yaw_rad)) + (left * math.sin(hover_yaw_rad))
        east = (forward * math.sin(hover_yaw_rad)) - (left * math.cos(hover_yaw_rad))
        
        # Target altitude is relative to our exact stable hover altitude
        target_alt = hover_absolute_alt + altitude_change
        down = -target_alt
        
        return PositionNedYaw(north, east, down, hover_yaw_deg)

    print("-- Preparing Offboard stream")
    # Feed an initial setpoint at its current exact location
    await drone.offboard.set_position_ned(get_ned_position(0.0, 0.0, 0.0))

    print("-- Starting Offboard Mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Straight leader mission only: no lateral or altitude pattern.
    print(f"-> Moving straight forward {STRAIGHT_DISTANCE_M} meters")
    await drone.offboard.set_position_ned(get_ned_position(STRAIGHT_DISTANCE_M, 0.0, 0.0))
    await asyncio.sleep(STRAIGHT_FLIGHT_S)

    print("-- Holding final straight-line position")
    await drone.offboard.set_position_ned(get_ned_position(STRAIGHT_DISTANCE_M, 0.0, 0.0))
    await asyncio.sleep(FINAL_HOVER_S)

    print("-- Stopping Offboard Mode to allow landing action")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    
if __name__ == "__main__":
    asyncio.run(run())
