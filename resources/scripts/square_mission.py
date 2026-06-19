import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

TAKEOFF_ALT_M = 10.0
STRAIGHT_DISTANCE_M = 10.0
STRAIGHT_FLIGHT_S = 8.0
FINAL_HOVER_S = 5.0

async def run():
    # Connect to the local SITL instance using the updated udpin:// protocol
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected successfully!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print(f"-- Taking off to {TAKEOFF_ALT_M} meters")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    
    # Wait for the drone to finish taking off and stabilize
    await asyncio.sleep(10)

    print("-- Setting initial setpoint to current position")
    # In NED coordinates: Z is negative for altitude.
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT_M, 0.0))

    print("-- Starting Offboard Mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print(f"-- Moving straight forward {STRAIGHT_DISTANCE_M} meters")
    await drone.offboard.set_position_ned(
        PositionNedYaw(STRAIGHT_DISTANCE_M, 0.0, -TAKEOFF_ALT_M, 0.0)
    )
    await asyncio.sleep(STRAIGHT_FLIGHT_S)

    print("-- Holding final straight-line position")
    await drone.offboard.set_position_ned(
        PositionNedYaw(STRAIGHT_DISTANCE_M, 0.0, -TAKEOFF_ALT_M, 0.0)
    )
    await asyncio.sleep(FINAL_HOVER_S)

    print("-- Stopping Offboard Mode to allow landing action")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
