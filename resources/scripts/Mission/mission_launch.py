import argparse
import asyncio
import json
import math
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


DEFAULT_WAYPOINT_FILE = Path("px4_mavsdk_waypoints.json")
DEFAULT_CONNECTION = "udpin://0.0.0.0:14540"
TAKEOFF_ALT_M = 1.0
LOOP_RATE_HZ = 10.0
DESIRED_SPEED_MPS = 4.0
FINAL_HOLD_SECONDS = 5.0


def wrap_degrees(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_waypoints(path):
    with path.open("r", encoding="utf-8") as file:
        mission = json.load(file)

    waypoints = []
    for item in mission["waypoints"]:
        ned = item["px4_ned"]
        waypoints.append(
            {
                "north_m": float(ned["north_m"]),
                "east_m": float(ned["east_m"]),
                "down_m": float(ned["down_m"]),
                "yaw_deg": float(item.get("yaw_deg", 0.0)),
                "speed_to_next_mps": item.get("speed_to_next_mps"),
                "source_index": item.get("source_index"),
            }
        )

    if not waypoints:
        raise ValueError("Mission file has no waypoints.")

    return waypoints


def distance_3d(start_wp, end_wp):
    dn = end_wp["north_m"] - start_wp["north_m"]
    de = end_wp["east_m"] - start_wp["east_m"]
    dd = end_wp["down_m"] - start_wp["down_m"]
    return math.sqrt(dn**2 + de**2 + dd**2)


def speed_for_leg(start_wp, default_speed_mps):
    speed = start_wp.get("speed_to_next_mps")
    if speed is None:
        return default_speed_mps

    speed = float(speed)
    if speed <= 0.0:
        raise ValueError("speed_to_next_mps must be greater than 0.")

    return speed


def interpolate_waypoint(start_wp, end_wp, t):
    return {
        "north_m": start_wp["north_m"] + ((end_wp["north_m"] - start_wp["north_m"]) * t),
        "east_m": start_wp["east_m"] + ((end_wp["east_m"] - start_wp["east_m"]) * t),
        "down_m": start_wp["down_m"] + ((end_wp["down_m"] - start_wp["down_m"]) * t),
        "yaw_deg": start_wp["yaw_deg"] + ((end_wp["yaw_deg"] - start_wp["yaw_deg"]) * t),
    }


def path_yaw_deg(start_wp, end_wp):
    north_delta = end_wp["north_m"] - start_wp["north_m"]
    east_delta = end_wp["east_m"] - start_wp["east_m"]
    if abs(north_delta) < 1e-6 and abs(east_delta) < 1e-6:
        return start_wp["yaw_deg"]
    return wrap_degrees(math.degrees(math.atan2(east_delta, north_delta)))


async def first_telemetry_value(stream):
    async for value in stream:
        return value
    raise RuntimeError("Telemetry stream ended before returning a value.")


async def wait_for_connection(drone):
    print("Waiting for PX4 connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 discovered.")
            return


async def wait_until_ready(drone):
    print("Waiting for global/home position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Estimator ready.")
            return


async def send_position(drone, waypoint, hover_absolute_alt_m, yaw_deg):
    # Converted waypoints store down_m as an offset from the first Blender point.
    # MAVSDK PositionNedYaw needs down relative to PX4 local origin, so offset it
    # by the real altitude captured after takeoff.
    absolute_down_m = -hover_absolute_alt_m + waypoint["down_m"]
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            waypoint["north_m"],
            waypoint["east_m"],
            absolute_down_m,
            wrap_degrees(yaw_deg),
        )
    )


async def fly_mission(args):
    mission_path = args.waypoints.resolve()
    waypoints = load_waypoints(mission_path)

    drone = System()
    await drone.connect(system_address=args.connection)

    await wait_for_connection(drone)
    await wait_until_ready(drone)

    print("-- Arming x500")
    await drone.action.arm()
    await asyncio.sleep(2)

    print(f"-- Taking off to {args.takeoff_alt}m")
    await drone.action.set_takeoff_altitude(args.takeoff_alt)
    await drone.action.takeoff()
    await asyncio.sleep(args.takeoff_wait)
    await asyncio.sleep(20)

    position = await first_telemetry_value(drone.telemetry.position())
    hover_absolute_alt_m = position.relative_altitude_m

    heading = await first_telemetry_value(drone.telemetry.heading())
    captured_yaw_deg = heading.heading_deg

    print(
        f"Hover reference captured: altitude={hover_absolute_alt_m:.2f}m, "
        f"yaw={captured_yaw_deg:.2f} deg"
    )

    first_wp = waypoints[0]
    initial_yaw = select_yaw(args.yaw_mode, first_wp, first_wp, captured_yaw_deg)

    print("-- Preparing offboard stream")
    await send_position(drone, first_wp, hover_absolute_alt_m, initial_yaw)

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    dt = 1.0 / args.loop_rate

    try:
        print(f"-> Flying {len(waypoints)} converted Blender/PX4 waypoints")
        for index in range(len(waypoints) - 1):
            start_wp = waypoints[index]
            end_wp = waypoints[index + 1]
            segment_distance = distance_3d(start_wp, end_wp)
            segment_speed = speed_for_leg(start_wp, args.speed)
            segment_duration = segment_distance / segment_speed
            steps = max(1, math.ceil(segment_duration * args.loop_rate))
            yaw_deg = select_yaw(args.yaw_mode, start_wp, end_wp, captured_yaw_deg)

            print(
                f"Leg {index} -> {index + 1} | "
                f"distance={segment_distance:.2f}m | "
                f"speed={segment_speed:.2f}m/s | "
                f"yaw={yaw_deg:.2f} deg"
            )

            for step in range(1, steps + 1):
                waypoint = interpolate_waypoint(start_wp, end_wp, step / steps)
                await send_position(drone, waypoint, hover_absolute_alt_m, yaw_deg)
                await asyncio.sleep(dt)

        final_wp = waypoints[-1]
        final_yaw = select_yaw(args.yaw_mode, final_wp, final_wp, captured_yaw_deg)
        hold_steps = max(1, math.ceil(args.final_hold * args.loop_rate))

        print("-> Final waypoint reached. Holding position.")
        for _ in range(hold_steps):
            await send_position(drone, final_wp, hover_absolute_alt_m, final_yaw)
            await asyncio.sleep(dt)

    finally:
        print("-- Stopping offboard mode")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed: {error._result.result}")

        print("-- Landing")
        await drone.action.land()


def select_yaw(yaw_mode, start_wp, end_wp, captured_yaw_deg):
    if yaw_mode == "current":
        return captured_yaw_deg
    if yaw_mode == "path":
        return path_yaw_deg(start_wp, end_wp)
    return start_wp["yaw_deg"]


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Fly converted Blender mesh/spline waypoints in PX4 Gazebo x500."
    )
    parser.add_argument(
        "waypoints",
        nargs="?",
        type=Path,
        default=DEFAULT_WAYPOINT_FILE,
        help="Converted JSON from convert_blender_vertices_to_px4.py",
    )
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--takeoff-alt", type=float, default=TAKEOFF_ALT_M)
    parser.add_argument("--takeoff-wait", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=DESIRED_SPEED_MPS)
    parser.add_argument("--loop-rate", type=float, default=LOOP_RATE_HZ)
    parser.add_argument("--final-hold", type=float, default=FINAL_HOLD_SECONDS)
    parser.add_argument(
        "--yaw-mode",
        choices=["file", "current", "path"],
        default="current",
        help=(
            "file = yaw from converted JSON, current = keep takeoff heading, "
            "path = face each path segment"
        ),
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    asyncio.run(fly_mission(args))


if __name__ == "__main__":
    main()
