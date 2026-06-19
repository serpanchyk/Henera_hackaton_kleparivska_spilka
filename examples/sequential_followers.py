#!/usr/bin/env python3
"""
Sequential follower-chain MVP demo.

This file uses mock observations to demonstrate controller behavior without
running Gazebo. The relay_state handoff between controllers below is only a
test-time stand-in for a future optical LED decoder.

Run:
  python3 examples/sequential_followers.py
"""

import importlib.util
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from drone_sdk import (
        FollowerController,
        FollowerControllerConfig,
        MissionState,
        VisualObservation,
        build_chain_config,
    )
except ModuleNotFoundError:
    module_path = Path(__file__).resolve().parents[1] / 'drone_sdk' / 'follower_controller.py'
    spec = importlib.util.spec_from_file_location('follower_controller_demo', module_path)
    follower_controller = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = follower_controller
    spec.loader.exec_module(follower_controller)
    FollowerController = follower_controller.FollowerController
    FollowerControllerConfig = follower_controller.FollowerControllerConfig
    MissionState = follower_controller.MissionState
    VisualObservation = follower_controller.VisualObservation
    build_chain_config = follower_controller.build_chain_config


def observation(
    mission_state,
    current_time,
    visible=True,
    horizontal_angle_deg=3.0,
    vertical_angle_deg=0.5,
    target_size=70.0,
):
    return VisualObservation(
        target_visible=visible,
        horizontal_angle_deg=horizontal_angle_deg,
        vertical_angle_deg=vertical_angle_deg,
        target_size=target_size,
        mission_state=mission_state,
        timestamp=current_time,
    )


def build_controllers(follower_count):
    cfg = FollowerControllerConfig(
        reacquire_frames=2,
        lost_timeout=0.3,
        observation_timeout=0.2,
        smoothing_alpha=0.0,
    )
    return [
        FollowerController(link.follower_id, link.target_id, cfg)
        for link in build_chain_config(follower_count)
    ]


def step_chain(controllers, leader_observation, current_time):
    commands = []
    upstream_observation = leader_observation
    for controller in controllers:
        command = controller.update(upstream_observation, current_time=current_time)
        commands.append(command)
        upstream_observation = observation(
            command.relay_state,
            current_time,
            visible=True,
            horizontal_angle_deg=0.0,
            vertical_angle_deg=0.0,
            target_size=80.0,
        )
    return commands


def print_step(label, commands):
    states = ' | '.join(
        f'f{i + 1}:{cmd.state}/{cmd.relay_state}'
        for i, cmd in enumerate(commands)
    )
    speeds = ' | '.join(
        f'f{i + 1}:forward={cmd.forward_m_s:.2f},yaw={cmd.yaw_rate_deg_s:.2f}'
        for i, cmd in enumerate(commands)
    )
    print(f'{label}: {states}')
    print(f'  {speeds}')


def run_mock_scenario(follower_count):
    print(f'Chain mapping for {follower_count} followers:')
    for link in build_chain_config(follower_count):
        print(f'  {link.follower_id} follows {link.target_id} using drone_id={link.drone_id}')

    controllers = build_controllers(follower_count)
    t = 0.0

    for index in range(follower_count + 1):
        commands = step_chain(controllers, observation(MissionState.FOLLOW, t), t)
        print_step(f'normal follow frame {index + 1}', commands)
        t += 0.1

    commands = step_chain(
        controllers,
        observation(MissionState.FOLLOW, t, visible=False),
        t,
    )
    print_step('leader temporarily lost by follower_1', commands)
    t += 0.4

    commands = step_chain(
        controllers,
        observation(MissionState.FOLLOW, t, visible=False),
        t,
    )
    print_step('cascade hold after lost_timeout', commands)
    t += 0.1

    for index in range(follower_count + 1):
        commands = step_chain(controllers, observation(MissionState.FOLLOW, t), t)
        print_step(f'sequential reacquire frame {index + 1}', commands)
        t += 0.1


def main():
    run_mock_scenario(2)
    print()
    run_mock_scenario(3)


if __name__ == '__main__':
    main()
