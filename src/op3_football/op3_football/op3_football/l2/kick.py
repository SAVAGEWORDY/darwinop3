"""Custom kick trajectories (not action pages). Tune keyframes on hardware."""

from __future__ import annotations

import time
from typing import Dict, List, Sequence, Tuple

from op3_football.l1.bridge_client import BridgeClient
from op3_football.l2.joint import JointAPI

# (duration_s, {joint_id: tick})
Keyframe = Tuple[float, Dict[int, int]]


def _lerp(a: int, b: int, t: float) -> int:
    return int(a + (b - a) * t)


class KickAPI:
    def __init__(self, bridge: BridgeClient, joint: JointAPI) -> None:
        self._bridge = bridge
        self._joint = joint

    def right(self) -> None:
        self._play(self._right_trajectory())

    def left(self) -> None:
        self._play(self._left_trajectory())

    def _play(self, frames: Sequence[Keyframe], dt: float = 0.02) -> None:
        self._bridge.walking_command('stop')
        self._bridge.set_module('direct_control_module')
        time.sleep(0.05)

        # start from current-ish stand-ish pose: first keyframe
        prev_pose = frames[0][1]
        self._joint.write_many(prev_pose)
        time.sleep(0.1)

        for duration, pose in frames[1:]:
            steps = max(1, int(duration / dt))
            for i in range(1, steps + 1):
                t = i / steps
                blended = {
                    jid: _lerp(prev_pose.get(jid, tick), tick, t)
                    for jid, tick in pose.items()
                }
                # keep joints from prev that are not in pose
                for jid, tick in prev_pose.items():
                    if jid not in blended:
                        blended[jid] = tick
                self._joint.write_many(blended)
                time.sleep(dt)
            prev_pose = {**prev_pose, **pose}

    def _right_trajectory(self) -> List[Keyframe]:
        # Placeholder keyframes — center 2048. Right leg swing forward.
        ready = {
            11: 2000, 12: 2100,
            13: 1900, 14: 2200,
            15: 2000, 16: 2100,
            7: 2048, 8: 2048,
        }
        swing = {
            11: 1700, 12: 2100,
            13: 1500, 14: 2200,
            15: 1800, 16: 2100,
        }
        follow = {
            11: 1950, 12: 2100,
            13: 1850, 14: 2200,
            15: 1950, 16: 2100,
        }
        return [
            (0.3, ready),
            (0.25, swing),
            (0.35, follow),
            (0.4, ready),
        ]

    def _left_trajectory(self) -> List[Keyframe]:
        ready = {
            11: 2100, 12: 2000,
            13: 2200, 14: 1900,
            15: 2100, 16: 2000,
            7: 2048, 8: 2048,
        }
        swing = {
            11: 2100, 12: 2396,
            13: 2200, 14: 2596,
            15: 2100, 16: 2296,
        }
        follow = {
            11: 2100, 12: 2148,
            13: 2200, 14: 2248,
            15: 2100, 16: 2148,
        }
        return [
            (0.3, ready),
            (0.25, swing),
            (0.35, follow),
            (0.4, ready),
        ]
