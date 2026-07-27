"""L2 joint API — values are Dynamixel ticks 0..4095."""

from __future__ import annotations

from typing import Dict, Sequence

from op3_football.l1.bridge_client import BridgeClient


class JointAPI:
    def __init__(self, bridge: BridgeClient) -> None:
        self._bridge = bridge

    def write(self, joint_id: int, value: int) -> None:
        """id.write(id, value) style — ticks 0..4095."""
        self._bridge.set_module('direct_control_module')
        self._bridge.joint_write(joint_id, value)

    def read(self, joint_id: int) -> int:
        return self._bridge.joint_read(joint_id)

    def write_many(self, goals: Dict[int, int]) -> None:
        ids = list(goals.keys())
        values = list(goals.values())
        self._bridge.set_module('direct_control_module')
        self._bridge.joint_write_many(ids, values)

    def write_pairs(self, ids: Sequence[int], values: Sequence[int]) -> None:
        self._bridge.set_module('direct_control_module')
        self._bridge.joint_write_many(ids, values)
