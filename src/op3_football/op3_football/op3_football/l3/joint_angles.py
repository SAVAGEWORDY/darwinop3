"""L3 joint helpers in degrees / radians on top of L2 tick write."""

from __future__ import annotations

from op3_football.l2.joint import JointAPI
from op3_football.util.units import degree_to_tick, radian_to_tick, tick_to_degree, tick_to_radian


class JointAngles:
    def __init__(self, joint: JointAPI) -> None:
        self._joint = joint

    def write(self, joint_id: int, value: int) -> None:
        """Still available: raw ticks."""
        self._joint.write(joint_id, value)

    def write_deg(self, joint_id: int, degrees: float) -> None:
        self._joint.write(joint_id, degree_to_tick(degrees))

    def write_rad(self, joint_id: int, radians: float) -> None:
        self._joint.write(joint_id, radian_to_tick(radians))

    def read(self, joint_id: int) -> int:
        return self._joint.read(joint_id)

    def read_deg(self, joint_id: int) -> float:
        return tick_to_degree(self._joint.read(joint_id))

    def read_rad(self, joint_id: int) -> float:
        return tick_to_radian(self._joint.read(joint_id))
