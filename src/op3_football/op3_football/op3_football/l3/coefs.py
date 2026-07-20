"""Walking coefficient presets for L3.

Isolated here so L4 / strategy code stays clean and we can later
swap heuristics for learned params without touching skill code.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields
from typing import Dict

from op3_walking_module_msgs.msg import WalkingParam


@dataclass
class WalkCoefs:
    """Subset of WalkingParam that L3 usually tunes.

    Units match the ROS WalkingParam message after yaml load:
    offsets/amplitudes in meters, angles in radians, period_time in seconds.
    """

    init_x_offset: float = -0.02
    init_y_offset: float = 0.015
    init_z_offset: float = 0.035
    init_roll_offset: float = 0.0
    init_pitch_offset: float = 0.0
    init_yaw_offset: float = 0.0

    period_time: float = 0.65
    dsp_ratio: float = 0.2
    step_fb_ratio: float = 0.28

    x_move_amplitude: float = 0.0
    y_move_amplitude: float = 0.0
    z_move_amplitude: float = 0.06
    angle_move_amplitude: float = 0.0
    move_aim_on: bool = False

    balance_enable: bool = True
    balance_hip_roll_gain: float = 0.35
    balance_knee_gain: float = 0.30
    balance_ankle_roll_gain: float = 0.70
    balance_ankle_pitch_gain: float = 0.90
    y_swap_amplitude: float = 0.028
    z_swap_amplitude: float = 0.006
    arm_swing_gain: float = 0.20
    pelvis_offset: float = 0.0
    hip_pitch_offset: float = 5.0 * math.pi / 180.0

    p_gain: int = 0
    i_gain: int = 0
    d_gain: int = 0

    def to_msg(self) -> WalkingParam:
        msg = WalkingParam()
        for f in fields(self):
            setattr(msg, f.name, getattr(self, f.name))
        return msg

    def with_step(
        self,
        x: float | None = None,
        y: float | None = None,
        angle: float | None = None,
    ) -> 'WalkCoefs':
        c = WalkCoefs(**asdict(self))
        if x is not None:
            c.x_move_amplitude = x
        if y is not None:
            c.y_move_amplitude = y
        if angle is not None:
            c.angle_move_amplitude = angle
        return c


def _base() -> WalkCoefs:
    """Conservative indoor defaults inspired by stock param.yaml + ball_follower limits."""
    return WalkCoefs()


def presets() -> Dict[str, WalkCoefs]:
    """Named presets used by L3.go(mode=...)."""
    base = _base()
    walk_base = WalkCoefs(**asdict(base))
    # Reduce forward torso lean for plain walking only (not turns/pivots).
    walk_base.hip_pitch_offset = base.hip_pitch_offset - 5.0 * math.pi / 180.0
    # More stable in-place rotation:
    # - longer double support phase,
    # - smaller yaw amplitude,
    # - slightly lower CoM/step dynamics.
    pivot_left = base.with_step(x=0.0, y=0.0, angle=0.13)
    pivot_left.period_time = 0.72
    pivot_left.dsp_ratio = 0.28
    pivot_left.y_swap_amplitude = 0.022
    pivot_left.z_move_amplitude = 0.050

    pivot_right = base.with_step(x=0.0, y=0.0, angle=-0.13)
    pivot_right.period_time = 0.72
    pivot_right.dsp_ratio = 0.28
    pivot_right.y_swap_amplitude = 0.022
    pivot_right.z_move_amplitude = 0.050

    return {
        # slow confident forward (safer than soccer demo max 40mm)
        'forward': walk_base.with_step(x=0.020, y=0.0, angle=0.0),
        'forward_fast': walk_base.with_step(x=0.030, y=0.0, angle=0.0),
        'backward': walk_base.with_step(x=-0.015, y=0.0, angle=0.0),
        'side_left': walk_base.with_step(x=0.0, y=0.020, angle=0.0),
        'side_right': walk_base.with_step(x=0.0, y=-0.020, angle=0.0),
        'turn_left': base.with_step(x=0.005, y=0.0, angle=0.15),
        'turn_right': base.with_step(x=0.005, y=0.0, angle=-0.15),
        'pivot_left': pivot_left,
        'pivot_right': pivot_right,
        'spot': walk_base.with_step(x=-0.003, y=0.0, angle=0.0),  # in-place like demo
    }
