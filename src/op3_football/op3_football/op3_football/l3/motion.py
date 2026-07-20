"""L3 motion prep — go() + module wiring + angle helpers."""

from __future__ import annotations

import math
import time
from typing import Optional

from op3_football.l2.robot import Robot
from op3_football.l3.coefs import WalkCoefs, presets
from op3_football.l3.joint_angles import JointAngles
from op3_football.l3.sense import Sense
from op3_football.util.units import clamp_tick, degree_to_tick


class Motion:
    """High-ish API for L4: walk modes, angles, sensing. No strategy here."""

    def __init__(self, robot: Optional[Robot] = None) -> None:
        self.robot = robot or Robot().start()
        self.joint = JointAngles(self.robot.joint)
        self.sense = Sense(self.robot)
        self._presets = presets()
        # Posture recovery settings for timed plain walking modes.
        self._final_back_deg = 3.0
        self._final_settle_s = 0.15
        self._ramp_start_before_end_s = 2.0
        self._ramp_steps = 4
        self._ramp_step_deg = 2.0
        self._ramp_modes = {"forward", "forward_fast", "backward", "side_left", "side_right", "spot"}

    def go(self, mode: str = 'forward', duration: Optional[float] = None) -> None:
        """Start prepared walking. Coefficients chosen here, not in L4.

        mode: key from presets() or demo turn modes:
              forward, turn_left, turn_right, demo_adaptive_turn, ...
        duration: if set, walk then stop; else leave walking on.
        """
        if mode == "demo_adaptive_turn":
            self._demo_adaptive_turn(duration if duration is not None else 3.0)
            return
        if mode == "demo_ball_follower_turn":
            self._demo_ball_follower_turn(duration if duration is not None else 3.0)
            return
        if mode == "turn_left":
            self._demo_turn_left(duration if duration is not None else 3.0)
            return
        if mode == "turn_right":
            self._demo_turn_right(duration if duration is not None else 3.0)
            return
        if mode == "pivot_left":
            self._demo_turn_left(duration if duration is not None else 3.0)
            return
        if mode == "pivot_right":
            self._demo_turn_right(duration if duration is not None else 3.0)
            return

        if mode not in self._presets:
            raise KeyError(
                "Unknown go mode "
                f"{mode!r}. Available presets: {list(self._presets)} "
                "plus demo_adaptive_turn, demo_ball_follower_turn."
            )

        # Connect modules at L3
        self.robot.set_module('walking_module')
        time.sleep(0.05)
        coefs = self._presets[mode]
        self.robot.walk.start(coefs)

        if duration is not None:
            if mode in self._ramp_modes:
                self._run_pre_stop_lean_back_ramp(duration)
            else:
                time.sleep(duration)
            self.stop()
            if mode in self._ramp_modes:
                self._apply_ankle_back_delta(self._final_back_deg)
                time.sleep(self._final_settle_s)

    def go_coefs(self, coefs: WalkCoefs, duration: Optional[float] = None) -> None:
        self.robot.set_module('walking_module')
        time.sleep(0.05)
        self.robot.walk.start(coefs)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def stop(self) -> None:
        self.robot.walk.stop()

    def stand(self) -> None:
        self.robot.stand()

    def sit(self) -> None:
        self.robot.sit()

    def kick(self, side: str = 'right') -> None:
        if side == 'left':
            self.robot.kick.left()
        else:
            self.robot.kick.right()
        # return control path to walking-ready posture
        self.robot.set_module('walking_module')

    def estop(self) -> None:
        self.robot.estop()

    def get_preset(self, name: str) -> WalkCoefs:
        return self._presets[name]

    def _apply_ankle_back_delta(self, delta_deg: float) -> None:
        """Apply relative backward ankle pitch correction in degrees."""
        # NOTE: In this project map, 15/16 are ankle pitch. 17/18 are ankle roll.
        right_ank_pitch = 15
        left_ank_pitch = 16
        delta = degree_to_tick(delta_deg) - degree_to_tick(0.0)

        try:
            r_now = self.joint.read(right_ank_pitch)
            l_now = self.joint.read(left_ank_pitch)
            self.robot.joint.write_many(
                {
                    right_ank_pitch: clamp_tick(r_now + delta),
                    left_ank_pitch: clamp_tick(l_now - delta),
                }
            )
        except Exception:
            # Keep go() robust if a transient read/write error happens.
            pass

    def _run_pre_stop_lean_back_ramp(self, duration: float) -> None:
        """Run configurable lean-back ramp before timed stop."""
        if duration <= 0.0:
            return

        ramp_window = min(duration, self._ramp_start_before_end_s)
        pre_window = max(0.0, duration - ramp_window)
        if pre_window > 0.0:
            time.sleep(pre_window)

        steps = max(1, int(self._ramp_steps))
        interval = ramp_window / float(steps)
        for _ in range(steps):
            self._apply_ankle_back_delta(self._ramp_step_deg)
            if interval > 0.0:
                time.sleep(interval)

    def _demo_adaptive_turn(self, duration: float) -> None:
        """In-place adaptive turning from original demo logic (no forward step)."""
        self._run_demo_turn(duration=duration, forward_x=0.0)

    def _demo_ball_follower_turn(self, duration: float) -> None:
        """Demo-style turn with slight backward support like ball_follower start."""
        self._run_demo_turn(duration=duration, forward_x=-0.003)

    def _demo_turn_left(self, duration: float) -> None:
        """Left turn using the same logic as demo_ball_follower_turn."""
        self._run_demo_turn(duration=duration, forward_x=-0.003, forced_target_angle=-0.3)

    def _demo_turn_right(self, duration: float) -> None:
        """Right turn using the same logic as demo_ball_follower_turn."""
        self._run_demo_turn(duration=duration, forward_x=-0.003, forced_target_angle=0.3)

    def _run_demo_turn(self, duration: float, forward_x: float, forced_target_angle: Optional[float] = None) -> None:
        # Demo constants from op3_demo ball_follower.
        min_turn = math.radians(5.0)
        max_turn = math.radians(15.0)
        unit_turn = math.radians(0.5)
        update_hz = 20.0
        dt = 1.0 / update_hz

        current_r_angle = 0.0
        coefs = self.get_preset("spot")
        coefs.x_move_amplitude = forward_x
        coefs.y_move_amplitude = 0.0
        coefs.angle_move_amplitude = 0.0

        self.robot.set_module("walking_module")
        time.sleep(0.05)
        self.robot.walk.start(coefs)

        start_t = time.time()
        while time.time() - start_t < duration:
            # target_angle is equivalent to current_pan_ in demo follower.
            if forced_target_angle is not None:
                target_angle = forced_target_angle
            else:
                try:
                    target_angle = self.joint.read_rad(19)
                except Exception:
                    target_angle = 0.0

            # If the head is centered, keep a small command so turn can be tested.
            if abs(target_angle) < min_turn:
                target_angle = min_turn

            rl_angle = 0.0
            if abs(target_angle) > min_turn:
                rl_goal = min(abs(target_angle) * 0.2, max_turn)
                rl_goal = max(rl_goal, min_turn)
                rl_angle = min(abs(current_r_angle) + unit_turn, rl_goal)
                if target_angle < 0.0:
                    rl_angle *= -1.0

            coefs.angle_move_amplitude = rl_angle
            self.robot.walk.update(coefs)
            current_r_angle = rl_angle
            time.sleep(dt)

        self.stop()
