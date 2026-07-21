"""L3 motion prep — go() + module wiring + angle helpers."""

from __future__ import annotations

from dataclasses import asdict
import math
from pathlib import Path
import time
from typing import Optional
from std_msgs.msg import Int32
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - ROS env should provide this
    get_package_share_directory = None

from op3_football.l2.robot import Robot
from op3_football.l3.coefs import WalkCoefs, presets
from op3_football.l3.joint_angles import JointAngles
from op3_football.l3.sense import Sense


class Motion:
    """High-ish API for L4: walk modes, angles, sensing. No strategy here."""

    def __init__(self, robot: Optional[Robot] = None) -> None:
        self.robot = robot or Robot().start()
        self.joint = JointAngles(self.robot.joint)
        self.sense = Sense(self.robot)
        self._presets = presets()
        # Posture recovery settings for timed plain walking modes.
        self._ramp_start_before_end_s = 3.0
        self._ramp_steps = 6
        self._ramp_step_deg = 1.5
        self._ramp_modes = {"forward", "forward_fast", "backward", "side_left", "side_right", "spot"}
        # Auto get-up settings (L3-level, from config; defaults match OP3 demo).
        self._auto_getup_enabled = True
        self._auto_getup_stand_after_getup = True
        self._fall_forward_limit_deg = 60.0
        self._fall_back_limit_deg = -60.0
        self._fall_alpha = 0.4
        self._filtered_pitch_deg = 0.0
        self._have_filtered_pitch = False
        self._getup_front_page = 122
        self._getup_back_page = 123
        self._getup_wait_s = 4.5
        self._stand_page = 9
        self._stand_page_wait_s = 1.2
        self._fall_watch_dt = 0.05
        self._load_auto_getup_config()
        self._action_page_pub = self.robot.bridge._node.create_publisher(Int32, "/robotis/action/page_num", 10)

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
        coefs = WalkCoefs(**asdict(self._presets[mode]))
        self.robot.walk.start(coefs)

        if duration is not None:
            if mode in self._ramp_modes:
                if self._run_pre_stop_lean_back_ramp(duration, coefs):
                    return
            else:
                if self._sleep_with_fall_watch(duration):
                    return
            self.stop()

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

    def check_and_recover_fall(self) -> bool:
        """Public one-shot fall check useful in teleop loops."""
        return self._recover_if_fallen()

    def _run_pre_stop_lean_back_ramp(self, duration: float, coefs: WalkCoefs) -> bool:
        """Run configurable lean-back ramp before timed stop.

        Important: keep walking module active and avoid direct-control writes
        while gait is running, otherwise manager can crash.
        """
        if duration <= 0.0:
            return False

        ramp_window = min(duration, self._ramp_start_before_end_s)
        pre_window = max(0.0, duration - ramp_window)
        if pre_window > 0.0 and self._sleep_with_fall_watch(pre_window):
            return True

        steps = max(1, int(self._ramp_steps))
        interval = ramp_window / float(steps)
        for _ in range(steps):
            if self._recover_if_fallen():
                return True
            coefs.hip_pitch_offset -= math.radians(self._ramp_step_deg)
            self.robot.walk.update(coefs)
            if interval > 0.0 and self._sleep_with_fall_watch(interval):
                return True
        return False

    def _sleep_with_fall_watch(self, duration: float) -> bool:
        """Sleep in short chunks and interrupt on fall recovery."""
        remaining = max(0.0, duration)
        while remaining > 0.0:
            if self._recover_if_fallen():
                return True
            dt = min(self._fall_watch_dt, remaining)
            time.sleep(dt)
            remaining -= dt
        return False

    def _recover_if_fallen(self) -> bool:
        if not self._auto_getup_enabled:
            return False
        fallen = self._fallen_state()
        if fallen is None:
            return False
        self.stop()
        self.robot.set_module("action_module")
        page = self._getup_front_page if fallen == "front" else self._getup_back_page
        self._play_action_page(page, self._getup_wait_s)
        if self._auto_getup_stand_after_getup:
            # Prefer action stand page over base ini_pose when offsets are rough.
            self._play_action_page(self._stand_page, self._stand_page_wait_s)
        self._have_filtered_pitch = False
        self._filtered_pitch_deg = 0.0
        return True

    def _fallen_state(self) -> Optional[str]:
        imu = self.robot.bridge.imu
        if imu is None:
            return None
        q = imu.orientation
        # Quaternion -> pitch (rotation around Y), radians to degrees.
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_deg = math.degrees(math.asin(sinp))
        if not self._have_filtered_pitch:
            self._have_filtered_pitch = True
            self._filtered_pitch_deg = pitch_deg
        else:
            a = self._fall_alpha
            self._filtered_pitch_deg = self._filtered_pitch_deg * (1.0 - a) + pitch_deg * a
        if self._filtered_pitch_deg > self._fall_forward_limit_deg:
            return "front"
        if self._filtered_pitch_deg < self._fall_back_limit_deg:
            return "back"
        return None

    def _load_auto_getup_config(self) -> None:
        cfg_path: Optional[Path] = None
        try:
            if get_package_share_directory is not None:
                cfg_path = Path(get_package_share_directory("op3_football")) / "config" / "l3_motion.yaml"
        except Exception:
            cfg_path = None
        if cfg_path is None:
            # Fallback for source-tree execution.
            cfg_path = Path(__file__).resolve().parents[2] / "config" / "l3_motion.yaml"

        try:
            data = yaml.safe_load(cfg_path.read_text()) or {}
        except Exception:
            return

        section = data.get("auto_getup", {})
        if not isinstance(section, dict):
            return

        self._auto_getup_enabled = bool(section.get("enabled", self._auto_getup_enabled))
        self._auto_getup_stand_after_getup = bool(
            section.get("stand_after_getup", self._auto_getup_stand_after_getup)
        )
        self._fall_forward_limit_deg = float(section.get("fall_forward_limit_deg", self._fall_forward_limit_deg))
        self._fall_back_limit_deg = float(section.get("fall_back_limit_deg", self._fall_back_limit_deg))
        self._fall_alpha = float(section.get("fall_alpha", self._fall_alpha))
        self._getup_front_page = int(section.get("getup_front_page", self._getup_front_page))
        self._getup_back_page = int(section.get("getup_back_page", self._getup_back_page))
        self._getup_wait_s = float(section.get("getup_wait_s", self._getup_wait_s))
        self._stand_page = int(section.get("stand_page", self._stand_page))
        self._stand_page_wait_s = float(section.get("stand_page_wait_s", self._stand_page_wait_s))
        self._fall_watch_dt = float(section.get("watch_dt_s", self._fall_watch_dt))

    def _play_action_page(self, page: int, wait_s: float) -> None:
        msg = Int32()
        msg.data = int(page)
        self._action_page_pub.publish(msg)
        if wait_s > 0.0:
            time.sleep(wait_s)

    def _demo_adaptive_turn(self, duration: float) -> None:
        """In-place adaptive turning from original demo logic (no forward step)."""
        self._run_demo_turn(duration=duration, forward_x=0.0)

    def _demo_ball_follower_turn(self, duration: float) -> None:
        """Demo-style turn with slight backward support like ball_follower start."""
        self._run_demo_turn(duration=duration, forward_x=-0.003)

    def _demo_turn_left(self, duration: float) -> None:
        """Left turn using the same logic as demo_ball_follower_turn."""
        self._run_demo_turn(duration=duration, forward_x=-0.003, forced_target_angle=0.3)

    def _demo_turn_right(self, duration: float) -> None:
        """Right turn using the same logic as demo_ball_follower_turn."""
        self._run_demo_turn(duration=duration, forward_x=-0.003, forced_target_angle=-0.3)

    def _run_demo_turn(self, duration: float, forward_x: float, forced_target_angle: Optional[float] = None) -> None:
        # Demo constants from op3_demo ball_follower.
        min_turn = math.radians(5.0)
        max_turn = math.radians(15.0)
        unit_turn = math.radians(0.5)
        update_hz = 20.0
        dt = 1.0 / update_hz

        current_r_angle = 0.0
        coefs = WalkCoefs(**asdict(self.get_preset("spot")))
        coefs.x_move_amplitude = forward_x
        coefs.y_move_amplitude = 0.0
        coefs.angle_move_amplitude = 0.0

        self.robot.set_module("walking_module")
        time.sleep(0.05)
        self.robot.walk.start(coefs)

        start_t = time.time()
        while time.time() - start_t < duration:
            if self._recover_if_fallen():
                return
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
            if self._sleep_with_fall_watch(dt):
                return

        self.stop()
