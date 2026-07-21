"""DualSense teleop for OP3 using Linux joystick device (/dev/input/js0).

Controls:
- Left stick +Y (up): walk forward
- Left stick -Y (down): walk backward
- R1 + Left stick +Y (up): walk forward_fast
- L2: turn left
- R2: turn right
- Cross: kick (right leg by default)

Safety choices:
- Movement is sent as short bursts (default 0.25s) to reduce fall risk.
- Turn has higher priority than forward.
- Auto-stop when controls are released.
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from typing import Dict

from op3_football.l3.motion import Motion


@dataclass
class ControllerMap:
    # Defaults for common Linux DualSense mapping.
    left_y_axis: int = int(os.getenv("OP3_LY_AXIS", "1"))
    l2_axis: int = int(os.getenv("OP3_L2_AXIS", "4"))
    r2_axis: int = int(os.getenv("OP3_R2_AXIS", "5"))
    cross_button: int = int(os.getenv("OP3_CROSS_BUTTON", "0"))
    r1_button: int = int(os.getenv("OP3_R1_BUTTON", "5"))


class JSDevice:
    EVENT_FMT = "IhBB"
    EVENT_SIZE = struct.calcsize(EVENT_FMT)

    def __init__(self, path: str = "/dev/input/js0") -> None:
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes: Dict[int, int] = {}
        self.buttons: Dict[int, int] = {}

    def close(self) -> None:
        os.close(self.fd)

    def poll(self) -> None:
        while True:
            try:
                data = os.read(self.fd, self.EVENT_SIZE)
            except BlockingIOError:
                break
            if len(data) != self.EVENT_SIZE:
                break

            _, value, etype, number = struct.unpack(self.EVENT_FMT, data)
            etype = etype & ~0x80  # clear init flag
            if etype == 0x01:  # button
                self.buttons[number] = value
            elif etype == 0x02:  # axis
                self.axes[number] = value


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalized_axis_signed(raw: int) -> float:
    return clamp(raw / 32767.0, -1.0, 1.0)


def main() -> None:
    cmap = ControllerMap()
    js_path = os.getenv("OP3_JS_DEV", "/dev/input/js0")

    if not os.path.exists(js_path):
        raise RuntimeError(
            f"Joystick device not found: {js_path}. "
            "Connect DualSense and check /dev/input/js*."
        )

    motion = Motion()
    js = JSDevice(js_path)

    # Teleop tuning for stability.
    step_duration = float(os.getenv("OP3_TELEOP_STEP", "0.25"))
    loop_sleep = float(os.getenv("OP3_TELEOP_LOOP", "0.02"))
    stick_deadzone = float(os.getenv("OP3_TELEOP_DEADZONE", "0.25"))
    trigger_threshold = float(os.getenv("OP3_TELEOP_TRIGGER", "0.20"))
    trigger_margin = float(os.getenv("OP3_TELEOP_TRIGGER_MARGIN", "0.05"))
    kick_cooldown = float(os.getenv("OP3_TELEOP_KICK_COOLDOWN", "1.0"))

    # Trigger neutral calibration (controllers vary by driver).
    calib_time = 1.0
    l2_neutral = 0.0
    r2_neutral = 0.0
    l2_count = 0
    r2_count = 0

    print("Teleop start: standing robot...")
    motion.stand()
    time.sleep(2.0)
    print(f"Calibrating triggers for {calib_time:.1f}s. Do not press L2/R2.")

    t0 = time.time()
    while time.time() - t0 < calib_time:
        js.poll()
        if cmap.l2_axis in js.axes:
            l2_neutral += js.axes[cmap.l2_axis]
            l2_count += 1
        if cmap.r2_axis in js.axes:
            r2_neutral += js.axes[cmap.r2_axis]
            r2_count += 1
        time.sleep(0.01)

    if l2_count:
        l2_neutral /= l2_count
    if r2_count:
        r2_neutral /= r2_count

    print(
        "Teleop ready. Controls: left stick up=forward, "
        "down=backward, R1+up=forward_fast, L2/R2=turn left/right, cross=kick."
    )

    last_cross = 0
    last_kick_time = 0.0
    moving_mode = "idle"

    try:
        while True:
            js.poll()
            if motion.check_and_recover_fall():
                moving_mode = "idle"
                time.sleep(loop_sleep)
                continue

            cross = js.buttons.get(cmap.cross_button, 0)
            r1 = js.buttons.get(cmap.r1_button, 0)
            ly_raw = js.axes.get(cmap.left_y_axis, 0)
            l2_raw = js.axes.get(cmap.l2_axis, int(l2_neutral))
            r2_raw = js.axes.get(cmap.r2_axis, int(r2_neutral))

            # Axis conventions:
            # - left stick up is usually negative raw values.
            ly = normalized_axis_signed(ly_raw)

            # Trigger activation relative to calibrated neutral.
            l2_active = clamp((l2_raw - l2_neutral) / 32767.0, 0.0, 1.0)
            r2_active = clamp((r2_raw - r2_neutral) / 32767.0, 0.0, 1.0)

            # Cross rising edge -> kick.
            now = time.time()
            if cross == 1 and last_cross == 0 and (now - last_kick_time) > kick_cooldown:
                motion.stop()
                motion.kick("right")
                last_kick_time = now
                moving_mode = "idle"
                last_cross = cross
                continue
            last_cross = cross

            # Priority: turn > forward > idle
            desired = "idle"
            if (l2_active > trigger_threshold) or (r2_active > trigger_threshold):
                if l2_active > (r2_active + trigger_margin):
                    desired = "turn_left"
                elif r2_active > (l2_active + trigger_margin):
                    desired = "turn_right"
            elif ly < -stick_deadzone:
                desired = "forward_fast" if r1 else "forward"
            elif ly > stick_deadzone:
                desired = "backward"

            if desired == "idle":
                if moving_mode != "idle":
                    motion.stop()
                    moving_mode = "idle"
                time.sleep(loop_sleep)
                continue

            # Short burst for stability and responsiveness.
            motion.go(desired, duration=step_duration)
            moving_mode = desired
            time.sleep(loop_sleep)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            motion.stop()
            motion.estop()
        except Exception:
            pass
        motion.robot.close()
        js.close()


if __name__ == "__main__":
    main()

