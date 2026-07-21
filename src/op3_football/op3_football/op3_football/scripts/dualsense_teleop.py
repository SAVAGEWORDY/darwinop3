"""DualSense teleop for OP3 using Linux joystick device (/dev/input/js0).

Controls:
- Left stick +Y (up): walk forward
- Left stick -Y (down): walk backward
- Left stick +X (right): turn right
- Left stick -X (left): turn left
- Cross: kick (right leg by default)
- L1: get-up from front (action page 81)
- R1: get-up from back (action page 82)
- After get-up: stand page 50

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

from std_msgs.msg import Int32

from op3_football.l3.motion import Motion


@dataclass
class ControllerMap:
    # Defaults for common Linux DualSense mapping.
    left_x_axis: int = int(os.getenv("OP3_LX_AXIS", "0"))
    left_y_axis: int = int(os.getenv("OP3_LY_AXIS", "1"))
    cross_button: int = int(os.getenv("OP3_CROSS_BUTTON", "0"))
    l1_button: int = int(os.getenv("OP3_L1_BUTTON", "4"))
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
    kick_cooldown = float(os.getenv("OP3_TELEOP_KICK_COOLDOWN", "1.0"))
    getup_cooldown = float(os.getenv("OP3_TELEOP_GETUP_COOLDOWN", "1.5"))
    getup_front_page = int(os.getenv("OP3_GETUP_FRONT_PAGE", "81"))
    getup_back_page = int(os.getenv("OP3_GETUP_BACK_PAGE", "82"))
    stand_page = int(os.getenv("OP3_STAND_PAGE", "50"))
    getup_wait = float(os.getenv("OP3_TELEOP_GETUP_WAIT", "4.0"))
    stand_wait = float(os.getenv("OP3_TELEOP_STAND_WAIT", "1.2"))

    print("Teleop start: standing robot...")
    motion.stand()
    time.sleep(2.0)

    print(
        "Teleop ready. Controls: left stick up=forward, down=backward, "
        "left/right=turn left/right, cross=kick, "
        "L1=getup front(81), R1=getup back(82), then stand(50)."
    )

    last_cross = 0
    last_l1 = 0
    last_r1 = 0
    last_kick_time = 0.0
    last_getup_time = 0.0
    moving_mode = "idle"
    action_pub = motion.robot.bridge._node.create_publisher(Int32, "/robotis/action/page_num", 10)

    try:
        while True:
            js.poll()
            if motion.check_and_recover_fall():
                moving_mode = "idle"
                time.sleep(loop_sleep)
                continue

            cross = js.buttons.get(cmap.cross_button, 0)
            l1 = js.buttons.get(cmap.l1_button, 0)
            r1 = js.buttons.get(cmap.r1_button, 0)
            lx_raw = js.axes.get(cmap.left_x_axis, 0)
            ly_raw = js.axes.get(cmap.left_y_axis, 0)

            # Axis conventions:
            # - left stick up is usually negative raw values.
            # - left stick right is usually positive raw values.
            lx = normalized_axis_signed(lx_raw)
            ly = normalized_axis_signed(ly_raw)

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

            # L1/R1 rising edge -> get-up action pages.
            if l1 == 1 and last_l1 == 0 and (now - last_getup_time) > getup_cooldown:
                motion.stop()
                motion.robot.set_module("action_module")
                time.sleep(0.12)
                msg = Int32()
                msg.data = getup_front_page
                action_pub.publish(msg)
                last_getup_time = now
                moving_mode = "idle"
                last_l1 = l1
                last_r1 = r1
                time.sleep(getup_wait)
                msg.data = stand_page
                action_pub.publish(msg)
                time.sleep(stand_wait)
                continue
            if r1 == 1 and last_r1 == 0 and (now - last_getup_time) > getup_cooldown:
                motion.stop()
                motion.robot.set_module("action_module")
                time.sleep(0.12)
                msg = Int32()
                msg.data = getup_back_page
                action_pub.publish(msg)
                last_getup_time = now
                moving_mode = "idle"
                last_l1 = l1
                last_r1 = r1
                time.sleep(getup_wait)
                msg.data = stand_page
                action_pub.publish(msg)
                time.sleep(stand_wait)
                continue
            last_l1 = l1
            last_r1 = r1

            # Priority: turn (left-stick X) > forward/backward > idle
            desired = "idle"
            if lx > stick_deadzone:
                desired = "turn_right"
            elif lx < -stick_deadzone:
                desired = "turn_left"
            elif ly < -stick_deadzone:
                desired = "forward"
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

