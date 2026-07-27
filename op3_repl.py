#!/usr/bin/env python3
"""Interactive OP3 shell with ready-to-use Motion helpers."""

from __future__ import annotations

import os

from op3_football.l3.motion import Motion

m: Motion | None = None
_startup_head_tilt_deg = float(os.getenv("OP3_STARTUP_HEAD_TILT_DEG", "0.0"))


def get_motion() -> Motion:
    global m
    if m is None:
        m = Motion()
    return m


def stand() -> Motion:
    motion = get_motion()
    motion.stand()
    return motion


def sit() -> Motion:
    motion = get_motion()
    motion.sit()
    return motion


def stop() -> Motion:
    motion = get_motion()
    motion.stop()
    return motion


def estop() -> Motion:
    motion = get_motion()
    motion.estop()
    return motion


def go(mode: str = "forward", duration: float | None = None) -> Motion:
    motion = get_motion()
    motion.go(mode, duration=duration)
    return motion


def close() -> None:
    global m
    if m is not None:
        m.robot.close()
        m = None


def apply_startup_offsets() -> Motion:
    """Extra startup-only offsets for run_op3_shell.sh session."""
    motion = get_motion()
    try:
        # Keep camera vertical to reduce impact risk on forward falls.
        motion.joint.write_deg(20, _startup_head_tilt_deg)
    except Exception:
        pass
    return motion


get_motion()
apply_startup_offsets()

print("OP3 interactive shell ready.")
print("Preloaded: m, Motion, get_motion, stand, sit, go, stop, estop, close, apply_startup_offsets")
print("Examples:")
print("  m = stand()")
print("  go('forward', duration=5.0)")
print("  m.go('turn_left', duration=2.0)")
