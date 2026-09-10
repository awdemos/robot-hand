#!/usr/bin/env python3
"""robot-hand bridge — biomechanical tendon-pulley hand + HTTP/SSE server.

Physics model
-------------
Each digit is a chain of rigid phalanges driven by an antagonist pair of
tendons (flexor / extensor) reeled by forearm motors. Tendons are modelled
as spring-cables with nonlinear stiffness; each joint has a flexion moment
arm and an extension moment arm. Equations of motion are second-order:
net tendon torque minus damping, gravity, and joint-stop reaction.

Pre-designed hand postures plus per-joint/per-actuator explicit control are
provided: curl, spread, opposition, wrist pitch/yaw, and finger-by-finger joint
targets.

Endpoints
  GET  /            3D biomechanical hand viewer
  GET  /arm         full JSON state
  GET  /postures    list of available posture names
  POST /posture     {name: <posture>, duration_s}
  POST /goal        {kind: reach|grasp|release|point|wave|fist|ripple|pinch|ok|shaka|rock|spock, duration_s}
  POST /pose        {finger_targets: {...}, joint_targets: {...}, spread, thumb_opposition,
                     wrist: {pitch, yaw}, forearm: {roll}, curl, fist, duration_s}
  POST /actuator    {finger, joint, activation: 0..1, duration_s}
  POST /emotion     {stress, arousal, mood}
  GET  /events      SSE telemetry (~10 Hz)
  GET  /healthz
"""

import json
import logging
import math
import queue
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

log = logging.getLogger("hand-bridge")

FINGERS = ("thumb", "index", "middle", "ring", "pinky")

JOINTS = {
    "thumb": ("cmc", "mcp", "ip"),
    "index": ("mcp", "pip", "dip"),
    "middle": ("mcp", "pip", "dip"),
    "ring": ("mcp", "pip", "dip"),
    "pinky": ("mcp", "pip", "dip"),
}

MAX_ANGLE = {"mcp": 90.0, "pip": 100.0, "dip": 75.0, "cmc": 55.0, "ip": 80.0}

# Realistic phalanx/metacarpal lengths (cm)
SEG_LENGTH = {
    "thumb": [("cmc", 1.8), ("mcp", 2.8), ("ip", 2.4)],
    "index": [("mcp", 4.2), ("pip", 2.6), ("dip", 1.9)],
    "middle": [("mcp", 4.5), ("pip", 3.0), ("dip", 2.0)],
    "ring": [("mcp", 4.1), ("pip", 2.8), ("dip", 1.95)],
    "pinky": [("mcp", 3.4), ("pip", 2.2), ("dip", 1.7)],
}

# Tendon moment arms (cm)
MOMENT_ARM = {
    "cmc": (0.32, 0.28),
    "mcp": (0.36, 0.30),
    "pip": (0.28, 0.22),
    "dip": (0.20, 0.16),
    "ip": (0.24, 0.20),
}

DAMPING = 0.22
INERTIA = 0.018


def _clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, float(x)))


def _sigmoid(x, k=10.0, mid=0.5):
    return 1.0 / (1.0 + math.exp(-k * (x - mid)))


def _target_from_activation(activation, joint):
    """Map a 0..1 motor activation to a target joint angle (degrees)."""
    max_a = MAX_ANGLE[joint]
    return activation ** 0.85 * max_a


# ---------------------------------------------------------------- pre-designed postures
POSTURES = {
    "open": dict(
        joint_targets={f: {j: 5.0 for j in JOINTS[f]} for f in FINGERS},
        spread=0.25, opp=0.05, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "fist": dict(
        joint_targets={
            "thumb": {"cmc": 30.0, "mcp": 35.0, "ip": 20.0},
            **{f: {"mcp": 88.0, "pip": 95.0, "dip": 70.0} for f in ("index", "middle", "ring", "pinky")},
        },
        spread=0.0, opp=0.55, wrist={"pitch": 0.12, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "pinch": dict(
        joint_targets={
            "thumb": {"cmc": 42.0, "mcp": 55.0, "ip": 30.0},
            "index": {"mcp": 55.0, "pip": 45.0, "dip": 25.0},
            "middle": {"mcp": 12.0, "pip": 10.0, "dip": 5.0},
            "ring": {"mcp": 8.0, "pip": 5.0, "dip": 3.0},
            "pinky": {"mcp": 6.0, "pip": 4.0, "dip": 2.0},
        },
        spread=0.05, opp=0.72, wrist={"pitch": 0.08, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "ok": dict(
        joint_targets={
            "thumb": {"cmc": 45.0, "mcp": 60.0, "ip": 35.0},
            "index": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
            "middle": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
            "ring": {"mcp": 6.0, "pip": 4.0, "dip": 2.0},
            "pinky": {"mcp": 5.0, "pip": 3.0, "dip": 2.0},
        },
        spread=0.08, opp=0.70, wrist={"pitch": 0.05, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "point": dict(
        joint_targets={
            "thumb": {"cmc": 25.0, "mcp": 40.0, "ip": 20.0},
            "index": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
            "middle": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "ring": {"mcp": 88.0, "pip": 95.0, "dip": 70.0},
            "pinky": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
        },
        spread=0.08, opp=0.08, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "shaka": dict(
        joint_targets={
            "thumb": {"cmc": 25.0, "mcp": 25.0, "ip": 15.0},
            "index": {"mcp": 82.0, "pip": 90.0, "dip": 68.0},
            "middle": {"mcp": 85.0, "pip": 92.0, "dip": 70.0},
            "ring": {"mcp": 85.0, "pip": 92.0, "dip": 70.0},
            "pinky": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
        },
        spread=0.35, opp=0.12, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "rock": dict(
        joint_targets={
            "thumb": {"cmc": 20.0, "mcp": 30.0, "ip": 15.0},
            "index": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
            "middle": {"mcp": 85.0, "pip": 92.0, "dip": 70.0},
            "ring": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
            "pinky": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
        },
        spread=0.30, opp=0.10, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "spock": dict(
        joint_targets={
            "thumb": {"cmc": 18.0, "mcp": 28.0, "ip": 14.0},
            "index": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
            "middle": {"mcp": 8.0, "pip": 6.0, "dip": 4.0},
            "ring": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
            "pinky": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
        },
        spread=0.45, opp=0.08, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "ripple": dict(
        joint_targets={
            "thumb": {"cmc": 10.0, "mcp": 10.0, "ip": 6.0},
            "index": {"mcp": 35.0, "pip": 40.0, "dip": 28.0},
            "middle": {"mcp": 60.0, "pip": 65.0, "dip": 48.0},
            "ring": {"mcp": 82.0, "pip": 88.0, "dip": 65.0},
            "pinky": {"mcp": 88.0, "pip": 94.0, "dip": 70.0},
        },
        spread=0.20, opp=0.10, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "reach": dict(
        joint_targets={
            f: {"mcp": 18.0, "pip": 12.0, "dip": 8.0} for f in ("index", "middle", "ring", "pinky")
        },
        thumb={"cmc": 10.0, "mcp": 12.0, "ip": 8.0},
        spread=0.35, opp=0.10, wrist={"pitch": 0.25, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "grasp": dict(
        joint_targets={
            "thumb": {"cmc": 48.0, "mcp": 58.0, "ip": 35.0},
            "index": {"mcp": 78.0, "pip": 85.0, "dip": 62.0},
            "middle": {"mcp": 82.0, "pip": 90.0, "dip": 68.0},
            "ring": {"mcp": 80.0, "pip": 86.0, "dip": 64.0},
            "pinky": {"mcp": 75.0, "pip": 82.0, "dip": 58.0},
        },
        spread=0.08, opp=0.62, wrist={"pitch": 0.15, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "release": dict(
        joint_targets={f: {j: 6.0 for j in JOINTS[f]} for f in FINGERS},
        spread=0.28, opp=0.0, wrist={"pitch": -0.15, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "wave": dict(
        joint_targets={
            "thumb": {"cmc": 14.0, "mcp": 12.0, "ip": 8.0},
            **{f: {"mcp": 20.0, "pip": 16.0, "dip": 10.0} for f in ("index", "middle", "ring", "pinky")},
        },
        spread=0.10, opp=0.15, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "middle_finger": dict(
        joint_targets={
            "thumb": {"cmc": 45.0, "mcp": 50.0, "ip": 35.0},
            "index": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "middle": {"mcp": 6.0, "pip": 5.0, "dip": 3.0},
            "ring": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "pinky": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
        },
        spread=0.05, opp=0.30, wrist={"pitch": 0.0, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
    "thumbs_up": dict(
        joint_targets={
            # Full opposition twists the thumb's fold axis so it arcs up over
            # the palm instead of lying flat across it; mcp/ip stay nearly
            # straight so it reads as a long raised thumb, not a curled nub.
            "thumb": {"cmc": 40.0, "mcp": 10.0, "ip": 5.0},
            "index": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "middle": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "ring": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
            "pinky": {"mcp": 85.0, "pip": 92.0, "dip": 68.0},
        },
        spread=0.05, opp=1.0, wrist={"pitch": -0.2, "yaw": 0.0}, forearm={"roll": 0.0},
    ),
}
# fill in missing thumb targets for reach
POSTURES["reach"]["joint_targets"]["thumb"] = POSTURES["reach"].pop("thumb")

GOAL_POSTURES = {k: v for k, v in POSTURES.items() if k in ("reach", "grasp", "release", "point", "wave", "fist", "ripple", "pinch", "ok", "shaka", "rock", "spock", "middle_finger", "thumbs_up")}


@dataclass
class Tendon:
    name: str
    max_retraction: float = 3.2       # cm at full activation
    stiffness: float = 90.0           # N/cm of strain
    max_force: float = 95.0           # N at full active pull
    activation: float = 0.0
    motor_retraction: float = 0.0
    force: float = 0.0
    slip: float = 0.0

    def update(self, dt, path_delta):
        target_retraction = self.activation * self.max_retraction
        self.motor_retraction += (target_retraction - self.motor_retraction) * min(1.0, 7.0 * dt)
        # path_delta: tendon path length change from joint rotation (cm).
        # Flexion shortens the flexor path (negative) and stretches the
        # extensor path (positive) — the stretch is what lets an extensor
        # hold a joint against its flexor at mid-range angles.
        strain = max(0.0, self.motor_retraction + path_delta)
        passive = self.stiffness * strain
        active = self.max_force * _sigmoid(strain, k=25.0, mid=0.10) * _sigmoid(self.activation, k=12.0, mid=0.15)
        self.force = passive + active
        self.slip = 0.003 * self.force * (0.5 + 0.5 * math.sin(time.monotonic() * 17.0))
        return self.force


@dataclass
class Joint:
    name: str
    finger: str
    angle: float = 0.0
    velocity: float = 0.0
    heat: float = 0.0
    strain: float = 0.0
    contact: float = 0.0
    flexor: Tendon = None
    extensor: Tendon = None

    def __post_init__(self):
        if self.flexor is None:
            self.flexor = Tendon(f"flexor_{self.name}")
        if self.extensor is None:
            self.extensor = Tendon(f"extensor_{self.name}")
            self.extensor.activation = 0.06

    def step(self, dt, target_angle, load_contact, tremor, stress):
        max_a = MAX_ANGLE[self.name]
        target_angle = _clamp(target_angle, 0.0, max_a)

        base = _clamp((target_angle / max_a) ** 0.85)
        # closed-loop correction: pure feedforward overshoots badly in the
        # mid-range, so trim both antagonists by the tracking error
        err = (target_angle - self.angle) / max_a
        desired_activation = _clamp(base + 1.4 * err)
        self.flexor.activation = desired_activation
        coact = 0.04 + 0.22 * stress
        self.extensor.activation = 0.06 + coact + 0.15 * (self.angle / max_a) + _clamp(-1.2 * err, 0.0, 0.6)

        ma_f, ma_e = MOMENT_ARM[self.name]
        stretch = math.radians(self.angle)
        ff = self.flexor.update(dt, -stretch * ma_f)
        fe = self.extensor.update(dt, +stretch * ma_e)
        torque = ff * ma_f - fe * ma_e

        damp = DAMPING * self.velocity + 0.3 * math.tanh(self.velocity * 0.15)
        # hard joint limits: cancel torque driving into the stop. Velocity
        # into the stop is zeroed at the clamp below, so a pose held at a
        # limit settles to ~zero velocity instead of chattering forever.
        if self.angle <= 0.0 and torque < 0.0:
            torque = 0.0
        if self.angle >= max_a and torque > 0.0:
            torque = 0.0

        self.contact = 0.0
        if self.angle >= max_a - 2.0 and desired_activation > 0.70:
            self.contact = desired_activation
        if load_contact and desired_activation > 0.55:
            self.contact = max(self.contact, 0.5)

        accel = (torque * 100.0 - damp) / INERTIA
        accel += tremor

        self.velocity += accel * dt
        self.velocity = max(-360.0, min(360.0, self.velocity))
        self.angle += self.velocity * dt
        if self.angle < 0.0:
            self.angle = 0.0
            if self.velocity < 0.0:
                self.velocity = 0.0
        if self.angle > max_a:
            self.angle = max_a
            if self.velocity > 0.0:
                self.velocity = 0.0

        work = abs(self.velocity * math.radians(1) * torque) + 0.08 * ff + 0.05 * fe
        self.heat = max(0.0, self.heat + (work * 0.0008 - 0.008) * dt)
        self.strain = _clamp((ff / self.flexor.max_force) * (0.5 + 0.5 * self.contact))

        return abs(ff), abs(fe)


class HandModel:
    def __init__(self):
        self.joints = {f: {j: Joint(j, f) for j in JOINTS[f]} for f in FINGERS}
        self.joint_target = {f: {j: 0.0 for j in JOINTS[f]} for f in FINGERS}
        self.curl_target = {f: 0.0 for f in FINGERS}
        self.spread_target = 0.0
        self.thumb_opposition = 0.0
        self.wrist_target = {"pitch": 0.0, "yaw": 0.0}
        self.forearm_target = {"roll": 0.0}
        self.wrist = {"pitch": 0.0, "yaw": 0.0}
        self.forearm = {"roll": 0.0}
        self.load_contact = False
        self.goal = None
        self.emotion = {"stress": 0.2, "arousal": 0.3, "mood": "calm"}
        self._tremor_phase = 0.0
        self._lock = threading.Lock()
        self._pose_duration = 2.0

    def set_posture(self, spec):
        name = str(spec.get("name", "")).lower()
        if name not in POSTURES:
            raise ValueError(f"unknown posture {name!r}; try {sorted(POSTURES)}")
        dur = _clamp(spec.get("duration_s", 3.0), 0.2, 15.0)
        p = POSTURES[name]
        with self._lock:
            self.goal = {"kind": f"posture:{name}", "until": time.monotonic() + dur}
            self._apply_posture_dict(p)

    def _apply_posture_dict(self, p):
        jt = p.get("joint_targets", {})
        for f, joints in jt.items():
            if f not in FINGERS:
                continue
            for j, deg in joints.items():
                if j in self.joint_target[f]:
                    self.joint_target[f][j] = _clamp(float(deg), 0.0, MAX_ANGLE[j])
        # curl targets derived from joints so the UI stays consistent
        for f in FINGERS:
            angles = [self.joint_target[f][j] / MAX_ANGLE[j] for j in JOINTS[f]]
            self.curl_target[f] = sum(angles) / len(angles)
        if "spread" in p:
            self.spread_target = _clamp(p["spread"], 0.0, 1.0)
        if "opp" in p:
            self.thumb_opposition = _clamp(p["opp"])
        if "wrist" in p:
            for ax in ("pitch", "yaw"):
                if ax in p["wrist"]:
                    self.wrist_target[ax] = max(-1.0, min(1.0, float(p["wrist"][ax])))
        if "forearm" in p:
            for ax in ("roll",):
                if ax in p["forearm"]:
                    self.forearm_target[ax] = max(-1.0, min(1.0, float(p["forearm"][ax])))

    def set_pose(self, spec):
        with self._lock:
            # explicit joint targets win over curl
            jt = spec.get("joint_targets") or {}
            for f, joints in jt.items():
                if f not in FINGERS:
                    continue
                for j, deg in joints.items():
                    if j in self.joint_target[f]:
                        self.joint_target[f][j] = _clamp(float(deg), 0.0, MAX_ANGLE[j])

            t = spec.get("finger_targets") or {}
            for f in FINGERS:
                if f in t:
                    self.curl_target[f] = _clamp(t[f])
                    # spread curl across joints proportionally
                    for i, j in enumerate(JOINTS[f]):
                        mix = [0.45, 0.40, 0.25][i] if len(JOINTS[f]) == 3 else [0.55, 0.45]
                        self.joint_target[f][j] = self.curl_target[f] * mix * MAX_ANGLE[j]
            if "curl" in spec:
                for f in ("index", "middle", "ring", "pinky"):
                    self.curl_target[f] = _clamp(spec["curl"])
                    for j in JOINTS[f]:
                        self.joint_target[f][j] = self.curl_target[f] * MAX_ANGLE[j]
                self.curl_target["thumb"] = _clamp(spec["curl"] * 0.75)
                for j in JOINTS["thumb"]:
                    self.joint_target["thumb"][j] = self.curl_target["thumb"] * MAX_ANGLE[j]
            if "fist" in spec and spec["fist"]:
                self._apply_posture_dict(POSTURES["fist"])
            if "spread" in spec:
                self.spread_target = _clamp(spec["spread"], 0.0, 1.0)
            if "thumb_opposition" in spec:
                self.thumb_opposition = _clamp(spec["thumb_opposition"])
            w = spec.get("wrist") or {}
            for ax in ("pitch", "yaw"):
                if ax in w:
                    self.wrist_target[ax] = max(-1.0, min(1.0, float(w[ax])))
            fa = spec.get("forearm") or {}
            for ax in ("roll",):
                if ax in fa:
                    self.forearm_target[ax] = max(-1.0, min(1.0, float(fa[ax])))
            self._pose_duration = _clamp(spec.get("duration_s", 2.0), 0.05, 10.0)
            self.goal = None

    def set_actuator(self, spec):
        """Drive a single tendon actuator by activation 0..1."""
        f = str(spec.get("finger", ""))
        j = str(spec.get("joint", ""))
        side = str(spec.get("side", "flexor")).lower()
        activation = _clamp(spec.get("activation", 0.0))
        if f not in FINGERS or j not in JOINTS[f]:
            raise ValueError(f"invalid actuator {f}/{j}")
        with self._lock:
            tendon = self.joints[f][j].flexor if side == "flexor" else self.joints[f][j].extensor
            tendon.activation = activation
            # also nudge the target so the UI slider reflects the change
            self.joint_target[f][j] = _target_from_activation(activation, j)

    def set_goal(self, spec):
        kind = str(spec.get("kind", "")).lower()
        dur = _clamp(spec.get("duration_s", 3.0), 0.3, 15.0)
        if kind not in GOAL_POSTURES:
            raise ValueError(f"unknown goal {kind!r}; kinds: {sorted(GOAL_POSTURES)}")
        with self._lock:
            self.goal = {"kind": kind, "until": time.monotonic() + dur, "wave_phase": 0.0}
            self._apply_posture_dict(GOAL_POSTURES[kind])
            self.load_contact = kind == "grasp"

    def set_emotion(self, spec):
        with self._lock:
            for k in ("stress", "arousal"):
                if k in spec:
                    self.emotion[k] = _clamp(spec[k])
            if "mood" in spec:
                self.emotion["mood"] = str(spec["mood"])[:32]

    def step(self, dt):
        with self._lock:
            stress = self.emotion["stress"]
            arousal = self.emotion["arousal"]
            self._tremor_phase += dt * (8.0 + 12.0 * arousal)
            tremor_amp = 0.40 * stress + 0.20 * arousal

            if self.goal and self.goal["kind"] == "wave":
                self.goal["wave_phase"] += dt * 5.5
                s = math.sin(self.goal["wave_phase"])
                # yaw alone renders at 0.25 gain (a weak swivel); forearm roll
                # renders at 0.6, so together the whole hand rocks side to side
                self.wrist_target["yaw"] = s * 0.6
                self.forearm_target["roll"] = s * 0.5
            if self.goal and time.monotonic() > self.goal["until"]:
                for f in FINGERS:
                    self.curl_target[f] = min(self.curl_target[f], 0.12)
                    for j in JOINTS[f]:
                        self.joint_target[f][j] = min(self.joint_target[f][j], 15.0)
                self.spread_target = 0.15
                self.thumb_opposition = 0.05
                self.wrist_target = {"pitch": 0.0, "yaw": 0.0}
                self.forearm_target = {"roll": 0.0}
                self.load_contact = False
                self.goal = None

            for f in FINGERS:
                for j in JOINTS[f]:
                    target = self.joint_target[f][j]
                    if self.curl_target[f] and not self.joint_target[f][j]:
                        target = self.curl_target[f] * MAX_ANGLE[j]
                    tremor = tremor_amp * math.sin(self._tremor_phase * 1.3 + hash(j) % 7)
                    self.joints[f][j].step(dt, target, self.load_contact, tremor, stress)

            for ax in ("pitch", "yaw"):
                err = self.wrist_target[ax] - self.wrist[ax]
                self.wrist[ax] += err * min(1.0, 5.0 * dt)
            for ax in ("roll",):
                err = self.forearm_target[ax] - self.forearm[ax]
                self.forearm[ax] += err * min(1.0, 4.0 * dt)

    def snapshot(self):
        with self._lock:
            fingers = {}
            for f in FINGERS:
                joints = {}
                for j, jd in self.joints[f].items():
                    joints[j] = {
                        "angle": round(jd.angle, 2),
                        "flex_force": round(jd.flexor.force, 2),
                        "ext_force": round(jd.extensor.force, 2),
                        "flex_activation": round(jd.flexor.activation, 3),
                        "ext_activation": round(jd.extensor.activation, 3),
                        "strain": round(jd.strain, 3),
                        "heat": round(jd.heat, 3),
                        "velocity": round(jd.velocity, 1),
                        "contact": round(jd.contact, 2),
                        "target": round(self.joint_target[f][j], 2),
                    }
                fingers[f] = {
                    "curl_target": round(self.curl_target[f], 3),
                    "segments": [
                        {"name": n, "len": l} for n, l in SEG_LENGTH[f]
                    ],
                    "joints": joints,
                }
            return {
                "fingers": fingers,
                "spread": round(self.spread_target, 3),
                "thumb_opposition": round(self.thumb_opposition, 3),
                "wrist": {k: round(v, 3) for k, v in self.wrist.items()},
                "wrist_target": dict(self.wrist_target),
                "forearm": {k: round(v, 3) for k, v in self.forearm.items()},
                "forearm_target": dict(self.forearm_target),
                "goal": (self.goal or {}).get("kind"),
                "load_contact": self.load_contact,
                "emotion": dict(self.emotion),
                "time": time.time(),
            }


class EventBus:
    def __init__(self):
        self._subs = []
        self._lock = threading.Lock()

    def subscribe(self):
        q = queue.Queue(maxsize=60)
        with self._lock:
            self._subs.append(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            if q in self._subs:
                self._subs.remove(q)

    def publish(self, obj):
        data = json.dumps(obj)
        with self._lock:
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass


HERE = Path(__file__).resolve().parent
ROOT_PAGE = Path("/var/home/a/code/robot-hand/index.html")
STATIC = {
    "/": ROOT_PAGE,
    "/viewer.js": HERE / "viewer.js",
    "/three.min.js": HERE / "vendor" / "three.min.js",
}
MIME = {".html": "text/html; charset=utf-8", ".js": "application/javascript; charset=utf-8", ".css": "text/css"}


class Handler(BaseHTTPRequestHandler):
    server_version = "hand-bridge/0.3"
    protocol_version = "HTTP/1.1"

    def _json(self, obj, status=HTTPStatus.OK):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 64_000:
            raise ValueError("bad content length")
        return json.loads(self.rfile.read(n).decode())

    def log_message(self, fmt, *args):
        log.debug("%s %s", self.address_string(), fmt % args)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/healthz":
            return self._json({"ok": True})
        if path == "/arm":
            return self._json(self.server.model.snapshot())
        if path == "/postures":
            return self._json({"postures": sorted(POSTURES)})
        if path == "/events":
            return self._sse()
        if path in STATIC:
            p = STATIC[path]
        elif path.startswith('/bridge/vendor/'):
            p = (HERE / 'vendor' / path[len('/bridge/vendor/'):]).resolve()
            if not str(p).startswith(str(HERE / 'vendor')):
                return self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        elif path == '/' or path.endswith('.html') or path.endswith('.js') or path.endswith('.css') or path.endswith('.png') or path.endswith('.svg'):
            root = ROOT_PAGE.parent
            p = (root / path.lstrip('/')).resolve()
            if not str(p).startswith(str(root)):
                return self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        else:
            return self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        if not p.is_file():
            return self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        data = p.read_bytes()
        ext = p.suffix
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", MIME.get(ext, "application/octet-stream"))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        if ext in (".js", ".css"):
            self.send_header("Cache-Control", "max-age=3600")
        self.end_headers()
        self.wfile.write(data)
        return
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            body = self._read_json()
        except Exception as exc:
            return self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        try:
            if path == "/pose":
                self.server.model.set_pose(body)
                return self._json({"ok": True, "state": self.server.model.snapshot()})
            if path == "/posture":
                self.server.model.set_posture(body)
                return self._json({"ok": True, "posture": body.get("name"), "state": self.server.model.snapshot()})
            if path == "/goal":
                self.server.model.set_goal(body)
                return self._json({"ok": True, "goal": body.get("kind"), "state": self.server.model.snapshot()})
            if path == "/actuator":
                self.server.model.set_actuator(body)
                return self._json({"ok": True, "state": self.server.model.snapshot()})
            if path == "/emotion":
                self.server.model.set_emotion(body)
                return self._json({"ok": True})
        except Exception as exc:
            return self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _sse(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        q = self.server.bus.subscribe()
        try:
            self.wfile.write(b": connected\n\n")
            while True:
                try:
                    data = q.get(timeout=10.0)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    continue
                self.wfile.write(f"data: {data}\n\n".encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.server.bus.unsubscribe(q)


def run(port=8765):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    model = HandModel()
    bus = EventBus()
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.model = model
    server.bus = bus

    def sim_loop():
        t0 = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            dt = min(0.02, now - t0)
            t0 = now
            model.step(dt)
            if int(now * 10) % 2 == 0:
                bus.publish({"type": "arm", "state": model.snapshot()})

    threading.Thread(target=sim_loop, daemon=True).start()
    log.info("hand bridge on http://127.0.0.1:%s", port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    run(args.port)
