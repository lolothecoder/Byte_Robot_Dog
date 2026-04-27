"""Forward and inverse kinematics for the 3-DOF leg.

Frames: ROS REP-103. x forward, y left, z up. Hip frame origin is the
hip-abduct axis. At zero joint angles the leg hangs straight down along -z.

Joint axes:
    hip_abduct  : +x (rolls the leg sideways, moving foot in y/z)
    hip_pitch   : +y (pitches the thigh, moving foot in x/z)
    knee        : +y (flexes the shank relative to the thigh)

Convention here: q_knee >= 0 means the knee flexes so the shank rotates
from the thigh toward +x (foot tucks forward relative to thigh).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LegGeometry:
    thigh_length: float
    shank_length: float


def forward_kinematics(q_haa: float, q_hp: float, q_k: float,
                       geom: LegGeometry) -> tuple[float, float, float]:
    L1 = geom.thigh_length
    L2 = geom.shank_length
    x = L1 * math.sin(q_hp) + L2 * math.sin(q_hp + q_k)
    r = L1 * math.cos(q_hp) + L2 * math.cos(q_hp + q_k)
    y = r * math.sin(q_haa)
    z = -r * math.cos(q_haa)
    return x, y, z


def inverse_kinematics(x: float, y: float, z: float,
                       geom: LegGeometry) -> tuple[float, float, float]:
    L1 = geom.thigh_length
    L2 = geom.shank_length

    # Reach envelope: clamp the target to the annulus [|L1-L2|+eps, L1+L2-eps]
    # so acos never blows up on edge cases.
    eps = 1e-4
    d_min = abs(L1 - L2) + eps
    d_max = L1 + L2 - eps
    d = math.sqrt(x * x + y * y + z * z)
    if d < d_min or d > d_max:
        scale = max(d_min, min(d_max, d)) / max(d, 1e-9)
        x, y, z = x * scale, y * scale, z * scale

    q_haa = math.atan2(y, -z)
    r = math.sqrt(y * y + z * z)
    d2 = x * x + r * r
    cos_k = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    q_k = math.acos(cos_k)
    q_hp = math.atan2(x, r) - math.atan2(L2 * math.sin(q_k),
                                         L1 + L2 * math.cos(q_k))
    return q_haa, q_hp, q_k
