import math
import random

import pytest

from byte_leg_control.kinematics import (
    LegGeometry,
    forward_kinematics,
    inverse_kinematics,
)


GEOM = LegGeometry(thigh_length=0.30, shank_length=0.30)


@pytest.mark.parametrize('seed', list(range(50)))
def test_ik_fk_round_trip(seed: int) -> None:
    rng = random.Random(seed)
    # Sample targets on a sphere of radius in (|L1-L2|+eps, L1+L2-eps).
    r_min = abs(GEOM.thigh_length - GEOM.shank_length) + 1e-3
    r_max = GEOM.thigh_length + GEOM.shank_length - 1e-3
    r = rng.uniform(r_min, r_max)
    # Restrict to the hemisphere where the leg hangs (z <= 0), and
    # keep y/z such that -z is positive (leg isn't flipped upright).
    theta = rng.uniform(-math.pi / 3, math.pi / 3)   # abduct angle
    phi = rng.uniform(-math.pi / 2.5, math.pi / 2.5)  # fore/aft angle
    y = r * math.sin(theta) * math.cos(phi)
    x = r * math.sin(phi)
    z = -r * math.cos(theta) * math.cos(phi)

    q_haa, q_hp, q_k = inverse_kinematics(x, y, z, GEOM)
    x2, y2, z2 = forward_kinematics(q_haa, q_hp, q_k, GEOM)

    assert math.isfinite(q_haa)
    assert math.isfinite(q_hp)
    assert math.isfinite(q_k)
    assert abs(x - x2) < 1e-6
    assert abs(y - y2) < 1e-6
    assert abs(z - z2) < 1e-6


def test_ik_clamps_out_of_reach() -> None:
    # Reach well beyond L1+L2=0.6; solver must clamp and produce finite angles.
    q_haa, q_hp, q_k = inverse_kinematics(0.0, 0.0, -10.0, GEOM)
    for q in (q_haa, q_hp, q_k):
        assert math.isfinite(q)
    # At full extension along -z the knee should be ~0 (leg almost straight).
    assert abs(q_k) < 0.1


def test_ik_too_close() -> None:
    # Target inside |L1-L2| (which is 0 here). Origin is the degenerate case.
    q_haa, q_hp, q_k = inverse_kinematics(0.0, 0.0, 0.0, GEOM)
    for q in (q_haa, q_hp, q_k):
        assert math.isfinite(q)
