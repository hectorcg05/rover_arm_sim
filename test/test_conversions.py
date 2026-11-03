"""Criterio 2: las conversiones del emulador dan los mismos valores que las funciones de los drivers
reales, con varios ángulos por joint (incluye fuera de rango y bordes)."""

import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from real_code import DRIVE_YAML, TalonHarness, epos_launch_params, real_epos_node  # noqa: E402

from rover_arm_sim.conversions import EposConversion, TalonConversion  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_sim.yaml')


@pytest.fixture(scope='module')
def cfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def harness():
    with open(DRIVE_YAML) as f:
        p = yaml.safe_load(f)['arm_drive_ctre']['ros__parameters']
    h = TalonHarness([p['j1_deg_min'], p['j1_deg_max'], p['j1_ticks_min'], p['j1_ticks_max'],
                      p['j4_deg_min'], p['j4_deg_max'], p['j4_ticks_min'], p['j4_ticks_max']])
    yield h
    h.close()


ANGLES = [-200.0, -150.0001, -150.0, -120.5, -85.0001, -85.0, -84.99, -45.3, -10.0, -0.05, 0.0, 0.05,
          12.345, 45.0, 89.999, 90.0, 90.0001, 120.0, 149.999, 150.0, 150.0001, 179.0]
TICKS = [-6000, -5617, -4000, -3000, -2206, -186, 0, 100, 867, 1000, 1862, 2500, 4096]


@pytest.mark.parametrize('joint', [1, 4])
def test_talon_command_matches_drive_can_tank(cfg, harness, joint):
    conv = TalonConversion.from_cfg(cfg['joints'][f'joint{joint}']['talon_motion_magic'])
    log = cfg['joints'][f'joint{joint}']['log_name']
    for a in ANGLES:
        ok_real, target_real, warn = harness.command(joint, a)
        ok, angle, target = conv.command(a)
        assert ok == ok_real, a
        if ok:
            assert target == pytest.approx(target_real, abs=1e-9), a
            assert warn is None
        else:
            # Mismo texto de WARN que el real
            expected = f'{log} angle {angle:.1f} out of range [{conv.deg_min:.1f}, {conv.deg_max:.1f}]'
            assert warn == expected, (warn, expected)


@pytest.mark.parametrize('joint', [1, 4])
def test_talon_feedback_matches_drive_can_tank(cfg, harness, joint):
    conv = TalonConversion.from_cfg(cfg['joints'][f'joint{joint}']['talon_motion_magic'])
    for t in TICKS:
        assert conv.ticks_to_deg(t) == pytest.approx(harness.ticks_to_deg(joint, t), abs=1e-9), t


@pytest.mark.parametrize('joint', [1, 4])
def test_talon_round_trip(cfg, joint):
    conv = TalonConversion.from_cfg(cfg['joints'][f'joint{joint}']['talon_motion_magic'])
    for a in (-80.0, -1.0, 0.0, 33.3, 85.0):
        assert conv.ticks_to_deg(conv.deg_to_ticks(a)) == pytest.approx(a)


@pytest.fixture(scope='module')
def epos():
    return real_epos_node(epos_launch_params())


@pytest.mark.parametrize('joint', [2, 3])
def test_epos_command_matches_joint_control(cfg, epos, joint):
    node, _ns = epos
    conv = EposConversion.from_cfg(cfg['joints'][f'joint{joint}']['epos_ppm'])
    real_cb = node.cb_joint2 if joint == 2 else node.cb_joint3
    fake = node.epos_j2 if joint == 2 else node.epos_j3

    class Msg:
        def __init__(self, data):
            self.data = data
    for a in ANGLES + [-160.0, -159.99, -160.01, -146.0, -5.0, 170.0, 170.01, -5.01, 168.4]:
        real_cb(Msg(a))
        real_target = fake.positions[-1][0]
        _angle, target = conv.command(a)
        assert target == real_target, a
        assert isinstance(target, int)


@pytest.mark.parametrize('joint', [2, 3])
def test_epos_feedback_matches_joint_control(cfg, epos, joint):
    node, _ns = epos
    conv = EposConversion.from_cfg(cfg['joints'][f'joint{joint}']['epos_ppm'])
    real = node._counts_to_deg_j2 if joint == 2 else node._counts_to_deg_j3
    for c in (-800000, -728178, -664462, -28800, -1, 0, 1, 123456, 773689, 979200, 1000000):
        assert conv.counts_to_deg(c) == pytest.approx(real(float(c)), abs=1e-12), c


@pytest.mark.parametrize('joint', [2, 3])
def test_epos_virtual_home_matches_joint_control(cfg, epos, joint):
    """El home virtual del emulador usa la misma fórmula (int(my_map(...))) que _init_epos."""
    node, _ns = epos
    jcfg = cfg['joints'][f'joint{joint}']['epos_ppm']
    conv = EposConversion.from_cfg(jcfg)
    real_cfg = dict(node.j2_cfg if joint == 2 else node.j3_cfg)
    for angle in (jcfg['initial_angle'], real_cfg['initial_angle'], 0.0, 12.7):
        real_cfg['initial_angle'] = angle
        fake = node._init_epos(f'Joint{joint}', real_cfg, 'x.eds')
        assert conv.initial_counts(angle) == fake.home
        lo, hi = conv.drive_limits(int(jcfg['drive_limit_margin_counts']))
        assert (lo, hi) == (fake.ppm['pos_min'], fake.ppm['pos_max'])
        assert fake.ppm['max_vel'] == jcfg['profile_velocity'] * jcfg['max_velocity_factor']


def test_talon_limits_are_float32_like_drive_can_tank():
    """Con límites que no caben exactos en float, el emulador redondea igual que el C++."""
    lim = [-85.3, 90.1, 1862.7, -186.1, -150.2, 150.3, -2206.9, -5617.1]
    h = TalonHarness(lim)
    try:
        c1 = TalonConversion(*lim[:4], invert=False, phys_error=0.0, add_phys_error=True)
        c4 = TalonConversion(*lim[4:], invert=True)
        for a in (-80.0, 0.0, 33.3, 89.0):
            assert c1.command(a)[2] == pytest.approx(h.command(1, a)[1], abs=1e-9)
            assert c4.command(a)[2] == pytest.approx(h.command(4, a)[1], abs=1e-9)
    finally:
        h.close()
