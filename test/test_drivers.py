"""Modelos de driver: rechazos y recortes (criterio 4), perfiles y velocidades del YAML (criterios 5 y
9), fallas por error de seguimiento y su reinicio (criterio 6), servo, gripper y torque."""

import copy
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from fakes import FakeClock, FakeNode  # noqa: E402

from rover_arm_sim.drivers import DRIVER_MODELS, model_class  # noqa: E402
from rover_arm_sim.drivers.gripper import DcGripper  # noqa: E402
from rover_arm_sim.drivers.groups import EposGroup, TalonGroup  # noqa: E402
from rover_arm_sim.profiles import TrapezoidProfile  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_sim.yaml')
DT = 0.01


@pytest.fixture
def cfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def make(cfg, key, start):
    jcfg = cfg['joints'][key]
    node, clock = FakeNode(), FakeClock()
    gcfg = cfg['driver_nodes'][jcfg['driver_node']]
    group = {'talon_group': TalonGroup, 'epos_group': EposGroup}.get(gcfg['kind'])
    model = model_class(jcfg['driver_model'])(key, jcfg, node, clock)
    if group:
        g = group(jcfg['driver_node'], gcfg, node, clock)
        g.add(model)
    model.start(start)
    return model, node, clock


def run(model, clock, seconds, measured=None, follow=True):
    """Avanza el modelo; si follow, Gazebo sigue la orden perfectamente (salvo `measured` fijo)."""
    pos = model.gazebo_command(0.0) if measured is None else measured
    for _ in range(int(round(seconds / DT))):
        clock.t += DT
        model.update(clock.t, DT, pos)
        if follow and measured is None:
            pos = model.gazebo_command(pos)
    return pos


# ------------------------------------------------------------------------------- perfil
def test_profile_time_matches_trapezoid():
    p = TrapezoidProfile(0.0, 10.0, 20.0)
    p.set_target(10.0)                  # 0.5 s acelerando + 0.5 s frenando + 0.5 s de crucero
    assert p.remaining_time() == pytest.approx(1.5)
    t = 0.0
    while not p.done:
        p.step(0.001)
        t += 0.001
    assert t == pytest.approx(1.5, abs=0.002) and p.x == 10.0


def test_profile_replans_from_current_velocity():
    p = TrapezoidProfile(0.0, 100.0, 50.0)
    p.set_target(1000.0)
    for _ in range(300):
        p.step(0.01)
    assert p.v == pytest.approx(100.0)
    p.set_target(p.x + 5.0)            # objetivo más cerca que la distancia de frenado: se pasa y vuelve
    peak = p.x
    while not p.done:
        p.step(0.01)
        peak = max(peak, p.x)
    assert peak == pytest.approx(300.0, abs=0.5) and p.x == pytest.approx(205.0)


# ------------------------------------------------------------------------------- Talon (J1, J4)
@pytest.mark.parametrize('key,bad', [('joint1', 90.5), ('joint1', -86.0), ('joint4', 151.0), ('joint4', -150.2)])
def test_talon_rejects_out_of_range_with_same_warn(cfg, key, bad):
    m, node, clock = make(cfg, key, 0.0)
    node.send(cfg['joints'][key]['command_topic'], bad)
    run(m, clock, 1.0)
    assert m.gazebo_command(0.0) == pytest.approx(0.0, abs=1e-9)    # no se movió
    c = cfg['joints'][key]['talon_motion_magic']
    assert node.logger.of('warn') == [
        f'{cfg["joints"][key]["log_name"]} angle {bad:.1f} out of range [{c["deg_min"]:.1f}, {c["deg_max"]:.1f}]']


@pytest.mark.parametrize('key,speed', [('joint1', 25 * 10 / (2048 / 175.0)), ('joint4', 30 * 10 / (3411 / 300.0))])
def test_talon_cruise_speed(cfg, key, speed):
    """J1 = 21.4 °/s y J4 = 26.4 °/s (reporte §3.2), salen de crucero 25/30 ticks/100 ms."""
    m, node, clock = make(cfg, key, 0.0)
    node.send(cfg['joints'][key]['command_topic'], 60.0)
    run(m, clock, 1.5)
    a = m.gazebo_command(0.0)
    run(m, clock, 0.5)
    assert (m.gazebo_command(0.0) - a) / 0.5 == pytest.approx(speed, rel=1e-3)


def test_talon_sync_velocity_clamped_and_truncated(cfg):
    m, node, clock = make(cfg, 'joint1', 0.0)
    node.send('/arm_sync/joint1_velocity', 50.7)        # drive_can_tank.cpp:334 → 9
    assert m.cruise == 9.0
    node.send('/arm_sync/joint1_velocity', 4.9)         # int(4.9) = 4
    assert m.cruise == 4.0


def test_talon_disable_after_10s_without_messages(cfg):
    m, node, clock = make(cfg, 'joint1', 0.0)
    node.send('/arm_teleop/joint1', 80.0)
    run(m, clock, 1.0)
    pos = m.gazebo_command(0.0)
    clock.t += 10.0                                     # pasan 10 s sin mensajes (FeedEnable vence)
    m.update(clock.t, DT, pos)
    assert not m.group.enabled(clock.t)
    held = m.gazebo_command(pos)
    run(m, clock, 1.0, measured=pos)
    assert m.gazebo_command(pos) == pytest.approx(held)


# ------------------------------------------------------------------------------- EPOS (J2, J3)
@pytest.mark.parametrize('key,cmd,expected', [('joint2', 185.0, 170.0), ('joint2', -20.0, -5.0),
                                              ('joint3', -176.0, -160.0), ('joint3', 171.0, 170.0)])
def test_epos_clamps_silently(cfg, key, cmd, expected):
    start = cfg['startup']['pose_deg'][key]
    m, node, clock = make(cfg, key, start)
    node.send(cfg['joints'][key]['command_topic'], cmd)
    pos = run(m, clock, 30.0)
    assert pos == pytest.approx(expected, abs=1e-3)
    assert not node.logger.of('warn') and not m.faulted


def test_epos_logs_like_dual_epos_node(cfg):
    m, node, _clock = make(cfg, 'joint2', 170.0)
    assert node.logger.of('info')[:2] == ['Joint2: home establecido en 170.0° (979200 counts)',
                                          'Joint2 (ID 5): PPM trapezoidal ready']


def test_epos_speed_comes_from_yaml(cfg):
    """Criterio 9: cambiar speed_deg_s en el YAML cambia la velocidad sin tocar código."""
    times = {}
    for speed in (16.6, 8.3):
        c = copy.deepcopy(cfg)
        c['joints']['joint2']['epos_ppm']['speed_deg_s'] = speed
        m, node, clock = make(c, 'joint2', 170.0)
        node.send('/arm_teleop/joint2', 140.0)
        pos, t = 170.0, 0.0
        while abs(pos - 140.0) > 1e-6 and t < 20:
            clock.t += DT
            t += DT
            m.update(clock.t, DT, pos)
            pos = m.gazebo_command(pos)
        times[speed] = t
    # trapecio: 30° con a = 47.4 °/s² y la latencia de 20 ms
    assert times[16.6] == pytest.approx(30 / 16.6 + 16.6 / 47.4 + 0.02, abs=0.02)
    assert times[8.3] == pytest.approx(30 / 8.3 + 8.3 / 47.4 + 0.02, abs=0.02)


def test_epos_following_error_fault_and_reset(cfg):
    cfg['joints']['joint2']['epos_ppm']['initial_angle'] = 30.0
    m, node, clock = make(cfg, 'joint2', 30.0)
    node.send('/arm_teleop/joint2', 15.0)
    run(m, clock, 1.0, measured=30.0)                   # Gazebo no deja bajar: choque
    assert m.faulted
    m.group.check_faults()
    err = node.logger.of('error')
    assert err == ["Joint2 FAULT | codigo: 0x8611 — Following error | registro: ['Generic error', "
                   "'Motion error'] | historial: ['Following error']"]
    held = m.gazebo_command(30.0)
    node.send('/arm_teleop/joint2', 60.0)               # en falla no responde
    run(m, clock, 1.0, measured=held)
    assert m.gazebo_command(held) == held
    assert m.reset_fault(held)                          # botón "Reiniciar driver"
    m.group.check_faults()
    assert node.logger.of('info')[-1] == 'Joint2: falla resuelta, drive operativo'
    assert m.gazebo_command(held) == pytest.approx(held, abs=1e-3)   # no redeclara la posición
    node.send('/arm_teleop/joint2', 60.0)
    assert run(m, clock, 5.0) == pytest.approx(60.0, abs=1e-3)


def test_epos_small_lag_is_not_a_fault(cfg):
    """Gazebo se atrasa unos ms; dentro de sim_tracking_window_s no es falla."""
    m, node, clock = make(cfg, 'joint3', -146.0)
    node.send('/arm_teleop/joint3', -116.0)
    lagged, history = -146.0, []
    for _ in range(300):
        clock.t += DT
        m.update(clock.t, DT, lagged)
        history.append(m.gazebo_command(lagged))
        lagged = history[-4] if len(history) >= 4 else -146.0   # 30 ms de atraso
    assert not m.faulted


def test_epos_buffered_mode_waits_for_move(cfg):
    c = copy.deepcopy(cfg)
    c['joints']['joint2']['epos_ppm']['setpoint_mode'] = 'buffered'
    m, node, clock = make(c, 'joint2', 170.0)
    node.send('/arm_teleop/joint2', 160.0)
    run(m, clock, 0.1)
    node.send('/arm_teleop/joint2', 150.0)              # va al búfer
    node.send('/arm_teleop/joint2', 140.0)              # búfer lleno: se ignora
    run(m, clock, 0.3)
    assert m.profile.target == pytest.approx(m.conv.command(160.0)[1])
    pos = run(m, clock, 10.0)
    assert pos == pytest.approx(150.0, abs=1e-3)


# ------------------------------------------------------------------------------- J5 y gripper
def test_servo_clamps_and_rate_limits(cfg):
    m, node, clock = make(cfg, 'joint5', 0.0)
    node.send('/arm_teleop/joint5', 1641.5)             # salto ×57 (C27)
    run(m, clock, 0.5)
    assert m.gazebo_command(0.0) == pytest.approx(30.0)  # 60 °/s
    run(m, clock, 2.0)
    assert m.gazebo_command(0.0) == pytest.approx(90.0)


def test_gripper_stroke_time_and_stop_on_object(cfg):
    g = cfg['gripper']
    node, clock = FakeNode(), FakeClock()
    grip = DcGripper(g, node, clock, 0.0)
    lo, hi = grip.finger_limits()
    node.send(g['command_topic'], 1.0)                  # +1 = abrir (PENDIENTE P4)
    pos = [lo, lo]
    for _ in range(int(g['full_stroke_time_s'] / DT)):
        clock.t += DT
        grip.update(clock.t, DT, pos)
        pos = list(grip.finger_commands())
    assert grip.opening_from(*pos) == pytest.approx(g['stroke_m'], abs=1e-3)   # 17 cm en 4 s
    node.send(g['command_topic'], -1.0)                 # cerrar sobre un objeto de 12 cm
    stop = grip.finger_position(0.12)
    for _ in range(300):
        clock.t += DT
        grip.update(clock.t, DT, pos)
        pos = [max(stop, c) for c in grip.finger_commands()]
    assert grip.opening_from(*pos) == pytest.approx(0.12, abs=1e-6)
    assert all(grip.blocked) and grip.grasp_request() == 'hold'
    node.send(g['command_topic'], 0.0)
    grip.update(clock.t, DT, pos)
    assert grip.grasp_request() == 'release'            # hold_requires_button


def test_torque_model_contract(cfg):
    jcfg = copy.deepcopy(cfg['joints']['joint2'])
    jcfg['driver_model'] = 'torque'
    jcfg['torque'] = dict(cfg['torque_model_template'], limits_deg=[-5.0, 170.0])
    node, clock = FakeNode(), FakeClock()
    m = DRIVER_MODELS['torque']('joint2', jcfg, node, clock)
    m.start(170.0)
    node.send(jcfg['torque']['command_topic'], 5.0)     # se recorta a 0.20 N·m
    m.update(clock.t, DT, 170.0)
    assert m.gazebo_command(170.0) == pytest.approx(0.20 * jcfg['torque']['joint_gear_ratio'])
    clock.t += 0.6                                      # watchdog de 500 ms
    m.update(clock.t, DT, 170.0)
    assert m.gazebo_command(170.0) == 0.0
    m.publish_feedback()
    assert node.pubs[jcfg['torque']['feedback_deg_topic']].msgs[-1].data == pytest.approx(170.0)


def test_epos_virtual_home_offset_like_finding_13(cfg):
    """Hallazgo 13: con j2_initial_angle = 0 (valor del código) y el brazo físico en 170°, el driver
    declara 0° y todo J2 queda corrido 170°. La sim lo reproduce si se pone 0 en el YAML."""
    cfg['joints']['joint2']['epos_ppm']['initial_angle'] = 0.0
    m, node, clock = make(cfg, 'joint2', 170.0)
    m.update(clock.t, DT, 170.0)
    m.publish_feedback()
    assert node.pubs['/arm_feedback/joint2_deg'].msgs[-1].data == pytest.approx(0.0, abs=1e-3)
    node.send('/arm_teleop/joint2', 23.1)              # primer tick de R1/R2 (reporte §E.12)
    assert run(m, clock, 5.0) == pytest.approx(193.1, abs=1e-2)
