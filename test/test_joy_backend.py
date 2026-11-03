"""Joystick virtual: conversión igual a joy_node, dispositivo uinput tipo Attack 3 y respaldo /joy."""

import os
import sys
import types

import pytest
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from fakes import FakeClock, FakeNode  # noqa: E402

from rover_arm_sim.joy_backend import (TopicBackend, UinputBackend, joy_axis, raw_to_sdl_int16,  # noqa: E402
                                       sdl_to_raw, to_sdl)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_sim.yaml')


@pytest.fixture
def jcfg():
    with open(CONFIG) as f:
        return yaml.safe_load(f)['joystick']


def test_joy_axis_semantics():
    dz = 0.1
    assert joy_axis(0, dz) == 0.0
    assert joy_axis(int(32767 * 0.09), dz) == 0.0                 # dentro de la zona muerta
    assert joy_axis(32767, dz) == pytest.approx(-1.0)             # SDL + → ROS −
    assert joy_axis(-32768, dz) == pytest.approx(1.0)
    assert joy_axis(int(32767 * 0.55), dz) == pytest.approx(-0.5, abs=1e-4)


def test_directions_follow_report_2_2(jcfg):
    """Adelante = +axes[1]; derecha = −axes[0] (reporte §2.2, PENDIENTE P1)."""
    node = FakeNode()
    b = TopicBackend(jcfg, node, clock=FakeClock())
    b.send(to_sdl(0.0, 1.0, 0.0), [0] * 11)
    msg = node.pubs['/joy'].msgs[-1]
    assert msg.axes[1] == pytest.approx(1.0) and msg.axes[0] == pytest.approx(0.0)
    b.send(to_sdl(1.0, 0.0, 0.0), [0] * 11)
    assert node.pubs['/joy'].msgs[-1].axes[0] == pytest.approx(-1.0)
    b.send(to_sdl(0.0, 0.0, 0.0), [0, 0, 0, 1] + [0] * 7)
    assert list(node.pubs['/joy'].msgs[-1].buttons) == [0, 0, 0, 1] + [0] * 7
    assert node.pubs['/joy'].msgs[-1].header.frame_id == 'joy'


def test_topic_backend_autorepeat(jcfg):
    node, clock = FakeNode(), FakeClock()
    b = TopicBackend(jcfg, node, clock=clock)
    b.send(to_sdl(0, 0, 0), [0] * 11)
    n = len(node.pubs['/joy'].msgs)
    for _ in range(100):                     # 1 s en pasos de 10 ms
        clock.t += 0.01
        b.tick()
    assert len(node.pubs['/joy'].msgs) - n == pytest.approx(20, abs=1)   # autorepeat_rate 20 Hz


def test_8bit_quantization_round_trip(jcfg):
    lo, hi = jcfg['axis_raw_min'], jcfg['axis_raw_max']
    assert sdl_to_raw(-1.0, lo, hi) == lo and sdl_to_raw(1.0, lo, hi) == hi
    assert raw_to_sdl_int16(lo, lo, hi) == -32768 and raw_to_sdl_int16(hi, lo, hi) == 32767


def fake_evdev():
    events = []

    class UInput:
        def __init__(self, caps, name, vendor, product, version, bustype):
            self.caps, self.name, self.vendor, self.product, self.bustype = caps, name, vendor, product, bustype
            self.closed = False

        def write(self, etype, code, value):
            events.append((etype, code, value))

        def syn(self):
            events.append('syn')

        def close(self):
            self.closed = True

    import evdev
    mod = types.SimpleNamespace(ecodes=evdev.ecodes, AbsInfo=evdev.AbsInfo, UInput=UInput)
    return mod, events


def test_uinput_device_looks_like_attack3(jcfg):
    import evdev
    mod, events = fake_evdev()
    b = UinputBackend(jcfg, evdev_module=mod)
    caps = b.device.caps
    assert len(caps[evdev.ecodes.EV_KEY]) == 11
    assert caps[evdev.ecodes.EV_KEY][0] == evdev.ecodes.BTN_TRIGGER
    assert [c for c, _ in caps[evdev.ecodes.EV_ABS]] == [evdev.ecodes.ABS_X, evdev.ecodes.ABS_Y,
                                                          evdev.ecodes.ABS_THROTTLE]
    assert b.device.name == jcfg['device_name'] and b.device.vendor == 0x046d and b.device.product == 0xc214
    b.send(to_sdl(0.0, 1.0, 0.0), [1] + [0] * 10)
    assert (evdev.ecodes.EV_ABS, evdev.ecodes.ABS_Y, 0) in events        # adelante = Y mínimo
    assert (evdev.ecodes.EV_KEY, evdev.ecodes.BTN_TRIGGER, 1) in events and events[-1] == 'syn'
    n = len(events)
    b.send(to_sdl(0.0, 1.0, 0.0), [1] + [0] * 10)                         # sin cambios: no escribe
    assert len(events) == n
    b.close()
    assert b.device.closed
