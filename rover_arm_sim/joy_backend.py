"""Cómo llega el "Attack 3 virtual" al sistema.

* UinputBackend (modo principal, reporte D3): crea un joystick USB virtual con los mismos ejes y
  botones que el Attack 3. El joy_node REAL (lanzado por drive_arm_teleop_launch.py) lo lee igual
  que al físico, así que se reutilizan también su zona muerta, signos y autorepetición.
* TopicBackend (respaldo, sin permisos de /dev/uinput): publica /joy directo copiando lo que hace
  joy_node 3.3.0 con los parámetros del launch real (zona muerta 0.1, autorepetición 20 Hz).

Convención de entrada (la de la ventana): x = palanca a la derecha (+1), y = palanca adelante (+1),
throttle = rueda hacia "+" (+1). Los botones van por índice (número impreso − 1).
"""

import time

from sensor_msgs.msg import Joy


def to_sdl(x, y, throttle):
    """Valores como los entrega el hardware: SDL/evdev usa +X derecha, +Y atrás (reporte §2.2)."""
    return [x, -y, -throttle]


def sdl_to_raw(value, raw_min, raw_max):
    """Valor normalizado [-1, 1] → entero del eje del dispositivo (Attack 3: 8 bits)."""
    value = max(-1.0, min(1.0, value))
    return int(round(raw_min + (value + 1.0) / 2.0 * (raw_max - raw_min)))


def raw_to_sdl_int16(raw, raw_min, raw_max):
    """Escalado de SDL2 (joystick de Linux): rango del eje → [-32768, 32767]."""
    return int(round((raw - raw_min) * 65535.0 / (raw_max - raw_min) - 32768.0))


def joy_axis(val_int16, deadzone):
    """Joy::convertRawAxisValueToROS de joy 3.3.0 (ros-humble-joy): zona muerta escalada y signo
    invertido (adelante e izquierda = positivo)."""
    v = float(val_int16)
    if v == -32768.0:
        v = -32767.0
    unscaled = 32767.0 * deadzone
    scale = -1.0 / (1.0 - deadzone) / 32767.0
    if v > unscaled:
        v -= unscaled
    elif v < -unscaled:
        v += unscaled
    else:
        v = 0.0
    return v * scale


class Backend:
    name = ''

    def send(self, axes_sdl, buttons):
        raise NotImplementedError

    def tick(self):
        """Se llama seguido (autorepetición del respaldo)."""

    def close(self):
        pass


class UinputBackend(Backend):
    name = 'uinput'

    def __init__(self, cfg, evdev_module=None):
        if evdev_module is None:
            import evdev as evdev_module
        ev = evdev_module.ecodes
        self.ev = ev
        j = cfg
        self.raw_min, self.raw_max = int(j['axis_raw_min']), int(j['axis_raw_max'])
        center = (self.raw_min + self.raw_max) // 2
        self.axis_codes = [getattr(ev, c) for c in j['axis_codes']]
        self.button_codes = [getattr(ev, c) for c in j['button_codes']]
        absinfo = evdev_module.AbsInfo(value=center, min=self.raw_min, max=self.raw_max, fuzz=0, flat=0,
                                       resolution=0)
        caps = {ev.EV_KEY: self.button_codes, ev.EV_ABS: [(c, absinfo) for c in self.axis_codes]}
        self.device = evdev_module.UInput(caps, name=j['device_name'], vendor=int(j['vendor_id']),
                                          product=int(j['product_id']), version=0x0110, bustype=ev.BUS_USB)
        self.last_axes = None
        self.last_buttons = None

    @property
    def device_path(self):
        return getattr(self.device, 'device', None) and self.device.device.path

    def send(self, axes_sdl, buttons):
        raw = [sdl_to_raw(v, self.raw_min, self.raw_max) for v in axes_sdl]
        changed = False
        for i, (code, value) in enumerate(zip(self.axis_codes, raw)):
            if self.last_axes is None or self.last_axes[i] != value:
                self.device.write(self.ev.EV_ABS, code, value)
                changed = True
        for i, (code, value) in enumerate(zip(self.button_codes, buttons)):
            if self.last_buttons is None or self.last_buttons[i] != value:
                self.device.write(self.ev.EV_KEY, code, int(value))
                changed = True
        if changed:
            self.device.syn()
        self.last_axes, self.last_buttons = raw, list(buttons)

    def close(self):
        self.device.close()


class TopicBackend(Backend):
    name = 'topic'

    def __init__(self, cfg, node, clock=time.monotonic):
        self.cfg = cfg
        self.node = node
        self.clock = clock
        self.pub = node.create_publisher(Joy, cfg['joy_topic'], 10)
        self.deadzone = float(cfg['joy_deadzone'])
        self.period = 1.0 / float(cfg['joy_autorepeat_rate'])
        self.raw_min, self.raw_max = int(cfg['axis_raw_min']), int(cfg['axis_raw_max'])
        self.axes = [0.0] * len(cfg['axis_codes'])
        self.buttons = [0] * len(cfg['button_codes'])
        self.last_pub = 0.0

    def convert(self, axes_sdl):
        out = []
        for v in axes_sdl:
            raw = sdl_to_raw(v, self.raw_min, self.raw_max)
            out.append(joy_axis(raw_to_sdl_int16(raw, self.raw_min, self.raw_max), self.deadzone))
        return out

    def send(self, axes_sdl, buttons):
        axes = self.convert(axes_sdl)
        if axes != self.axes or list(buttons) != self.buttons:
            self.axes, self.buttons = axes, [int(b) for b in buttons]
            self._publish()

    def tick(self):
        if self.clock() - self.last_pub >= self.period:
            self._publish()

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['joy_frame_id']
        msg.axes = [float(a) for a in self.axes]
        msg.buttons = list(self.buttons)
        self.pub.publish(msg)
        self.last_pub = self.clock()


def open_backend(cfg, node, mode):
    """Devuelve (backend, aviso). mode: auto | uinput | topic."""
    if mode in ('auto', 'uinput'):
        try:
            return UinputBackend(cfg), ''
        except Exception as exc:   # sin permisos en /dev/uinput, o sin el módulo
            if mode == 'uinput':
                raise
            return TopicBackend(cfg, node), (
                f'Sin acceso a /dev/uinput ({exc.__class__.__name__}): la ventana publica /joy directo. '
                'Para usarla como joystick USB, ver "Instalar" en guia_operadores_sim.md.')
    if mode == 'topic':
        return TopicBackend(cfg, node), ''
    raise ValueError(f'modo de joystick desconocido: {mode!r}')
