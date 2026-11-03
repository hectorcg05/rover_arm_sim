"""Ventana "Attack 3 virtual": imita el Logitech Attack 3 (11 botones, palanca con retorno al centro,
rueda de throttle) con los botones en su posición física y su función escrita (etiquetas del YAML).

Se maneja con teclado y mouse a la vez. Se presenta como joystick USB (uinput) para que el joy_node
real la lea igual que al Attack 3 físico; si no hay permisos de /dev/uinput, publica /joy directo.
Al cerrarse suelta todos los controles. Muestra el modo actual, los avisos de los drivers, las fallas
y el botón "Reiniciar driver".

Uso: ros2 run rover_arm_sim virtual_joystick [--config arm_sim.yaml] [--mode auto|uinput|topic]
"""

import argparse
import json
import signal
import sys
import time

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger

from python_qt_binding.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, Signal
from python_qt_binding.QtGui import QColor, QFont, QKeySequence, QPainter, QPen
from python_qt_binding.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel,
                                         QListWidget, QMainWindow, QPushButton, QSlider, QVBoxLayout,
                                         QWidget)

from . import model_builder as mb
from .joy_backend import open_backend, to_sdl

# Teclas físicas de un teclado de PC (keycodes de X11 = evdev + 8). Sirven para reconocer una tecla
# aunque Shift cambie su símbolo (Shift+2 = "@" o '"' según el idioma del teclado).
X11_KEYCODES = {'1': 10, '2': 11, '3': 12, '4': 13, '5': 14, '6': 15, '7': 16, '8': 17, '9': 18, '0': 19,
                '-': 20, '=': 21, 'Q': 24, 'W': 25, 'E': 26, 'R': 27, 'T': 28, 'A': 38, 'S': 39, 'D': 40,
                'F': 41, 'Space': 65, 'Up': 111, 'Left': 113, 'Right': 114, 'Down': 116}

STYLE = """
QMainWindow, QWidget#root { background: #20242b; }
QLabel { color: #dfe6ee; }
QFrame#panel { background: #2b3038; border: 1px solid #3c434d; border-radius: 10px; }
QPushButton { background: #3a414b; color: #f2f5f8; border: 1px solid #59626e; border-radius: 8px;
              padding: 4px; font-size: 11px; }
QPushButton:pressed, QPushButton[down="true"] { background: #e0a526; color: #111; border-color: #ffd062; }
QPushButton#reset { background: #b3261e; font-weight: bold; font-size: 12px; }
QPushButton#reset:disabled { background: #3a414b; color: #7d8590; }
QListWidget { background: #181b20; color: #f0c674; border: 1px solid #3c434d; font-size: 11px; }
"""


class StickPad(QWidget):
    """Palanca: se arrastra con el mouse y vuelve al centro al soltarla."""

    moved = Signal()

    def __init__(self, labels):
        super().__init__()
        self.setMinimumSize(230, 230)
        self.setFocusPolicy(Qt.NoFocus)
        self.mouse = (0.0, 0.0)
        self.keys = (0.0, 0.0)
        self.dragging = False
        self.labels = labels

    def value(self):
        x = max(-1.0, min(1.0, self.mouse[0] + self.keys[0]))
        y = max(-1.0, min(1.0, self.mouse[1] + self.keys[1]))
        return x, y

    def _radius(self):
        return min(self.width(), self.height()) / 2.0 - 18

    def _set_from(self, pos):
        r = self._radius()
        c = QPointF(self.width() / 2.0, self.height() / 2.0)
        x = (pos.x() - c.x()) / r
        y = -(pos.y() - c.y()) / r
        self.mouse = (max(-1.0, min(1.0, x)), max(-1.0, min(1.0, y)))
        self.moved.emit()
        self.update()

    def mousePressEvent(self, e):
        self.dragging = True
        self._set_from(e.pos())

    def mouseMoveEvent(self, e):
        if self.dragging:
            self._set_from(e.pos())

    def mouseReleaseEvent(self, _e):
        self.dragging = False
        self.mouse = (0.0, 0.0)
        self.moved.emit()
        self.update()

    def release(self):
        self.dragging = False
        self.mouse = (0.0, 0.0)
        self.keys = (0.0, 0.0)
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self._radius()
        c = QPointF(self.width() / 2.0, self.height() / 2.0)
        p.setPen(QPen(QColor('#59626e'), 2))
        p.setBrush(QColor('#181b20'))
        p.drawEllipse(c, r, r)
        p.setPen(QPen(QColor('#3c434d'), 1, Qt.DashLine))
        p.drawLine(QPointF(c.x() - r, c.y()), QPointF(c.x() + r, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - r), QPointF(c.x(), c.y() + r))
        p.setPen(QColor('#9aa4b1'))
        p.setFont(QFont('Sans', 8))
        p.drawText(QRectF(c.x() - 60, c.y() - r - 16, 120, 14), Qt.AlignCenter, 'adelante (W)')
        p.drawText(QRectF(c.x() - 60, c.y() + r + 2, 120, 14), Qt.AlignCenter, 'atrás (S)')
        p.drawText(QRectF(c.x() - r - 4, c.y() + 4, 60, 14), Qt.AlignLeft, 'izq (A)')
        p.drawText(QRectF(c.x() + r - 56, c.y() + 4, 60, 14), Qt.AlignRight, 'der (D)')
        x, y = self.value()
        k = QPointF(c.x() + x * r, c.y() - y * r)
        p.setPen(QPen(QColor('#ffd062'), 2))
        p.drawLine(c, k)
        p.setBrush(QColor('#e0a526') if (x or y) else QColor('#6b7480'))
        p.drawEllipse(k, 16, 16)


class JoyButton(QPushButton):
    def __init__(self, number, label, key):
        super().__init__(f'{number}\n{label}\n[{key}]')
        self.number = number
        self.setFocusPolicy(Qt.NoFocus)
        self.setMinimumSize(92, 54)
        self.mouse_down = False
        self.key_down = False

    def mousePressEvent(self, e):
        self.mouse_down = True
        self._refresh()

    def mouseReleaseEvent(self, e):
        self.mouse_down = False
        self._refresh()

    def set_key(self, down):
        self.key_down = down
        self._refresh()

    def release(self):
        self.mouse_down = self.key_down = False
        self._refresh()

    @property
    def pressed_state(self):
        return self.mouse_down or self.key_down

    def _refresh(self):
        self.setProperty('down', 'true' if self.pressed_state else 'false')
        self.style().unpolish(self)
        self.style().polish(self)


class VirtualJoystick(QMainWindow):
    def __init__(self, cfg, node, mode):
        super().__init__()
        self.cfg = cfg
        self.j = cfg['joystick']
        self.node = node
        self.backend, warning = open_backend(self.j, node, mode)
        self.setWindowTitle('Attack 3 virtual — simulación del brazo')
        self.setStyleSheet(STYLE)
        root = QWidget()
        root.setObjectName('root')
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        self.buttons = {}
        for b in self.j['buttons']:
            self.buttons[b['number']] = JoyButton(b['number'], b['label'], b['key'])
        self.key_map = {}
        self.scan_map = {}
        for b in self.j['buttons']:
            self._bind(b['key'], ('button', b['number']))
        for direction, keys in self.j['stick_keys'].items():
            for k in keys:
                self._bind(k, ('stick', direction))
        self._bind(self.j['throttle_keys']['up'], ('throttle', +1))
        self._bind(self.j['throttle_keys']['down'], ('throttle', -1))
        self.stick_keys_down = set()
        self.pressed_scancodes = {}

        # --- cabeza de la palanca + palanca + rueda
        top = QHBoxLayout()
        head = self._panel('Cabeza de la palanca')
        hg = QGridLayout()
        head.layout().addLayout(hg)
        place = {b['place']: b['number'] for b in self.j['buttons']}
        hg.addWidget(self.buttons[place['head_top']], 0, 1)
        hg.addWidget(self.buttons[place['head_left']], 1, 0)
        hg.addWidget(self.buttons[place['head_right']], 1, 2)
        hg.addWidget(self.buttons[place['head_bottom']], 2, 1)
        trig = self.buttons[place['trigger']]
        trig.setMinimumHeight(64)
        hg.addWidget(trig, 3, 0, 1, 3)
        top.addWidget(head)
        stick_panel = self._panel('Palanca (mouse o W A S D / flechas; Shift = 50 %)')
        self.stick = StickPad(self.j['axes_labels'])
        srow = QHBoxLayout()
        srow.addWidget(self.stick, 1)
        scol = QVBoxLayout()
        for key in ('stick_y', 'stick_x'):
            lab = QLabel(self.j['axes_labels'][key])
            lab.setWordWrap(True)
            lab.setMaximumWidth(150)
            scol.addWidget(lab)
        scol.addStretch(1)
        srow.addLayout(scol)
        stick_panel.layout().addLayout(srow)
        top.addWidget(stick_panel, 2)
        thr = self._panel('Rueda (R / F)')
        self.throttle = QSlider(Qt.Vertical)
        self.throttle.setRange(-100, 100)
        self.throttle.setValue(0)
        self.throttle.setFocusPolicy(Qt.NoFocus)
        self.throttle.setMinimumHeight(180)
        thr.layout().addWidget(self.throttle, 1, Qt.AlignHCenter)
        tl = QLabel(self.j['axes_labels']['throttle'])
        tl.setWordWrap(True)
        tl.setMaximumWidth(120)
        thr.layout().addWidget(tl)
        top.addWidget(thr)
        outer.addLayout(top)

        # --- base: lado izquierdo (6, 7), frente (8, 9), lado derecho (11, 10)
        base = self._panel('Base del joystick')
        bg = QGridLayout()
        base.layout().addLayout(bg)
        bg.addWidget(self.buttons[place['base_left_back']], 0, 0)
        bg.addWidget(self.buttons[place['base_left_front']], 1, 0)
        bg.addWidget(self.buttons[place['base_front_left']], 1, 1)
        bg.addWidget(self.buttons[place['base_front_right']], 1, 2)
        bg.addWidget(self.buttons[place['base_right_back']], 0, 3)
        bg.addWidget(self.buttons[place['base_right_front']], 1, 3)
        note = QLabel('↑ atrás (lado del cable)\nvista desde arriba\n↓ frente (lado de la rueda)')
        note.setAlignment(Qt.AlignCenter)
        note.setStyleSheet('color: #9aa4b1;')
        bg.addWidget(note, 0, 1, 1, 2)
        outer.addWidget(base)

        # --- barra de estado
        status = self._panel('Estado')
        self.mode_label = QLabel('Modo: —')
        self.mode_label.setFont(QFont('Sans', 11, QFont.Bold))
        self.link_label = QLabel(self._link_text(warning))
        self.link_label.setWordWrap(True)
        self.gripper_label = QLabel('Gripper: —')
        row = QHBoxLayout()
        self.fault_label = QLabel('Drivers: sin fallas')
        self.fault_label.setFont(QFont('Sans', 11, QFont.Bold))
        self.reset_button = QPushButton('Reiniciar driver')
        self.reset_button.setObjectName('reset')
        self.reset_button.setFocusPolicy(Qt.NoFocus)
        self.reset_button.setEnabled(False)
        self.reset_button.clicked.connect(self._reset_driver)
        row.addWidget(self.fault_label, 1)
        row.addWidget(self.reset_button)
        self.log_list = QListWidget()
        self.log_list.setMinimumHeight(110)
        self.log_list.setFocusPolicy(Qt.NoFocus)
        for w in (self.mode_label, self.link_label, self.gripper_label):
            status.layout().addWidget(w)
        status.layout().addLayout(row)
        status.layout().addWidget(QLabel('Avisos de los drivers y de arm_manager (lo que en el rover sale en el log):'))
        status.layout().addWidget(self.log_list)
        outer.addWidget(status)

        # --- ROS
        self.mode = '—'
        self.jog_mode = '—'
        self.sim_status = {}
        node.create_subscription(String, '/arm_mode/current', lambda m: setattr(self, 'mode', m.data), 10)
        node.create_subscription(String, '/arm_jog/mode/current', lambda m: setattr(self, 'jog_mode', m.data), 10)
        node.create_subscription(String, cfg['emulator']['status_topic'], self._on_status, 10)
        node.create_subscription(Log, '/rosout', self._on_log, 50)
        self.reset_client = node.create_client(Trigger, cfg['emulator']['reset_service'])
        self.status_nodes = set(self.j['status_nodes'])

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self._send)
        self.send_timer.start(10)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(20)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_status)
        self.ui_timer.start(200)
        self.closing = False

    # ------------------------------------------------------------------ construcción
    def _panel(self, title):
        f = QFrame()
        f.setObjectName('panel')
        v = QVBoxLayout(f)
        t = QLabel(title)
        t.setFont(QFont('Sans', 9, QFont.Bold))
        v.addWidget(t)
        return f

    def _bind(self, key, action):
        seq = QKeySequence(str(key))
        if seq.count():
            self.key_map[seq[0]] = action
        code = X11_KEYCODES.get(str(key).upper() if len(str(key)) == 1 else str(key))
        if code:
            self.scan_map[code] = action

    def _link_text(self, warning):
        if self.backend.name == 'uinput':
            return f'Conexión: joystick USB virtual "{self.j["device_name"]}" (lo lee el joy_node real).'
        text = 'Conexión: publica /joy directo (modo de respaldo).'
        return text + (' ' + warning if warning else '')

    # ------------------------------------------------------------------ teclado
    def _action_for(self, event):
        action = self.key_map.get(event.key())
        if action is None:
            action = self.scan_map.get(event.nativeScanCode())
        return action

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        action = self._action_for(event)
        if action is None:
            return super().keyPressEvent(event)
        self.pressed_scancodes[event.nativeScanCode()] = action
        self._apply_key(action, True)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        action = self.pressed_scancodes.pop(event.nativeScanCode(), None) or self._action_for(event)
        if action is not None:
            self._apply_key(action, False)

    def _apply_key(self, action, down):
        kind, value = action
        if kind == 'button':
            self.buttons[value].set_key(down)
        elif kind == 'stick':
            (self.stick_keys_down.add if down else self.stick_keys_down.discard)(value)
        elif kind == 'throttle' and down:
            step = int(round(100 * float(self.j['throttle_key_step'])))
            self.throttle.setValue(self.throttle.value() + value * step)
        self._update_stick_keys()

    def _update_stick_keys(self):
        mods = QApplication.keyboardModifiers()
        gain = float(self.j['keyboard_half_deflection']) if mods & Qt.ShiftModifier else 1.0
        d = self.stick_keys_down
        x = (('right' in d) - ('left' in d)) * gain
        y = (('forward' in d) - ('back' in d)) * gain
        self.stick.keys = (x, y)
        self.stick.update()

    def focusOutEvent(self, event):
        self.release_all()
        super().focusOutEvent(event)

    def changeEvent(self, event):
        # Si la ventana pierde el foco con una tecla apretada, Qt no avisa la tecla soltada: se suelta
        # todo para que el brazo no siga moviéndose (el teleop real no tiene tiempo límite, hallazgo 1).
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            self.release_all()
        super().changeEvent(event)

    # ------------------------------------------------------------------ envío
    def state(self):
        self._update_stick_keys()
        x, y = self.stick.value()
        thr = self.throttle.value() / 100.0
        buttons = [1 if self.buttons[n].pressed_state else 0 for n in sorted(self.buttons)]
        return to_sdl(x, y, thr), buttons

    def _send(self):
        if self.closing:
            return
        axes, buttons = self.state()
        self.backend.send(axes, buttons)
        self.backend.tick()

    def release_all(self):
        """Palanca al centro y botones sueltos (la rueda se queda donde está, como la física)."""
        self.stick.release()
        self.stick_keys_down.clear()
        self.pressed_scancodes.clear()
        for b in self.buttons.values():
            b.release()

    # ------------------------------------------------------------------ ROS
    def _spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _on_status(self, msg):
        try:
            self.sim_status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_log(self, msg):
        if msg.name in self.status_nodes and msg.level >= int(self.j['status_min_level']):
            level = {30: 'AVISO', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, str(msg.level))
            stamp = time.strftime('%H:%M:%S')
            self.log_list.insertItem(0, f'{stamp} {level} [{msg.name}] {msg.msg}')
            while self.log_list.count() > 50:
                self.log_list.takeItem(self.log_list.count() - 1)

    def _refresh_status(self):
        self.mode_label.setText(f'Modo: {self.mode} · submodo: {self.jog_mode}')
        s = self.sim_status
        faults = s.get('faults', [])
        stalled = s.get('stalled', [])
        if faults:
            self.fault_label.setText('⚠ EN FALLA: ' + ', '.join(faults) +
                                     ' (no responde; usa "Reiniciar driver")')
            self.fault_label.setStyleSheet('color: #ff6b5e;')
        elif stalled:
            self.fault_label.setText('Atascado (el motor sigue empujando): ' + ', '.join(stalled))
            self.fault_label.setStyleSheet('color: #f0c674;')
        else:
            self.fault_label.setText('Drivers: sin fallas')
            self.fault_label.setStyleSheet('color: #8fd18f;')
        self.reset_button.setEnabled(bool(faults))
        g = s.get('gripper')
        if g:
            text = f'Gripper: apertura {100 * g.get("opening_m", 0):.1f} cm'
            if g.get('holding'):
                text += f' · sosteniendo «{g["holding"]}»'
            if g.get('at_end_stop'):
                text += ' · en el tope'
            self.gripper_label.setText(text)

    def _reset_driver(self):
        if not self.reset_client.service_is_ready():
            self.log_list.insertItem(0, 'El servicio de reinicio no está disponible todavía.')
            return
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.log_list.insertItem(0, 'Reiniciar driver: ' + f.result().message))

    # ------------------------------------------------------------------ cierre
    def shutdown(self):
        """Suelta todo, espera a que el joy_node lo lea y recién entonces destruye el dispositivo."""
        if self.closing:
            return
        self.release_all()
        self._send()
        end = time.monotonic() + float(self.j['release_wait_s'])
        while time.monotonic() < end:
            self.backend.send(*self.state())
            self.backend.tick()
            QApplication.processEvents()
            time.sleep(0.02)
        self.closing = True
        self.backend.close()

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


def main(argv=None):
    argv = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(description='Ventana "Attack 3 virtual"')
    ap.add_argument('--config', default=None)
    ap.add_argument('--mode', default=None, help='auto | uinput | topic (por defecto, el del YAML)')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(argv)[1:])
    cfg = mb.load_config(args.config)
    rclpy.init(args=argv)
    node = rclpy.create_node('attack3_virtual')
    app = QApplication(sys.argv[:1])
    app.setFont(QFont('Sans', 9))
    win = VirtualJoystick(cfg, node, args.mode or cfg['joystick']['mode'])
    node.get_logger().info(f'Ventana del joystick lista (conexión: {win.backend.name}).')
    win.resize(860, 760)
    win.show()

    def stop(*_):
        win.shutdown()
        app.quit()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    app.aboutToQuit.connect(win.shutdown)
    code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
