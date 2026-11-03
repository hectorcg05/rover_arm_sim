"""Registro de modelos de driver. Para agregar uno: escribir la clase y registrarla aquí."""

from .epos import EposPPM
from .servo import Servo
from .talon import TalonMotionMagic
from .torque import Torque

DRIVER_MODELS = {
    'talon_motion_magic': TalonMotionMagic,
    'epos_ppm': EposPPM,
    'servo': Servo,
    'torque': Torque,
}


def model_class(name):
    try:
        return DRIVER_MODELS[name]
    except KeyError:
        raise ValueError(f'driver_model desconocido: {name!r}. Opciones: {sorted(DRIVER_MODELS)}') from None
