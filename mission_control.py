"""
Модуль управления миссиями для БАС
Реализует протокол MAVLink Mission Protocol для работы с полётными заданиями
"""

from dataclasses import dataclass
from typing import List, Optional

from pymavlink import mavutil


@dataclass
class MissionItem:
    """
    Один пункт миссии в формате MISSION_ITEM_INT

    Attributes:
        seq: порядковый номер пункта (0..N-1)
        frame: система координат (например, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        command: команда MAV_CMD_NAV_... или другая
        current: 1 для текущей точки (обычно только у первой)
        autocontinue: 1, если автоматически переходить к следующей точке
        param1..4: параметры команды (зависят от типа команды)
        x, y: широта/долгота * 1e7
        z: высота в метрах
    """
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int  # lat * 1e7
    y: int  # lon * 1e7
    z: float  # alt (м)