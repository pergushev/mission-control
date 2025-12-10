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


def clear_mission(master: mavutil.mavlink_connection) -> bool:
    """
    Очистка миссии командой MISSION_CLEAR_ALL

    Args:
        master: MAVLink соединение

    Returns:
        True если команда отправлена успешно
    """
    try:
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component
        )
        print("Команда MISSION_CLEAR_ALL отправлена")

        # Ожидаем подтверждение
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("Миссия успешно очищена")
            return True
        elif msg:
            print(f"Ошибка очистки миссии: код {msg.type}")
            return False
        else:
            print("Таймаут ожидания MISSION_ACK")
            return False

    except Exception as e:
        print(f"Ошибка при очистке миссии: {e}")
        return False


def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem]) -> bool:
    """
    Загрузка миссии по протоколу Mission Protocol:
    1) MISSION_COUNT
    2) цикл: MISSION_REQUEST_INT -> MISSION_ITEM_INT
    3) ожидание MISSION_ACK

    Args:
        master: MAVLink соединение
        items: список точек миссии

    Returns:
        True если миссия успешно загружена
    """
    count = len(items)
    if count == 0:
        print("Предупреждение: попытка загрузить пустую миссию")
        return False

    print(f"Начинаем загрузку миссии из {count} точек")

    try:
        # Отправляем количество точек
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        print(f"MISSION_COUNT отправлен: {count} точек")

        sent = 0
        max_iterations = count * 3  # Защита от бесконечного цикла
        iterations = 0

        while sent < count and iterations < max_iterations:
            iterations += 1

            msg = master.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                blocking=True,
                timeout=5
            )

            if msg is None:
                print(f"Таймаут ожидания запроса точки {sent}")
                continue

            msg_type = msg.get_type()

            if msg_type in ['MISSION_REQUEST_INT', 'MISSION_REQUEST']:
                seq = msg.seq

                # ВАЖНО: проверяем, что seq в диапазоне [0, count-1]
                if seq < 0 or seq >= count:
                    print(f"Получен запрос с некорректным seq={seq}, ожидаем 0..{count - 1}")
                    continue

                item = items[seq]

                print(f"Отправка точки {seq}/{count - 1}: команда={item.command}, z={item.z}м")

                master.mav.mission_item_int_send(
                    master.target_system,
                    master.target_component,
                    item.seq,
                    item.frame,
                    item.command,
                    item.current,
                    item.autocontinue,
                    item.param1,
                    item.param2,
                    item.param3,
                    item.param4,
                    item.x,
                    item.y,
                    item.z,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                sent += 1

            elif msg_type == 'MISSION_ACK':
                ack_type = msg.type
                if ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"✓ Миссия успешно загружена ({sent} точек)")
                    return True
                else:
                    print(f"✗ Ошибка загрузки миссии: код {ack_type}")
                    return False

        if iterations >= max_iterations:
            print("✗ Превышено максимальное число итераций")
            return False

        print("✗ Не получено подтверждение MISSION_ACK")
        return False

    except Exception as e:
        print(f"✗ Исключение при загрузке миссии: {e}")
        return False