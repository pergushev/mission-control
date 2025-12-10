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


def download_mission(master: mavutil.mavlink_connection) -> List[MissionItem]:
    """
    Чтение миссии по протоколу Mission Protocol:
    1) MISSION_REQUEST_LIST
    2) получение MISSION_COUNT
    3) цикл: MISSION_REQUEST_INT -> MISSION_ITEM_INT

    Args:
        master: MAVLink соединение

    Returns:
        Список точек миссии (пустой список при ошибке)
    """
    print("Запрос списка миссии...")

    try:
        master.mav.mission_request_list_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        msg = master.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)
        if msg is None:
            print("Таймаут ожидания MISSION_COUNT")
            return []

        count = msg.count
        print(f"Получено MISSION_COUNT: {count} точек")

        if count == 0:
            print("Миссия пуста")
            return []

        items: List[MissionItem] = []

        for seq in range(count):
            master.mav.mission_request_int_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

            item_msg = master.recv_match(
                type=['MISSION_ITEM_INT', 'MISSION_ITEM'],
                blocking=True,
                timeout=5
            )

            if item_msg is None:
                print(f"Таймаут ожидания точки {seq}")
                continue

            items.append(
                MissionItem(
                    seq=item_msg.seq,
                    frame=item_msg.frame,
                    command=item_msg.command,
                    current=item_msg.current,
                    autocontinue=item_msg.autocontinue,
                    param1=item_msg.param1,
                    param2=item_msg.param2,
                    param3=item_msg.param3,
                    param4=item_msg.param4,
                    x=item_msg.x,
                    y=item_msg.y,
                    z=item_msg.z,
                )
            )
            print(f"Получена точка {seq}: команда={item_msg.command}, z={item_msg.z}м")

        # Отправляем ACK что приняли миссию
        master.mav.mission_ack_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        print(f"✓ Миссия успешно прочитана: {len(items)} точек")
        return items

    except Exception as e:
        print(f"✗ Исключение при чтении миссии: {e}")
        return []


def create_test_mission(master: mavutil.mavlink_connection, target_alt_m: float = 20.0) -> List[MissionItem]:
    """
    Создание расширенной тестовой миссии: взлёт -> 3 точки -> посадка

    Args:
        master: MAVLink соединение для получения текущих координат
        target_alt_m: целевая высота полёта в метрах

    Returns:
        Список точек миссии из 5 элементов
    """
    # Получаем текущую позицию
    print("Ожидание GPS позиции...")
    gps_msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=10)

    if gps_msg is None:
        print("Предупреждение: не получены GPS данные, используем координаты по умолчанию")
        home_lat = int(55.7558 * 1e7)  # Москва
        home_lon = int(37.6173 * 1e7)
    else:
        home_lat = gps_msg.lat
        home_lon = gps_msg.lon
        print(f"Текущая позиция: lat={home_lat / 1e7:.7f}, lon={home_lon / 1e7:.7f}")

    # Создаём миссию
    mission = []

    # Точка 0: Взлёт
    mission.append(MissionItem(
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        current=1,  # Первая точка - текущая
        autocontinue=1,
        param1=0,  # Pitch
        param2=0,
        param3=0,
        param4=0,  # Yaw
        x=home_lat,
        y=home_lon,
        z=target_alt_m  # Высота взлёта
    ))

    # Точка 1: Путевая точка на север (20м по широте ≈ 0.0002°)
    mission.append(MissionItem(
        seq=1,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=5.0,  # Задержка 5 сек
        param2=0,
        param3=0,
        param4=0,
        x=home_lat + int(0.0002 * 1e7),
        y=home_lon,
        z=target_alt_m
    ))

    # Точка 2: Путевая точка на восток
    mission.append(MissionItem(
        seq=2,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=5.0,
        param2=0,
        param3=0,
        param4=0,
        x=home_lat + int(0.0002 * 1e7),
        y=home_lon + int(0.0002 * 1e7),
        z=target_alt_m
    ))

    # Точка 3: Путевая точка возврат на юг
    mission.append(MissionItem(
        seq=3,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=5.0,
        param2=0,
        param3=0,
        param4=0,
        x=home_lat,
        y=home_lon + int(0.0002 * 1e7),
        z=target_alt_m
    ))

    # Точка 4: Посадка в точке старта
    mission.append(MissionItem(
        seq=4,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_LAND,
        current=0,
        autocontinue=1,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=home_lat,
        y=home_lon,
        z=0.0
    ))

    print(f"✓ Создана расширенная миссия из {len(mission)} точек")
    return mission


def verify_mission(original: List[MissionItem], downloaded: List[MissionItem]) -> bool:
    """
    Проверка соответствия загруженной и прочитанной миссий

    Args:
        original: исходная миссия
        downloaded: прочитанная с борта миссия

    Returns:
        True если миссии идентичны
    """
    print("\n=== Верификация миссии ===")

    if len(original) != len(downloaded):
        print(f"✗ Количество точек не совпадает: {len(original)} != {len(downloaded)}")
        return False

    all_match = True

    for i, (orig, down) in enumerate(zip(original, downloaded)):
        match = True
        issues = []

        if orig.seq != down.seq:
            issues.append(f"seq: {orig.seq} != {down.seq}")
            match = False
        if orig.command != down.command:
            issues.append(f"command: {orig.command} != {down.command}")
            match = False
        if orig.frame != down.frame:
            issues.append(f"frame: {orig.frame} != {down.frame}")
            match = False
        if abs(orig.z - down.z) > 0.1:  # Допуск 10см
            issues.append(f"z: {orig.z} != {down.z}")
            match = False
        if abs(orig.x - down.x) > 10:  # Допуск в координатах
            issues.append(f"x: {orig.x} != {down.x}")
            match = False
        if abs(orig.y - down.y) > 10:
            issues.append(f"y: {orig.y} != {down.y}")
            match = False

        if match:
            print(f"✓ Точка {i}: OK (команда={orig.command}, z={orig.z}м)")
        else:
            print(f"✗ Точка {i}: несовпадение - {', '.join(issues)}")
            all_match = False

    if all_match:
        print("\n✓✓✓ Миссия полностью совпадает ✓✓✓")
    else:
        print("\n✗✗✗ Обнаружены расхождения ✗✗✗")

    return all_match