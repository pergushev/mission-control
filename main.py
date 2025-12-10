"""
Демонстрационный скрипт работы с модулем управления миссиями
Тестирует загрузку, чтение и верификацию миссии
"""

import sys
import time
from pymavlink import mavutil

# Импорт модуля управления миссиями
from mission_control import (
    clear_mission,
    upload_mission,
    download_mission,
    MissionItem,
    create_test_mission,
    verify_mission
)


def connect(connection_string: str = "tcp:127.0.0.1:5760") -> mavutil.mavlink_connection:
    """
    Подключение к SITL/дрон ArduPilot

    Args:
        connection_string: строка подключения
            - SITL: "tcp:127.0.0.1:5760" или "udp:127.0.0.1:14550"
            - Реальное устройство: "/dev/ttyUSB0" или "COM3"
            - Mission Planner: "tcp:127.0.0.1:14550"

    Returns:
        Объект MAVLink соединения или None при ошибке
    """
    print(f"Подключение к {connection_string}...")

    try:
        master = mavutil.mavlink_connection(connection_string)

        # Ждём первое HEARTBEAT от автопилота
        result = master.wait_heartbeat(timeout=10)

        if result is None:
            print("✗ Таймаут ожидания HEARTBEAT")
            return None

        print(f"✓ Подключено к системе {master.target_system}, компонент {master.target_component}")
        return master

    except Exception as e:
        print(f"✗ Ошибка подключения: {e}")
        return None


def wait_for_gps(master: mavutil.mavlink_connection, timeout: float = 30.0) -> bool:
    """
    Ожидание получения валидных GPS координат

    Args:
        master: MAVLink соединение
        timeout: максимальное время ожидания в секундах

    Returns:
        True если GPS данные получены
    """
    print(f"Ожидание GPS данных (таймаут {timeout}с)...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)

        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7

            # Проверяем, что координаты не нулевые
            if abs(lat) > 0.0001 or abs(lon) > 0.0001:
                print(f"✓ GPS получен: lat={lat:.7f}, lon={lon:.7f}")
                return True

        print("  Ожидание GPS...")
        time.sleep(1)

    print("✗ GPS данные не получены за отведённое время")
    return False


def create_simple_mission(master: mavutil.mavlink_connection, target_alt_m: float = 20.0) -> list[MissionItem]:
    """
    Создание простой миссии: взлёт -> 1 точка -> посадка

    Args:
        master: MAVLink соединение
        target_alt_m: целевая высота полёта

    Returns:
        Список точек миссии
    """
    print(f"\nСоздание простой миссии (высота {target_alt_m}м)...")

    # Получаем текущую позицию
    gps_msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=5)

    if gps_msg is None:
        print("Предупреждение: используем координаты по умолчанию")
        home_lat = int(55.7558 * 1e7)  # Москва
        home_lon = int(37.6173 * 1e7)
    else:
        home_lat = gps_msg.lat
        home_lon = gps_msg.lon
        print(f"Домашняя позиция: lat={home_lat / 1e7:.7f}, lon={home_lon / 1e7:.7f}")

    mission = []

    # Точка 0: Взлёт — seq=0, current=1
    mission.append(MissionItem(
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        current=1,  # ← только здесь!
        autocontinue=1,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=home_lat,
        y=home_lon,
        z=target_alt_m
    ))

    # Точка 1: WAYPOINT — seq=1
    mission.append(MissionItem(
        seq=1,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=1,
        param1=5.0,
        param2=0,
        param3=0,
        param4=0,
        x=home_lat + int(0.0001 * 1e7),
        y=home_lon,
        z=target_alt_m
    ))

    # Точка 2: LAND — seq=2
    mission.append(MissionItem(
        seq=2,
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

    print(f"✓ Создана миссия из {len(mission)} точек:")
    for item in mission:
        cmd_name = {
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "WAYPOINT",
            mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
        }.get(item.command, f"CMD_{item.command}")
        print(f"  {item.seq}: {cmd_name} (z={item.z}м)")

    return mission


def main():
    """
    Основная функция демонстрации работы с миссиями
    """
    print("=" * 70)
    print("Демонстрация модуля управления миссиями для БАС")
    print("=" * 70)

    # Выбор типа подключения
    connection_types = {
        "1": ("tcp:127.0.0.1:14550", "Mission Planner TCP"),
        "2": ("udp:127.0.0.1:14550", "Mission Planner UDP"),
    }

    print("\nВыберите тип подключения:")
    for key, (_, desc) in connection_types.items():
        print(f"  {key}. {desc}")

    choice = input("Ваш выбор (Mission Planner TCP): ").strip() or "1"

    if choice not in connection_types:
        print("Неверный выбор, используем Mission Planner TCP по умолчанию")
        choice = "1"

    connection_string, _ = connection_types[choice]

    # Шаг 1: Подключение
    print("\n" + "=" * 70)
    print("ШАГ 1: Подключение к автопилоту")
    print("=" * 70)

    master = connect(connection_string)
    if master is None:
        print("\n✗ Не удалось подключиться")
        print("\nДля SITL запустите:")
        print("  sim_vehicle.py -v ArduCopter --console --map")
        sys.exit(1)

    # Шаг 2: Ожидание GPS
    print("\n" + "=" * 70)
    print("ШАГ 2: Ожидание GPS данных")
    print("=" * 70)

    if not wait_for_gps(master):
        print("\n✗ GPS данные не получены, продолжаем с координатами по умолчанию")

    time.sleep(1)

    # Шаг 3: Очистка старой миссии
    print("\n" + "=" * 70)
    print("ШАГ 3: Очистка текущей миссии")
    print("=" * 70)

    if clear_mission(master):
        print("✓ Миссия очищена")
    else:
        print("⚠ Не удалось очистить миссию (возможно, её и не было)")

    time.sleep(1)

    # Шаг 4: Создание миссии
    print("\n" + "=" * 70)
    print("ШАГ 4: Создание новой миссии")
    print("=" * 70)

    # Выбор типа миссии
    print("\nВыберите тип миссии:")
    print("  1. Простая (взлёт → точка → посадка)")
    print("  2. Расширенная (взлёт → 3 точки → посадка)")

    mission_choice = input("Ваш выбор (Enter для простой): ").strip() or "1"

    if mission_choice == "2":
        mission = create_test_mission(master)
    else:
        mission = create_simple_mission(master, target_alt_m=20.0)

    # Шаг 5: Загрузка миссии
    print("\n" + "=" * 70)
    print("ШАГ 5: Загрузка миссии на борт")
    print("=" * 70)

    if not upload_mission(master, mission):
        print("\n✗ Не удалось загрузить миссию")
        sys.exit(1)

    print("✓ Миссия успешно загружена")
    time.sleep(2)

    # Шаг 6: Чтение миссии
    print("\n" + "=" * 70)
    print("ШАГ 6: Чтение миссии с борта")
    print("=" * 70)

    downloaded = download_mission(master)

    if not downloaded:
        print("\n✗ Не удалось прочитать миссию")
        sys.exit(1)

        print(f"✓ Прочитано {len(downloaded)} точек миссии")

        # Шаг 7: Верификация
        print("\n" + "=" * 70)
        print("ШАГ 7: Верификация миссии")
        print("=" * 70)

        if verify_mission(mission, downloaded):
            print("\n✓✓✓ УСПЕХ! Миссия полностью совпадает ✓✓✓")
        else:
            print("\n⚠⚠⚠ ВНИМАНИЕ! Обнаружены расхождения ⚠⚠⚠")

    # Завершение
    print("\n" + "=" * 70)
    print("РАБОТА ЗАВЕРШЕНА")
    print("=" * 70)
    print("\nМиссия готова к выполнению!")
    print("\nДля запуска миссии вручную:")
    print("  1. В Mission Planner или QGroundControl")
    print("  2. ARM дрон")
    print("  3. Переведите в режим AUTO")
    print("  4. Дрон выполнит загруженную миссию")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✗ Прервано пользователем (Ctrl+C)")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n✗ Неожиданная ошибка: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
