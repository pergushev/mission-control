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