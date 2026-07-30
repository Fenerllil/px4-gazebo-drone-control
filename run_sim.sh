#!/bin/bash

# Остановка скрипта при ошибках
set -e

# Путь к папке твоего проекта
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Определяем путь к PX4 (по умолчанию $HOME/PX4-Autopilot, но пользователь может задать свой)
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

# Проверяем наличие папки PX4
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ Ошибка: Папка PX4 не найдена по пути: $PX4_DIR"
    echo "Задайте путь вручную: export PX4_DIR=/путь/к/PX4-Autopilot"
    exit 1
fi

echo "✅ Найден PX4 по пути: $PX4_DIR"

# 1. Твой export (используем динамический путь к PX4)
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

echo "🚀 Запуск Gazebo..."

# 2. Запуск симуляции с моделью x500 в фоновом режиме (&)
gz sim -r "${PX4_DIR}/Tools/simulation/gz/models/x500/model.sdf" &
GZ_PID=$!

# Даем Gazebo 3 секунды на прогрузку 3D-сцены
sleep 3

echo "🛸 Запуск Python-скрипта управления..."

# 3. Запуск твоего Python-кода (укажи правильное имя своего .py файла)
python3 "${PROJECT_DIR}/drone_control.py"

# Когда Python-скрипт завершит работу, закрываем Gazebo
kill $GZ_PID
