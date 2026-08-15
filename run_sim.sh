#!/bin/bash

# Путь к текущей папке проекта
PROJECT_DIR=$(pwd)

echo "Запускаем симулятор Gazebo..."
export GZ_SIM_RESOURCE_PATH=${PROJECT_DIR}/models
gz sim -r ${PROJECT_DIR}/worlds/custom_world.sdf &
GZ_PID=$!

sleep 3 # Ждем пару секунд, пока прогрузится мир

echo "Запускаем мост MAVLink (телеметрия)..."
# Запускаем мост в фоновом режиме
python3 mavlink_bridge.py &
BRIDGE_PID=$!

echo "Запускаем ПИД-регулятор (управление)..."
# Этот скрипт запускаем на переднем плане, чтобы видеть логи и графики
python3 pid_current_z.py 

# Когда control.py закончит работу (выдаст графики и закроется), убиваем фоновые процессы
echo "Остановка симуляции и телеметрии..."
kill $BRIDGE_PID
kill $GZ_PID
