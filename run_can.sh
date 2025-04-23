#!/bin/bash

# can0 상태 확인
state=$(ip link show can0 2>/dev/null | grep -o 'state [A-Z]*' | awk '{print $2}')

if [ "$state" != "UP" ]; then
  echo "[INFO] can0 is not UP. Enabling with bitrate 500000..."
  sudo ip link set can0 up type can bitrate 500000
else
  echo "[INFO] can0 is already UP."
fi

# ROS 2 환경 설정
echo "[INFO] Sourcing ROS 2 workspace..."
source install/setup.bash

# ROS 2 노드 실행
echo "[INFO] Launching can_receiver_node..."
ros2 run ioniq5_can_pub can_receiver_node

