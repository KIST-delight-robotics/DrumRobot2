#!/bin/bash
# vcan0 인터페이스를 한 번 설정하는 스크립트.
# 부팅 후 SIL close-loop를 시작하기 전에 한 번 실행한다.
# 사용법: sudo bash setup_vcan.sh

set -e

echo "[vcan] loading kernel module..."
modprobe vcan

echo "[vcan] creating vcan0 interface..."
if ip link show vcan0 > /dev/null 2>&1; then
    echo "[vcan] vcan0 already exists, skipping add."
else
    ip link add dev vcan0 type vcan
fi

echo "[vcan] bringing up vcan0..."
ip link set up vcan0

echo "[vcan] done."
ip link show vcan0
