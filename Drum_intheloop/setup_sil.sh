#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DXL_DEV="/dev/ttyUSB0"
DXL_LINK="/tmp/ttyUSB0"
DXL_PEER="/tmp/ttyUSB0_sim"
SOCAT_PID=""

cleanup() {
    cleanup_dxl_dev
    rm -f "${DXL_LINK}" "${DXL_PEER}"

    if [[ -n "${SOCAT_PID}" ]] && kill -0 "${SOCAT_PID}" >/dev/null 2>&1; then
        kill "${SOCAT_PID}" >/dev/null 2>&1 || true
        wait "${SOCAT_PID}" >/dev/null 2>&1 || true
    fi
}

stop() {
    cleanup
    exit 0
}

trap cleanup EXIT
trap stop INT TERM

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "[SIL] missing command: $1" >&2
        exit 1
    fi
}

setup_can() {
    sudo modprobe vcan

    for index in 0 1 2 3; do
        iface="vcan${index}"
        if ! ip link show "${iface}" >/dev/null 2>&1; then
            sudo ip link add dev "${iface}" type vcan
        fi
        sudo ip link set "${iface}" up
    done
}

cleanup_dxl_dev() {
    if [[ -L "${DXL_DEV}" ]]; then
        local link_target
        local resolved_target
        link_target="$(readlink "${DXL_DEV}" || true)"
        resolved_target="$(readlink -f "${DXL_DEV}" || true)"

        if [[ "${link_target}" == /dev/pts/* || "${resolved_target}" == /dev/pts/* ]]; then
            sudo rm -f "${DXL_DEV}"
        fi
    fi
}

prepare_dxl_dev() {
    if [[ -e "${DXL_DEV}" && ! -L "${DXL_DEV}" ]]; then
        echo "[SIL] ${DXL_DEV} already exists; refusing to overwrite a real device" >&2
        exit 1
    fi

    if [[ -L "${DXL_DEV}" ]]; then
        local link_target
        local resolved_target
        link_target="$(readlink "${DXL_DEV}" || true)"
        resolved_target="$(readlink -f "${DXL_DEV}" || true)"

        if [[ "${link_target}" != /dev/pts/* && "${resolved_target}" != /dev/pts/* ]]; then
            echo "[SIL] ${DXL_DEV} already points to ${link_target}; refusing to overwrite it" >&2
            exit 1
        fi
    fi

    cleanup_dxl_dev
}

setup_dxl() {
    prepare_dxl_dev
    rm -f "${DXL_LINK}" "${DXL_PEER}"
    socat -d -d \
        "pty,raw,echo=0,mode=666,link=${DXL_LINK}" \
        "pty,raw,echo=0,mode=666,link=${DXL_PEER}" &
    SOCAT_PID="$!"

    for _ in $(seq 1 50); do
        if [[ -e "${DXL_LINK}" && -e "${DXL_PEER}" ]]; then
            sudo ln -sfn "$(readlink -f "${DXL_LINK}")" "${DXL_DEV}"
            return
        fi
        sleep 0.1
    done

    echo "[SIL] failed to create ${DXL_LINK}/${DXL_PEER}" >&2
    exit 1
}

require_cmd ip
require_cmd socat
setup_can
setup_dxl

echo "[SIL] vcan0..3 are up"
echo "[SIL] DXL endpoint for DrumRobot2: ${DXL_DEV}"
echo "[SIL] DXL endpoint for simulator: ${DXL_PEER}"
echo "[SIL] leave this terminal open to keep the DXL PTY alive"
echo "[SIL] run simulator in another terminal:"
echo "      cd ${ROOT_DIR}"
echo "      python3 simul.py --mode gui"

wait "${SOCAT_PID}"
