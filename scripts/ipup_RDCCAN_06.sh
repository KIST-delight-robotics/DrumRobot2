#!/bin/sh
module="RDC_CAN_17f3"
device="RDCCAN"
mode="664"
maxports="32"

# Checking available can devices
echo -e ""
echo -e "-------------------------------------------"
echo -e "check if there are a can0,can1 device."
echo -e ""
ip link show

# Showing setting information of a can device
echo -e ""
echo -e "-------------------------------------------"
echo -e "show setting information of a can device"
echo -e ""
ip link set can0 type can help
ip link set can1 type can help

# Setting bitrate, sample-point, and restart-ms for can devices
ip link set can0 type can bitrate 1000000 sample-point 0.850 restart-ms 1
ip link set can1 type can bitrate 1000000 sample-point 0.850 restart-ms 1

# Showing statistic information of can devices
echo -e ""
echo -e "-------------------------------------------"
echo -e "show statistic information of can0 and can1"
echo -e ""
ip -details -statistics link show can0
ip -details -statistics link show can1

# Activating the can devices
echo -e ""
echo -e "-------------------------------------------"
echo -e "ip link set can0 up type can"
echo -e "type ifconfig to see can0"
ip link set can0 up type can
ip link set can1 up type can