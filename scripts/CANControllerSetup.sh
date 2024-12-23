#!/bin/sh
module=""
device=""
mode="664"
maxports="32"

# Loading necessary CAN modules
echo -e "Load can module and can protocol"
modprobe can
modprobe can-raw
modprobe can-dev

# Installing CAN Controller driver
echo -e "CAN Controller driver"
insmod can-ahc0512.ko

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

ip link set can0 down type can
ip link set can1 down type can

# Setting bitrate, sample-point, and restart-ms for can devices
ip link set can0 type can bitrate 1000000 sample-point 0.6 restart-ms 1
ip link set can1 type can bitrate 1000000 sample-point 0.6 restart-ms 1

# Activating the can devices
echo -e ""
echo -e "-------------------------------------------"
echo -e "ip link set can0,can1 up type can"
echo -e "type ifconfig to see can0, can1"

ip link set can0 up type can
ip link set can1 up type can

# Showing statistic information of can devices
echo -e ""
echo -e "-------------------------------------------"
echo -e "show statistic information of can0 and can1"
echo -e ""
ip -details -statistics link show can0
ip -details -statistics link show can1


