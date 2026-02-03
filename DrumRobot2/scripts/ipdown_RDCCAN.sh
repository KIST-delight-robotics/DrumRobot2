#!/bin/sh
module="RDC_CAN_17f3"
device="RDCCAN"
mode="664"
maxports="32"

ip link set can0 down type can
ip link set can1 down type can