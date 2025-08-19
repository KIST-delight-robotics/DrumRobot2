#!/bin/sh
module=""
device=""
mode="664"
maxports="32"

echo -e "Load can module and can protocol"
modprobe can
modprobe can-raw
modprobe can-dev


echo -e "CAN Controller driver"
insmod can-ahc0512.ko
