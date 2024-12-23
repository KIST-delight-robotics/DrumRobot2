#!/bin/sh
module=""
device=""
mode="664"
maxports="32"

echo -e "Unload RDC_CAN_platform driver"
rmmod can-ahc0512
