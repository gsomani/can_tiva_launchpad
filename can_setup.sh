#!/bin/bash
modprobe can
modprobe can_raw
modprobe can_bcm
ip link set can0 type can bitrate 500000
ip link set can0 up
ip link set can1 type can bitrate 500000
ip link set can1 up