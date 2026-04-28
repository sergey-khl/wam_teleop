#!/bin/bash
sudo modprobe peak_usb

# follower
sudo ip link set can1 down || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# leader
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
