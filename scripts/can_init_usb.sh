#!/bin/bash
sudo modprobe peak_usb

# follower
sudo ip link set can10 down || true
sudo ip link set can10 type can bitrate 1000000
sudo ip link set can10 up

# leader
sudo ip link set can20 down || true
sudo ip link set can20 type can bitrate 1000000
sudo ip link set can20 up
