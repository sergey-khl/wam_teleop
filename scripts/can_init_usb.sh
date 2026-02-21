#!/bin/bash
sudo modprobe peak_usb

sudo ip link set can2 type can bitrate 1000000
sudo ip link set up can2

sudo ip link set can3 type can bitrate 1000000
sudo ip link set up can3
