#!/bin/bash
sudo modprobe peak_pciefd
sudo modprobe peak_pci

sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

sudo ip link set can2 down || true
sudo ip link set can2 type can bitrate 1000000 dbitrate 5000000 sjw 10 dsjw 5 sample-point 0.666 dsample-point 0.666 restart-ms 1000 fd on
sudo ip link set can2 up

sudo ip link set can3 down || true
sudo ip link set can3 type can bitrate 1000000 dbitrate 5000000 sjw 10 dsjw 5 sample-point 0.666 dsample-point 0.666 restart-ms 1000 fd on
sudo ip link set can3 up
