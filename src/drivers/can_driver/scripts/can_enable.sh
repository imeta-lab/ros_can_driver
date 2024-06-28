#!/bin/sh

sudo ifconfig can00_09 down
sudo ip link set can00_09 type can bitrate 1000000
sudo ifconfig can00_09 up

sudo ifconfig can01_08 down
sudo ip link set can01_08 type can bitrate 1000000
sudo ifconfig can01_08 up

sudo ifconfig can02_07 down
sudo ip link set can02_07 type can bitrate 1000000
sudo ifconfig can02_07 up

sudo ifconfig can03_06 down
sudo ip link set can03_06 type can bitrate 1000000
sudo ifconfig can03_06 up

sudo ifconfig can04_05 down
sudo ip link set can04_05 type can bitrate 1000000
sudo ifconfig can04_05 up