#!/bin/bash

num_cans=5

for ((i = 0;i < num_cans; i++)); do
  sudo ip link add dev can$i type vcan
  sudo ip link set can$i up
  echo "create can$i success"
done

ip link show | grep can