#!/bin/bash

num_cans=5
for ((i = 0; i < num_cans; i++)); do
  sudo ip link delete can$i
  echo "delete can$i"
done

ip link show | grep can