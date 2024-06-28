#!/bin/bash

thread_num=$(($(nproc) - 1))

# ros msgs
catkin_make install --pkg ros_msgs -j${thread_num}

# drivers
catkin_make install --pkg can_driver -j${thread_num}     # can