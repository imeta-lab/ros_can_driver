#!/bin/bash

# Shell script to publish ControlCommand messages at 100HZ

TOPIC_NAME="/humanoid/control/control_command"
MSG_TYPE="ros_msgs/ControlCommand"
RATE=100

# torque control
MESSAGE='{"header": {"seq": 1, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""}, 
"control_command": [{"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}, 
                    {"p": 0, "dp": 0, "torque": 1, "kp": 0, "kd": 0}]}'

# position control
# MESSAGE='{"header": {"seq": 1, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""}, 
# "control_command": [{"p": 3.14, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 2, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 3.14, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 2, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 3.14, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 2, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 3.14, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 2, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 3.14, "dp": 0, "torque": 0, "kp": 2, "kd": 0.01}, 
#                     {"p": 2, "dp": 0, "torque": 0, "kp": 2, "kd": 0.01}]}'

# MESSAGE='{"header": {"seq": 1, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""}, 
# "control_command": [{"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 1}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 0.01}, 
#                     {"p": 0, "dp": 0, "torque": 0, "kp": 2, "kd": 0.01}]}'

rostopic pub -r $RATE $TOPIC_NAME $MSG_TYPE "$MESSAGE"

echo "Success publish ControlCommand at $RATE HZ"
