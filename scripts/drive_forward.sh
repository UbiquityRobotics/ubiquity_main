#!/bin/bash

#Drives the robot forward and does not do any jittering
# (like teleop) due to latency or keyboard issues.
#   ./drive_forward.sh <speed>
# speed is in m/s
#   Press control-C to stop after starting
echo "Publishing forward at rate of $1 m/s."
echo "Press control-C to stop"
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{x: $1}" "{}"
