#!/bin/sh
#  Update Ubiquity Robotics repositories for Bot Side Firmware
#
cd ~/catkin_ws

echo ""
echo "Pull current code for Ubiquity Robotics Loki and Freya firmware"

# All known repositories (we only update ones that exist)
for dir in ubiquity-misc bus_common bus_loki bus_freya bus_raspberry_pi bus_server bus_slave bus_bridge_encoders_sonar
do
  echo "-------------------------------------------"
  if [ -d  src/$dir ]
  then
    echo "cd to dir and do git pull for: src/$dir"
    cd src/$dir
    git pull
    cd ~/catkin_ws
  else
    echo "not pulling non-existing local dir src/$dir"
  fi
done

