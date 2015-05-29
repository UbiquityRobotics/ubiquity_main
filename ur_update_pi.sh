#!/bin/sh
#  Update Ubiquity Robotics repositories for Bot Side code
#
cd ~/catkin_ws

echo ""
echo "Pull current code for Ubiquity Robotics Bot-Side code"

# All known repositories (we only update ones that exist)
for dir in fiducials ubiquity-misc ubiquity_motor joystick_input navigation_layers ros_arduino_bridge
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
