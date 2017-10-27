#!/bin/bash

:<<"END_COMMENT"

 This script updates a system updated from the ubiquity image to incoporate changes
 made since the last release

 Invoke with this command:

 wget -O - https://raw.githubusercontent.com/UbiquityRobotics/ubiquity_main/kinetic/update-magni-image.sh | bash

END_COMMENT

# We are always going to want to do this
sudo apt-get update && sudo apt-get upgrade

# Install some packages if not already installed
sudo apt-get install ros-kinetic-teleop-twist-keyboard

# If a current move_basic is not available via debs, then build from source
if [[ ! `apt-cache policy ros-kinetic-move-basic` =~ "Installed: 0.2.2" ]]; then
   echo "Installing move_basic from source"
   if [ -d ~/catkin_ws/src/move_basic ]; then
      echo "Source directory exists"
      cd ~/catkin_ws/src/move_basic && git pull && git checkout kinetic-devel
   else
      cd ~/catkin_ws/src && git clone https://github.com/UbiquityRobotics/move_basic.git
   fi
   cd ~/catkin_ws && catkin_make   
fi
