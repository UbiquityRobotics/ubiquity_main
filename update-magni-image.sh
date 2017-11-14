#!/bin/bash

:<<"END_COMMENT"

 This script updates a system updated from the ubiquity image to incoporate changes
 made since the last release

 Invoke with this command:

 wget -O - https://ubiquityrobotics.com/patch-script | bash

END_COMMENT

function get_source {
  pkg=$1
  echo "Installing ${pkg} from source"
  if [ -d ~/catkin_ws/src/${pkg} ]; then
     echo "Source directory exists"
     cd ~/catkin_ws/src/${pkg} && git pull
  else
    cd ~/catkin_ws/src && git clone https://github.com/UbiquityRobotics/${pkg}.git
  fi
}

# We are always going to want to do this
echo "Upgrading installed packages"
sudo apt-get update && sudo apt-get upgrade -y

get_source 'ubiquity_launches'

echo "Done"
