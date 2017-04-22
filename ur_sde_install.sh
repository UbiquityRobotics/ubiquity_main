#!/bin/sh

# Make sure that ROS kinetic is 
if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]
   then echo "Add ROS kinetic to repository list"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
fi

echo "Update repositories"
sudo apt-get update

echo "Download full ROS kinetic desktop"
sudo apt-get install -y ros-kinetic-desktop-full

echo "Upgrade to lastest ubunutu patches"
sudo apt-get upgrade -y

