#!/bin/sh

# It is a requirement of this script file that it be "reentrant".
# This means that it can be run multiple times without breaking anything.

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

echo "Initialize/Update rosdep"
if [ ! -d /etc/ros/rosdep/sources.list.d ]
   then "Initializing rosdep"
	sudo rosdep init
fi
rosdep update

# Make sure that we some useful packages -- editors, zeroconf, chrony, etc.
echo "Install a bunch of useful packages"
sudo apt-get install -y vim emacs libnss-mdns chrony
sudo apt-get install -y turtlebot-simulator

echo "Add ROS to path"
if ! grep setup.bash ~/.bashrc
   then echo ''                                            >> ~/.bashrc
	echo '# Only modify $PATH if ROS not in path'      >> ~/.bashrc
        echo 'if [ -d "/opt/ros/kinetic/bin" ] ; then '    >> ~/.bashrc
	echo '   case ":$PATH:" in'                        >> ~/.bashrc
	echo '   *:/opt/kinetic/bin:*) ;;'                 >> ~/.bashrc
	echo '   *) source /opt/ros/kinetic/setup.bash ;;' >> ~/.bashrc
	echo '   esac'                                     >> ~/.bashrc
	echo 'fi'                                          >> ~/.bashrc
fi
source ~/.bashrc

# Make sure there is a catkin workspace:
echo "Ensure there is a catkin workspace"
mkdir -p ~/catkin_ws/src
(cd ~/catkin_ws ; catkin_make )

# Make sure we have a copy of ubquity_main repository:
echo "Make sure ubiquity_main repositiory is up-to-date"
if [ ! -d ~/catkin_ws/src/ubquity_main ]
   then	(cd ~/catkin_ws/src ; git clone https://github.com/UbiquityRobotics/ubiquity_main.git )
fi
(cd ~/catkin_ws/src/ubiquity_main ; git pull)
