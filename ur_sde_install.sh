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

echo "Initialize/Update rosdep"
if [ ! -d /etc/ros/rosdep/sources.list.d ]
   then "Initializing rosdep"
	sudo rosdep init
fi
rosdep update

echo "Add ROS to path"
if ! grep setup.bash ~/.bashrc
   then echo '# Only modify $PATH if ROS not in path'      >> ~/.bashrc
        echo 'if [ -d "/opt/ros/kinetic/bin" ] ; then '    >> ~./bashrc
	echo '   case ":$PATH:" in'                        >> ~./bashrc
	echo '   *:/opt/kinetic/bin:*) ;;'                 >> ~/.bashrc
	echo '   *) source /opt/ros/kinetic/setup.bash ;;' >> ~/.bashrc
	echo '   esac'                                     >> ~/.bashrc
	echo 'fi'                                          >> ~/.bashrc
fi
source ~/.bashrc
