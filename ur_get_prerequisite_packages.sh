#
# Get many pacakges we generally want or require for Ubiquity Robotics Software
#
# Prerequisites (only SOME of these are loosely checked!
# - There needs to be a ~/catkin_ws workspace created already
# - The /etc/apt/sources.list.d/ros-latest.list file needs to contain
#   deb http://packages.ros.org/ros/ubuntu trusty main  
# - apt-get keys installed sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
# - User has done a 'sudo apt-get update' and a 'sudo apt-get upgrade' 
#
sudo apt-get install -y wpasupplicant                      # for setup of wifi dongle
sudo apt-get install -y minicom                            # useful serial console
sudo apt-get install -y setserial
sudo apt-get install -y mgetty                             # used to change baud rate on a serial port
sudo apt-get install -y wireless-tools                     # many utilities like iwconfig and so on for wireless support
sudo apt-get install --reinstall build-essential git       # this may be on the image, I'm not sure

sudo apt-get install -y ros-indigo-ros-tutorials           # has a lot of things like turtlesim that packages use Pose and so on
sudo apt-get install -y ros-indigo-joystick-drivers
sudo apt-get install -y python-serial                      # needed by ros arduino bridge
sudo apt-get install -y ros-indigo-serial                  # For ubiquity-sonar  50150521
sudo apt-get install -y ros-indigo-navigation              # HUGE package.   Required for fiducial and nav of bot 
sudo apt-get install -y ros-indigo-tf-conversions
sudo apt-get install -y ros-indigo-robot-model             # this has kdl_parser, joint-state-publisher, eigen and collada stuff
sudo apt-get install -y ros-indigo-tf2-geometry-msgs

# now we need to get source that is required for ur packages
# TODO: We should check for the folder and abort if not present
if [ -d  ~/catkin_ws/src ]
  then
  cd ~/catkin_ws/src
  git clone https://github.com/UbiquityRobotics/ubiquity-misc.git   # many things of value for Ubiquity Robotics
  git clone https://github.com/DLu/navigation_layers.git            # For sonar publishing Maybe for Sonar sensors to mimic lidar map?
  git clone https://github.com/ros/robot_state_publisher.git
  git clone https://github.com/bosch-ros-pkg/usb_cam.git            # For using USB camera.  This is optional maybe but good for expansion
else
  echo "ERROR!  Cannot pull source into ~/catkin_ws/src as the directory is not present!"
fi

echo "Have a nice day."
