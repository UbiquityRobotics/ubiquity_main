# Ubiquity_Main


This repository serves as a starting place where documents are kept
along with some of the shell scripts used by the Ubiquity team.

This repository also constains files used by the Ubiquity team
that have no other obvious place to be put.  For now,
this repository contains all of the documentation and shell
files used to the Ubiqutiy/Ubuntu/ROS Kernel image.

## List of Key Documentation In This Repository

The names of the files give a clue to the contents and each file is in itself a README 
so feel free to browse and inspect these documents to learn more about
the projects and processes of Ubiquity Robotics 

* [Doc_Installing_ROS_on_Ubuntu_on_VirtualBox.md](Doc_Installing_ROS_on_Ubuntu_on_VirtualBox.md)

* [Doc_Downloading_and_Installing_the_Ubiquity_Ubuntu_ROS_Kernel_Image.md](Doc_Downloading_and_Installing_the_Ubiquity_Ubuntu_ROS_Kernel_Image.md)

* [Doc_Hardware_Architecture.md](Doc_Hardware_Architecture.md)

* [Doc_Network_Architecture.md](Doc_Network_Architecture.md)

* [Doc_Robot_Launch_Repositories.md](Doc_Robot_Launch_Repositories.md)

## Modifications to This README

If you read this documentation and it does not work for
you, please do the following:

1. Make a clone of this repository on you local desktop/laptop.

        cd /tmp           # Or someplace else of your choosing
        git clone https://github.com/UbiquityRobotics/ubiquity-misc.git
        cd ubiqutiy-misc

2. Edit this file (`README.md`) using your favorite editor.
   Insert you comments and/or questions directly into the file
   using the following format:

        > * ....
        > ....
        > .... -- {your_name} *

   By way of explanation, lines start with `>` are indented
   by the markdown processor.  The `*` causes the text to be
   italicized.

3. Stuff the question(s) back up to the repository:

        git add README.md
        git commit -m "Added some questions/comments."
        git push

## Manually Building a .deb from Scratch

The following commands show how to manually build a `.deb` from scratch:

        ################
        # Read:
        #  http://answers.ros.org/question/173804/generate-deb-from-ros-package/
        # Browse:
        #  https://wiki.debian.org/BuildingTutorial
        #  http://answers.ros.org/question/11315/creating-private-deb-packages-for-distribution/
        ################
        # Install required software:
        sudo apt-get install -y build-essential fakeroot devscripts equivs
        sudo apt-get install -y python-bloom gdebi-core
        ################
        # Start in the correct directory:
        cd .../catkin_ws/src/YOUR_PACKAGE # YOUR_PACKAGE==name of your package
        ################
        # Make sure that there is no `debian` directory
        rm -rf debian
        ################################################################
        # Run bloom to generate `debian` directory.  The `rosdebian`
        # argument will install files into /ros/indigo/...  In theory,
        # replacing `rosdebian` with `debian` will install into `/usr`,
        # but it does not seem to work that well:
        bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
        ################
        # Deal with dependencies:
        sudo mk-build-deps -i -r
        ################
        # Now build the package:
        fakeroot debian/rules clean
        fakeroot debian/rules binary
        ################
        # The package should be in `../ros-indigo-*.deb`.  It can be installed:
        sudo gdebi ../ros-indigo-YOUR-PACKAGE # Where YOUR-PACKAGE has '-', not '_'
	################
        # To totally remove, purge, and expunge:
        sudo apt-get purge ../ros-indigo-YOUR-PACKAGE


The Reprepro system is apparently used to deploy an apt-get repository
using reprepro:

* [https://wiki.debian.org/SettingUpSignedAptRepositoryWithReprepro](https://wiki.debian.org/SettingUpSignedAptRepositoryWithReprepro)

* [https://www.digitalocean.com/community/tutorials/how-to-use-reprepro-for-a-secure-package-repository-on-ubuntu-14-04](https://www.digitalocean.com/community/tutorials/how-to-use-reprepro-for-a-secure-package-repository-on-ubuntu-14-04)


## Fiducials Software

1. Make sure we have a `Tag_Heights.xml` file:

        mkdir -p ~/.ros/fiducials
        cp ~/catkin_ws/src/fiducials/fiducial_lib/Tag_Heights.xml ~/.ros/fiducials

2. Start the `map.txt` file:

        mkdir -p ~/.ros/slam
        echo '41 0.0 0.0 0.0 180.0 -0.0 0.0 0.0 1' > ~/.ros/slam/map.txt

   where `41` is replaced with the fiducial to use as origin.


## `gscam` Notes

> It looks like gscam is already being built and installed
> into the kernel image.
> -Wayne

The following commands build gscam:

        #sudo modprobe bcm2835-v4l2 gst_v4l2src_is_broken=1
        sudo apt-get build-essential g++
        cd catkin_ws/src
        catkin_make
        source devel/setup.bash
        cd src
        git clone https://github.com/ros-drivers/gscam.git
        rosdep update
        rosdep install --from-paths . -i -y 
        sudo apt-get install -y ros-indigo-image-common
        sudo apt-get install -y libgstreamer-plugins-base0.10-dev
        sudo apt-get isntall -y gstreamer0.10-plugins-good
        catkin_make
        roslaunch fiduicals_lib gscam.launch
        rosrun image_view image_view image:=/camera/image_raw

## Older Stuff

The stuff below is getting old and probably needs to be deleted...

## `gscam` Notes

> It looks like gscam is already being built and installed
> into the kernel image.
> -Wayne

The following commands build gscam:

        sudo modprobe bcm2835-v4l2 gst_v4l2src_is_broken=1
        sudo apt-get build-essential g++
        cd ~
        mkdir -p catkin_ws/src
        cd catkin_ws
        catkin_make
        source devel/setup.bash
        cd src
        git clone https://github.com/ros-drivers/gscam.git
        rosdep update
        sudo apt-get install ros-indigo-image-common
        sudo apt-get install gstreamer0.10-plugins-good
        catkin_make
        roslaunch fiduicals_lib gscam.launch
        rosrun image_view image_view image:=/camera/image_raw

## Software Tasks

### General

* identify robot operating modes (development, deployment,
  trade-show, setup, etc.)

### Setup

* How does the user set things up?

### ROS Stack

* Find and fix bug with ARM7 `robot-state-publisher`

* Get `range-sensor-layer` to work.

* Bring fiducials, sonars, and encoders with navigation stack.

* Merge in game controllers.

* Get "Go To" application working.

* Bring up arm.

### Platforms

* Loki

  * Switch over to micro metal gearmotors and hall effect sensors.

  * Bring up real time clock.

  * Tune PID loops

* Freya

  * Refactor Loki Sonar code into Sonar10 modules.

  * Get ROS Arduino Bridge working with Sonar 10 modules

  * Tune PID loops

### Bus Software

* Write `.xml` regsiter definition files for `bus_sonar10`,
  `bus_bridge_encoders_sonar`, and `bus_loki`.

* Modify `bus_slave/bus_code_generator.py` to generate C++
  code fragments that can be specialized.

* Replace ROS Arduino Bridge.

  * Get bus_server_server.py to talk to other modules.

  * Get parameters to work.

  * Create odometry topic publisher.

  * Create sonar range topic publisher.

  * Create cmd_vel topic subscriber.

* Get dynamic reconfiguration working.

* Rewrite bus_server_server.py in C++.

* Rewrite topic publisher and subscribers in C++.

* Get bus reset to work.

* Get discovery to work.

* Get over the wire firmware upgrade to work.

### Bus Hardware

* Bring up additional modules -- bus_battery, bus_dynabus, bus_grove12,
  bus_servo32, bus_servo8, bus_shield, etc.

### Build

* Grind out weekly kernel images

* Make all repositories have package manifests.

## Scripts

### `mgit`

The `mgit` script a shell script that executes the same git command
across all all `src` sub-directories in a `catkin_ws` directory.
It can be executed executed in the `catkin_ws` directory or any
of the sub-directories under `catkin_ws`.

Examples:

List the status of each git repository the catkin workspace:

        mgit status

Pull the latest updates from all of the remote repositories:

        mgit pull

Perform a commit for all repositories that have had files
where a `git add` has been performed:

        mgit commit -m "..."

## Random Notes

        cd ~/catkin_ws/src
        # Install fiducials:
        git clone https://github.com/UbiquityRobotics/fiducials.git
        sudo apt-get install ros-indigo-tf ros-indigo-tf2-geometry-msgs

        # Install gscam
        git clone https://github.com/ros-drivers/gscam
        sudo apt-get install -y ros-indigo-image-common
        sudo apt-get install -y gstreamer0.10-plugins-good
        sudo apt-get install -y libgstreamer0.10-dev
        sudo apt-get install -y libgstreamer-plugins-base0.10-dev

        # Install joystick drivers and ROS Arduino Bridge:
        git clone https://github.com/UbiquityRobotics/joystick_drivers.git
        git clone https://github.com/UbiquityRobotics/ros_arduino_bridge

## Joystick Notes:

There are two broad categories of hand game controllers -- PS3 and Xbox.
Some of these controllers have wire connections, but most of them
are now wireless.  The PS3 wireless controllers are layered on top
of the BlueTooth HID communication layer.  The XBox wireless is
a proprietary communication protocol.

In order to have one of these game controllers talk to ROS, it
is necessary to have a dongle that plugs into a USB port.  There
are game dongles that are compatible with both the XBox and PS3
controllers.  In addition, for the PS3, (I think that) there are
more generic Bluetooh HID USB dongles that can be used.

These game controllers basically have three classes of inputs
1) buttons, 2) joysticks, and 3) acceleratometers.

The buttons are numbered from 0-16.  An image of the button assignms
can be found near the end of the URL below:

        http://wiki.ros.org/ps3joy

Notice that the joysticks are also buttons that can be clicked
by pushing down on the joystick (i.e. -Z axis direction).

The last image shows the axis numbers for the joysticks and the
accelerometers.  The axes are numbered 0-19 and are assigned
as follows:

* Axis 0: left joystick side to side.
* Axis 1: left joystick forward and back.
* Axis 2: right joystick side to side.
* Axis 3: right joystick forward and back.
* Axis 16: accellerometer side to side.
* Axis 17: accellerometer forward and back
* axis 18: accellerometer up and down.
* Axis 19: accellerometer roll.


In order to support this stuff it is necessary to install the
following packages:

        sudo apt-get install joystick ros-indigo-joy ros-indigo-joystick-drivers
        sudo apt-get install ros-indigo-teleop-twist-joy
        sudo apt-get install ros-indigo-yocs-velocity-smoother
        sudo apt-get install ros-indigo-turtlebot-teleop

Getting a USB Dongle to work with a PS3 sixshock 3 game controller
is a bunch of work:

Visit http://www.pabr.org/sixlinux/sixlinux.en.html :
        
        sudo apt-get install libjack-dev
        # Add "#include <unistd.h>" to file QtSixA-1.5.1/sixad/shared.h

Visit:

        https://github.com/rdepena/node-dualshock-controller/wiki/Pairing-The-Dual-shock-3-controller-in-Linux-(Ubuntu-Debian)

## Random  Notes:

        # Mike Ferguson says the command below solves all the rosdop problems:
        rosdep install --from-paths src --ignore-src --rosdistro indigo -y

        #rosdep install --from-paths . -i -y 

        rosdep install --from-paths src -i -y 

        sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

        sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -
        sudo apt-get update && sudo apt-get install build-essential ros-indigo-ros-base ros-indigo-common-msgs ros-indigo-tf ros-indigo-tf2 ros-indigo-tf2-ros
        sudo rosdep init && rosdep update

> * WiFi material has been removed and placed in a separate document *

### Tony's notes on bringing up `stormbringer`

The following issues were encountered on bringing up Tony's `stormbringer`:

* hosing /etc/host - adding 12 megs of vacuous stuff

* bus_server was not initially installed

* bus_common was not installed

* bus_loki was not installed

* bus_slave was not installed

* Arduino was not installed

* Arudino-Makefile was not installed

* Apt package gcc-avr was not installed

* Apt package avr-libc was not installed

* sudo apt-get install libjack-dev

* sudo apt-get install joystick ros-indigo-joy ros-indigo-joystick-drivers



