# Ubiquity ROS Video

## Video 1: Introduction to Ubiquity

* Intro of Wayne as HBRC president and member of UR team.
* Show UR splash page.
* Intro of UR.
  * We believe that robots will be ubiquitous.
  * In 1977, Star Wars was filled with "droids".
  * We call them "Bots".
  * Our slogan is "Bots Everywhere!"
  * Introduce our robots:
    * Stage (simulator)
    * ROS Pi Bot
    * BotVac
    * Feya
    * Loki
    * Magni
  * All of our robots run ROS (Robot Operating System)
  * We'll talk more about ROS next time

## Video 2: ROS: Robot Operating System

* ROS stands for "Robot Operating System"
* Created by Willow Garage (now closed) which was funded
  by an early Google employee.
* Currently maintained by Open Source Robotics Foundation.
* Show stage demo running move base (i.e. the Girts demo.)
* Explain that "Stage" is a simulator with a robot, walls,
  a Lidar (explain what a lidar is).
* Show move base in operation.
* Now show `ros_rqt` graph of nodes and topics.
* Explain about nodes, topics, and services.
  * Node is a *nix process
  * Node's talk to one another via topics and services
    * A topic is multi-cast.
    * A service is a remote procedure call.
* Also explain about parameters.
* All nodes, topics, services, and parameters have a namespace.
* Show `rosnode list`, `rostopic list`, `rosservice list`, and
  `rosparam list`.
* Lastly, ROS launch files are used to fire off all of the
  nodes and plug them all together.
* Next is installing via virtual box.

## Video 3: Installing ROS Via Virtualbox

* ROS currently only runs under Ubuntu Linux.
* If you are running some other operating system (e.g. Windows,
  MacOs, Non-Ubuntu Linux, Solaris), use virutalbox.
* Visit [Oracle VirtualBox](https://www.virtualbox.org/) and download.
* Download VirtualBox image from UR.
* Download the Extension Pack (Is that need for VB 5.x?)
* Start virtualbox image.
* Explain that we are running lubuntu/LXDE.
* Login as user ubuntu and user ubuntu
* Start a terminal window
* Fire off stage demo again
* Show it working again.
* Show `rqt_graph` again.
* Show `rosnode list`, `rostopic list`, `rosservice list`,
  `rosparam list` again.

## Video 4: 1st ROS Program: Keyboard Teleop

* Talk about github.com.
* Create a github account.
* Fork a github repository for the template.
* Create a catkin workspace.
* Explain what a catkin is and how it relates to Willow Garage
* Go to src and do a git clone
* cd into dir and run `urcatkin_make`
* Run program using roslaunch
* Explain that we always use Python for beginners
* Show `f xxx` and `r xxx` commands to go forward and turn right.
* Bring up an editor (vi or emacs) and look at code.
* Walk people through the code.
* Show how to add `b xxx` backward and command.
* Explain about Ubuntu (i.e. Debian) packages
* Explain about ROS packages.

## Video 5: 2nd ROS program: Keyboard Move Base

* Show basic program.
* Add way point recording
* Add way point "goto"

## Video 6: ROSberry Pi Bot / Ceiling Fiducials

* Show ceiling fiducial and explain its components
* Show ceiling fiducials on ceiling
* Load up micro-SD card
* Plug in Ethernet cable
* (Optional) plug in HTMI cable and USB keyboard
* Power up. Login. Set host, WiFi, and user.
* On laptop use `ssh`

## Video 7: Networking and WiFi

* Hostnames *must* be unique
* We use zerconf => requires common subnet
* WiFi is a bit tricky
* Dual-band 2.4GHz/5GHz
* Configuring WiFi on Robot
* Use a good WiFi/router

## Video 9: BotVac

## Video 10: Loki

## Video 11: Magni

