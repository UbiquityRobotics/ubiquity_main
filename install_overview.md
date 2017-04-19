# Installing and Configuring the Ubiquity Robotics Software Development Environment

Before you can develop software for your Ubiquity Robotics robot platform,
you need to download the necessary software from the internet and properly
configure it.  This document provides you with an overview of all the required
steps.

## Overview

The Ubiquity Robotics Software Development Environment provides
all of the software tools that you need to develop software for
a Ubiquity Robotics robot.  Since "Ubiquity Robotics Software
Development Environment" is a real mouthful, it tends to be
abbreviated as "UR SDE", where "UR" stands for "Ubiquity Robotics"
and "SDE" stands for "Software Development Environment".

The primary issue that makes the UR SDE  complicated is that the UR
robot platforms are "headless".  What "headless" means is that there
is no display, keyboard, or mouse attached to the robot.  The reason
why UR robots are headless is because it is difficult to develop
software when your display, keyboard, and mouse are moving around.

The way the UR SDE is structured, all software development is done
at stationary computer called the host computer.  The host computer
has the attached display, keyboard and mouse.  The host computer can
be either a desktop or a laptop computer.  The host computer connects
to the UR robot platform via a WiFi network connection.

Setting up the stationary host computer and connecting it to a UR
robot platform is a bit involved.  This document will carefully
guide you through the unfortunately somewhat tedious process.

The host computer hardware requirements are pretty flexible:

* A 64-bit Intel or AMD processor with 2 or more cores.

* RAM memory should be 8 Gigabytes or more.

* Disk space should be 256 Gigabytes or more.

* Some 3D graphics acceleration is required.  NVidia and AMD graphics
  acceleration is slightly preferred over Intel graphics acceleration.

* Internet access is required.

In general, there is no need to go overboard on the host computer hardware.

Currently, all the UR robot platforms run the Robot Operating System
(hereafter abbreviated as ROS.)  There are multiple versions of ROS.
All of the current UR robot platforms use the ROS version called
"ROS Kinetic Kame", which is usually abbreviated to "ROS Kinetic".

The only operating system that is fully supported for ROS Kinetic is the
Ubuntu 16.04LTS (Xenial Xerus) release.  This needs a little decoding.  Ubuntu
is something called a Linux distribution.  All Ubuntu Linux distributions
are managed by a company called Canonical.  16.04 stands for the April 2016
(year xx16 and month 04).  LTS stands for Long Term Support, which means
that patches and upgrades will be available for 5 years.  Each release is
named after an animal, preceded by an adjective.  In this case, the chosen
animal is an African ground squirrel called the xerus and the chosen adjective
is "xenial" which basically means "hospitable".   For the rest of this document,
you will see "Ubuntu 16.04LTS (Xenial Xerus) abbreviated to "Ubunutu 16.04LTS".

The key thing to understand is that as far a Ubiquity Robotics is concerned,
only ROS Kinetic running on Ubuntu 16.04LTS is supported.  Period.  What
this means is that ROS Kinetic needs to run on both the host computer and
the UR robot platform.  Actually, it is fairly to install the necessary software
on the robot, you just download a preconfigured version of ROS Kinetic on
Ubuntu 16.04LTS.  Installing this software on a the host computer can be
more involved depending upon whether Linux, Windows, or MacOS X is being
run natively on the host computer hardware.  For Windows, MacOS X and somee
versions of Linux, virtualizer softare (e.g. VirtualBox, VMWare, etc.) needs
to be installed before Ubuntu 16.04LTS can be installed.  Again, this document
will help you perform the necessary installation and configuration.

## Installation Roadmap

The installation occurs in three distinct phases:

1. Bring up ROS Kinetic on your host computer.

2. Bring up ROS on your Raspberry Pi 3.

3. Plug your Rasperry Pi 3 to your UR robot and connect to it via WiFi.

Please note that phase 2 can be done with a stand-alone Raspberry Pi 3 board.
 Thus, you can be sure that your UR SDE is ready use before the UR Platform
that you ordered is delivered to you.

### Phase 1: Install and Configure the Host Computer

Please step through the following installation steps to bring up the UR SDE
on your host computer:

1. [Install virtualization](install_virtualization.md)
   software on host computer.

2. [Install ROS Kinetic](install_ros.md)
   software on host computer.

3. [Install Ubiquity Robotics Software](install_ubiquity_host_software.md)
   on host computer.

4. [Install desired desktop software](install_ubuntu.md)
   on host computer.

5. [Run Robot Simulator](install_run_robot_simulator)
   on host computer.

### Phase 2: Install and Configure the Raspberry Pi 3 Board

1. Download
   [Raspberry Pi3 Image]
   (install_rpi3_micro_sd.md) and install onto a micro-SD card.

2. Install micro-SD card onto robot and
   [start the robot](install_start_robot.md).

3. [Configure WiFi connection](install_configure_wifi.md)
   between host and robot.


### Phase 3: Attach Raspberry Pi 3 to UR Robot Platform

1. [Attach Raspberry Pi3](install_attach_pi3_to_robot.md)
   to UR robot platform.

2. [Run UR Demo Programs](install_run_ur_demos.md)
   on your UR robot platform.

### Conclusion

{Conclusion goes here.}





