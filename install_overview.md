# Installing and Configuring Ubiquity Robotics Software Development Environment

Before you can develop software for you Ubiquity Robotics robot
platform, you need to assemble the necessary hardware followed
by downloading the necessary software from the internet and
properly configuring it.  This document provides you with an
overview of all the required steps.

## Overview

The primary issue that makes the Ubiquity Robotics (hereafter
abbreviated as UR) Software Development Environment (hereafter
abbreviated as SDE) complicated is that the UR robot platforms
are mobile.  A mobile platform can independently move around
in its own environment.  What this means is that the UR robots
are programmed  without a display, keyboard, and mouse directly
connected to the robot.  A robot that is without out a connected display,
keyboard and mouse is said to be in a "headless" configuration.

The way you program a headless UR robot is with another computer
that we call the host computer.  The host computer has the attached
display, keyboard and mouse.  It can be either a desktop computer
or a laptop.  The host computer connects to the UR mobile robot
via a WiFi network connection.  This is a bit involved and you
be carefully taken through the unfortunately somewhat tedious process.

All of the current UR robot platforms only support the Raspberry Pi 3
processor board.  The host computer hardware requirements are more
flexible:

* Either an Intel or AMD 64-bit Intel with 3 or more processor cores.

* RAM memory should be 16 Gigabytes or more.

* Disk space should be 256 Gibaytes or more.

* Some 3D graphics acceleration is required.  NVidia and AMD graphics
  acceleration is slightly preferred over Intel.

In general, there is no need to go overboard on the host computer hardware.

Currently, all the UR mobile robot platforms run the Robot Operating
System (hereafter abbreviated as ROS.)  There are multiple versions of ROS.
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
the UR robot.  Actually, it is fairly to install the necessary software on
the robot, you just download a preconfigured version of ROS Kinetic on
Ubuntu 16.04LTS.  Installing this software on a the host computer can be
more involved depending upon whether Linux, Windows, or MacOS X is being
run natively on the host computer hardware.  For Windows and MacOS X, some
sort of virtualizer (e.g. VirtualBox, VMWare, etc.) software needs to be installed
before Ubuntu 16.04LTS can be installed.  Again, this document will help
you perform the necessary installation and configuration.

Please step through the following installation steps:

1. If you are running either Windows or MacOS, install the 
   [virtualization software](install_virtualization.md).

2. Install
   [Ubuntu 16.04LTS](install_ubuntu.md) linux distribution on host computer.

3. Install
   [ROS Kinetic](install_ros.md)
   software on host computer.

4. Install
   [Ubiquity Robotics Software](install_ubiquity_host_software.md)
   on host computer.

5. Download
   [Raspberry Pi3 image]
   (install_rpi3_micro_sd.md) and install onto a micro-SD card.

6. Install micro-SD card onto robot and
   [start the robot](install_start_robot.md).

7. Configure 
   [WiFi connection](install_configure_wifi.md)
   between host and robot.

8. Configure and
   [run the first program](install_run_first_program.md).

That is it.



