# Installing Virtualization Software on the Host Computer

Unless your host computer is already natively running Ubuntu 16.04LTS
you need to do some work to get Ubuntu 16.04 LTS running on your host
computer.

## Overview

The steps involved are:

1. You need to figure out if you need to install virtualization software.
   (Most of you will need virtualization software.)

2. Download and install the virtualization software.  Ubiquity Robotics only supports
   the VirtualBox virtualization software from Oracle.

3. Create a virtual machine using VirtualBox.

4. Load Ubuntu 16.04 into the virtual machine.

5. Install the virtual box guest extensions into the virtual machine.

With no further adeau, you can get started now.

## Is Virtualization Software Needed

For this document, we partition native operating systems into ones
that are base Linux operating systems and non-Linux based operating systems
(e.g. Windows, Mac OS, Solaris, etc.).

You can skip reading the rest of this section,
if your host computer is running non-Linux native operating system.
All non-Linux native operating systems require the installation of
virtualization software.

If you are running Linux natively on your host computer please run the
following command in a terminal window:

        lsb_release -i -r

and it will output:

        Distributor ID: DDDDDD
	Release:        RRRRRR

where `DDDDDD` is the distributor name (e.g. Ubuntu, Redhat, etc.)
and `RRRRRR` is the release.

* If your distributor id is `Ubuntu` and the release number is 16.04, you are
  done and no virtualization software needs to be installed.

* If your distributor id is not `Ubuntu` you *must*
  install the virtualization software.

* If your distributor id is `Ubuntu`, but the release number is less than 16.04,
  you can upgrade your Ubuntu release to 16.04.  LTS releases always occur
  on in April of even years (i.e. 1x.04, where x is even.)  To upgrade,
  always go to the closest new LTS release and then upgrade LTS releases
  until you get to 16.04.  For example, to upgrade Ubuntu 11.10, first upgrade
  to 12.04, then 14.04 and finally to 16.04.  Use your web favorite search
  engine an search for "Upgrade Ubuntu xx.xx to yy.yy" to get instructions
  on how to do this.

* If your distributor id is `Ubuntu` and the release number is greater than 16.04,
  you are running a newer version that 16.04.  In theory, ROS Kinetic will
  run on such a system, but it is not heavily tested.  While it should work,
  Ubiquity Robotics does not support this configuration.  It is recommended
  that you install the virtualiation software so you can run 16.04 in
  the virtualization software.
  
## Installing the VirtualBox Virtualization Software

While there are multiple virtualization software systems out there,
Ubiquity Robotics has selected
[VirtualBox](https://www.virtualbox.org/)
from Oracle for its virtualization software.  This software is
currently available with out further charge.  You ware welcome
to try one of the others virtualization software system (e.g VMWare, etc.),
but Ubiquity Robotics can not provide with *any* support anything other
than VirtualBox.

You are basically going to be working through the first few chapters of the
[VirtualBox User Manual](https://www.virtualbox.org/manual/UserManual.html).
To better understand what is going on, selected portions of chapters 1 through 4
should be read.

* Please preread chapter 1 up to and including the section on extension packs.

* Please preread appropriate section chapter 2 that corresponds to 
  which native operating system you are running.

* Please skim through chapter 3 to see what sorts of things you need to
  configure for your virtual machine.

* Please skim through chapter 4 (Guest Additions).

The remaining chapters in the user manual contain useful information, but
you do not need to preread them.

Now it is time to get started:

* Download VirtualBox for your 64-bit platform:

    https://www.virtualbox.org/wiki/Downloads

* Install VirtualBox.  Instructions are at:
    
    https://www.virtualbox.org/manual/ch02.html#install-linux-host
	
* Run VirtualBox.

## Create Virtual Machine

1. [new1](install_images/new1.png)

2. [new2](install_images/new2.png)

3. [new3](install_images/new3.png)

4. [new4](install_images/new4.png)

5. [new5](install_images/new5.png)

6. [new6](install_images/new6.png)

7. [new7](install_images/new7.png)

8. [new8](install_images/new8.png)


* Download Lubuntu 14.04 LTS.  Lubuntu is a lightweight version of Ubuntu. You will get a file with
  a suffix of .iso. (A .iso file is an image of a CD/DVD ROM.)
  You will feed this file to VirtualBox to set up your virtual machine.	

    http://cdimages.ubuntu.com/lubuntu/releases/trusty/release/	        # choose the 64-bit PC (AMD64) desktop image


* Start up VirtualBox and click on **New** to create a new virtual machine.
  Give your new virtual machine a Name (how about "yourname-ros"), Type (Linux), and Version (Ubuntu (64-bit). It is reasonble to default pretty much everything
  else, except the virtual hard disk size, which should be at least 32 GB.  **Do not start your virtual machine yet.**
  
  Your new VM should appear in the list of virtual machines, and should be selected.

* Click on the **Settings** button to bring up the virtual machine settings panel.

  * In the Settings panel, click on **System** and make sure
    that the boot order is set to try CD-ROM ("Optical") first and disk second.
    Not that it matters, but the floppy disk should be disabled.

  * In the Settings panel, click on **Display**.  Given
    how intense `rviz` is with graphics, upping the video memory
    to 64MB seems prudent.  Enable 3D acceleration as well.

  * In the Settings panel, click on **Storage**.  You need
    to find or make an optical drive.  It is indicated by a small icon intended to look like a CD-ROM. 
	If there is one, it should show "empty".  Now click the word "empty", and attach
    the Lubuntu .iso file to the drive. There is a small icon picturing an optical disk next to the optical drive name for doing this.  
	If there is no optical drive, add one to the IDE Controller.  Again, there is an icon for this.

  * In the Settings panel, click on **Network**.  Enabme the Network adapter, and select
    the Bridged Adapter.

 * Now you can start your virtual machine by clicking on the
  **Start** button. It will boot from the .iso file that you have loaded into the (virtual) optical drive. 
  
  Choose the "Install Ubuntu" option. 
  Do not choose the option to download updates while installing. The remaining installation steps are self-explanatory. 
  The virtual machine should close and restart automatically, but if it does not, close it (File/Close) and restart it. 

* Upon restart, log in with your password.   The screen will show up in 640 x 480 mode no matter how you resize the larger
  window.  Later steps fix this problem.

* Find a terminal window.  In Ubuntu, there is a Search icon at the top of the launcher (the column of icons on the left of the screen).  Search for "terminal"; this will find the terminal application.  In Kubuntu, it is found in the lower
  left corner **K with Gear** => **Applications** => **System** => **Terminal**.
  It is possible to drag the terminal icon from the launcher to the
  desktop. Start the terminal.  If you need help, try reading:

    http://askubuntu.com/questions/38162/what-is-a-terminal-and-how-do-i-open-and-use-it

* From the terminal, update the software:

        sudo apt-get update	     # Takes about a minute
        sudo apt-get upgrade     # Takes 1 to 10 minutes

* Now that you have a terminal, it is time to install the "VirtualBox
  Additions" to solve the small screen issues.  This URL is useful:

    http://www.binarytides.com/vbox-guest-additions-ubuntu-14-04/

  * Download some useful stuff:

        sudo apt-get install build-essential module-assistant dkms
        sudo m-a prepare

  * Now find the **Devices** submenu on your virtual box window.
    Select **Devices** => **VirtualBoxGuestAddtions.iso disk**.
    Sometimes Ubuntu will offer to run the disk image; let it do so. 

  * Otherwise, Ubuntu will normally have mounted the contents of the .iso on
    `/media/USERNAME` where `USERNAME`is your user name.  If it
    does not show up (i.e. try `ls /media/USERNAME`), try the following:
		sudo mkdir -p /cdrom
		sudo mount /dev/cdrom /cdrom
		ls /mnt

	Fire off the `VBoxLinuxAdditions.run` script, which may take a few minutes:
        cd /path..to/VBOXADDITIONS*          
		sudo ./VBoxLinuxAdditions.run     
-------------------------------------------- the following does not work, no response from vboxguest
 * Do the following and check that the VERSION and LINUXVERSION
    match what you have installed:

        # check loaded modules
        $ lsmod | grep -io vboxguest
        vboxguest
        # check module 
        $ modinfo vboxguest
        filename:       /lib/modules/LINUXVERSION-generic/updates/dkms/vboxguest.ko
        version:        VERSION
        license:        GPL
        description:    Oracle VM VirtualBox Guest Additions for Linux Module
        author:         Oracle Corporation
        .....
        $ lsmod | grep -io vboxguest | xargs modinfo | grep -iw version
        version:        VERSION
		
		___________________________________________________________________________down to here

  * Reboot your virtual machine:

        sudo reboot

  * Log in again and bring up a terminal window.  For convenience, lock the terminal application to the Launcher.  TODO: HOW?
  At this point the VM console will be able to expand to fill the window provided for it by VirtualBox. 
  In the **Devices** menu you will also be able to activate the shared clipboard, drag & drop, and shared folders (with the host).
  
  * Install avahi-daemon, which allows connection by machine name instead of IP address.
  
		sudo apt-get install libnss-mdns
		
At this point a virtual machine has been created and Lubuntu has been installed.  It is ready for the installation of ROS and the Ubiquity software.
  
  * Now follow the instructions for installing ROS indigo:

        http://wiki.ros.org/indigo/Installation/Ubuntu
		
		TODO:  we don't know how to add whatever to the repositories, as in the U. instructions
			
  * Some notes on the installation instructions:

	  * If you are not using Ubuntu, think about using `synaptic` to configure the repositories:

			sudo apt-get install synaptic
			sudo synaptic
			
		Use **Settings** => **Repositories** and select the appropriate repositories.  Click **OK**, then **Reload**.  Kill synpatic.

		Right before you install the full desktop install:

			sudo apt-get update
			
	Now install the desktop
		
        sudo apt-get install ros-indigo-desktop-full
		
  * Environment Variables
  
    Two environment variables, ROS_MASTER_URI and ROS_HOSTNAME will
    have to be set.  ROS_HOSTNAME must have the name of this virtual
    machine, and ROS_MASTER_URI must be the IP address (or equivalent)
    of the robot, and must be the same on both your VM and on the robot.
    Of course, you cannot set this until your robot is up. But you can
    set ROS_HOSTNAME now.

        env | grep ROS_HOSTNAME     # Show settings of env. var.
	export ROS_HOSTNAME="yourname-ros"  # if that's what you named it.

 * It makes sense to install some editors:

        sudo apt-get intall vim emacs

That kind of wraps it all up.
