
## Installing ROS on Ubuntu on VirtualBox
September 2015
March 2016 work in progress

* Download Ubuntu 14.04LTS.  You need a file with
  a suffix of .iso. (A .iso file is an image of a CD/DVD ROM.)
  You will need to feed this file to VirtualBox to set up your
  virtual machine.  There are several flavors of Ubuntu 14.04LTS that can be downloaded; most people download the "default" version, which uses the Unity desktop:

    http://www.ubuntu.com/download/desktop        # choose the 64-bit version

  Some use Kubuntu instead:

    http://www.kubuntu.org/getkubuntu

* Download and install VirtualBox for your 64-bit platform (amd64 is required):

    https://www.virtualbox.org/wiki/Downloads

* Start up VirtualBox and click on **New** to create a new virtual machine.
  Give your new virtual machine a Name (how about "yourname-ros"), Type (Linux), version (Ubuntu (64-bit)), 
  and a password.  It is reasonble to default pretty much everything
  else, except the virtual hard disk size, which should be at least 16 GB.  Do not start your virtual machine yet.
  
  Your new VM should appear in the list of virtual machines, and should be selected.

* Click on the **Settings** button to bring up the virtual machine settings panel.

  * In the settings panel, click on **System** and make sure
    that the boot order is set to try CD-ROM first and disk second.
    Not that it matters, but the floppy disk should be disabled.

  * In the settings panel, click on **Display**.  Given
    how intense `rviz` is with graphics, upping the video memory
    to 64MB seems prudent.  Enable 3D acceleration as well.

  * In the settings panel, click on **Storage**.  You need
    to find or make an optical drive (which should be listed there).  It should show "empty".  Now click the word "empty", and attach
    the Ubuntu .iso file to the drive. There is a small icon picturing an optical disk next to the optical drive name for doing this.

  * In the settings panel, click on **Network**.  Select
    the Bridged Adapter.

 * Now you can start your virtual machine by clicking on the
  **Start** button. It will boot from the .iso file that you have loaded into the (virtual) optical drive. Choose the "Install Ubuntu" option. Do not download updates while installing. The remaining installation steps are self-explanatory. The virtual machine should close and restart automatically, but if it does not, close it (File/Close) and restart it. 


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
  Guest Additions" to solve the small screen issues.  This URL is useful:

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

	Fire off the `VBoxLinuxAdditions.run` script:
        cd /path..to/VBOXADDITIONS*          
		sudo ./VBoxLinuxAdditions.run     # Takes a few minutes

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

  * Reboot your virtual machine:

        sudo reboot

  * Log in again and bring up a terminal window.  For convenience, lock the terminal application to the Launcher. At this point the VM console will be able to expand to fill the window provided for it by VirtualBox. In the **Devices** menu you will also be able to activate the shared clipboard, mouse integration, drag & drop, and shared folders (with the host).
  
  * Now follow the instructions for installing ROS indigo:

        http://wiki.ros.org/indigo/Installation/Ubuntu

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
