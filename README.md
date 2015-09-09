# ubiquity-misc

This repository constains files used by the Ubiquity team
that have no other obvious place to be put.  For now,
this repository contains all othe documentation and shell
files used to the Ubiqutiy/Ubuntu/ROS Kernel image.

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

## Ubiquity/Ubuntu/ROS Kernel Image

The Ubuquity/Ubuntu/ROS Kernel Image is a Ubuntu 14.04LTS
Kernel for the ARM7 hard float architecture.  This image
has many Ubuntu and ROS packages preinstalled along with
a ROS catkin workspace.  The primary purpose of this image
is to ensure that all of the Ubiquity robot platforms start
from a common software base.

This kernel image only works for Ubuntu 14.04LTS
(i.e. "Trusty").

### Download and Installing the Ubiquity/Ubuntu/ROS Kernal Image

The most common way of install the image is to download
the image and copy it onto a micro-SD card.  THe micro-SD
card is plugged into a Raspberry Pi 2 and the Raspberry Pi 2
is powered up.  Please do the following steps:

1. Get a micro-SD card that is at least 4GB is size.  Frankly,
   we recommend a minimum of 16GB.

2. Get either a USB/micro-SD adapter or regular SD to micro-SD
   card adaptor and plug your micro-SD card into it.

3. On your laptop/desktop, run the following command on your
   Ubuntu 14.04LTS kernel from the command line:

        sudo blkid

   This will print a bunch of stuff out.

4. Insert your micro-SD adapter into your desktop and run
   the same command again:

        sudo blkid

   Again, this will print a bunch of stuff out.

5. For the `bmaptool copy` command further below, we are going
   to need to type in the correct location to copy the
   Ubiquity/Unbuntu/ROS kernel to.  This will have a form
   of `/dev/XXXX`, where `XXXX` depends upon your system.

   Now look through the two lists and visually search for the
   new entries from the first and second invocations of `blkid`.

   If see a one or two new lines show up that look like:

        /dev/sdX#: UUID="..." TYPE="..."

   The `X` will be a lower case letter (typically `a` or `b`)
   and the `#` will be a digit like `0` or `1`.
   The `/dev/XXXX` for the `bmaptool copy ...` command will be
   `/dev/sdX` where `X` is the lower case letter.

   Alternatively, if you see one or two lines that look like:

        /dev/mmcblk@p# UUID="..." TYPE="..."

   where both `@` and `#` are digits like `0` or `1`.
   The `/dev/XXX` for the `bmaptool copy ...` will be `/dev/mmcblk@`
   (i.e. no `p#`.)

   Write down the /dev/XXXX value we have just determined, we are
   going to need it later.

6. Download the image using the following commands:

        cd /tmp      # Or someplace else if you choose...
        #wget http://kchristo.homeip.net/files/rpi2_kernel.zip
        #wget http://gramlich.net/2015-07-12-ubuntu-trusty.zip
        wget http://gramlich.net/2015-07-19-ubuntu-trusty.zip
        # This takes a while, it is ~570MB.

7. Unpacking the zip file below should result in a *.img and *.bmap file.

        unzip *.zip

8. Copy the .img file onto the micro-SD card.  This is the
   place where you will type in the value for `/dev/XXXX`
   that was determined in step 5 above.  Run the following
   commands (Remember to replace `/dev/XXXX`.):

        sudo apt-get install -y bmap-tools
        sudo bmaptool copy --bmap *.bmap *.img /dev/XXXX
        sudo sync

9. Remove the micro-SD card from the adaptor and plug it into
   Raspberry Pi 2 micor-SD slot.  This slot is on the *back*
   of the Raspberry Pi 2.

10. Connect an RJ45 Ethernet cable between your Raspberry Pi 2 and
    your router.

11. Apply power to the Raspberry Pi 2 via the micro-USB
    connector on the edge of the the Raspberry Pi 2.

12. Wait about a minute.  The LED's on the Raspberry Pi 2
    should stop blinking.

13. Connect to the Raspberry Pi 2 from your laptop desktop:

        ssh ubuntu@ubuntu.local
        # If you asked a yes/no questions, answer `yes`.
        # Password is `ubuntu`
        # You should see a prompt that looks like:
        ubuntu@ubuntu:~$

14. Expand the 2nd partion to the full size of the micro-SD card:

        sudo fdisk /dev/mmcblk0
        # Delete 2nd partition with (d,2)
        # Recreate 2nd partition with (n,p,2,enter,enter)
        # Write the new 2nd partition out (w)
        #
        # Now immediately reboot:
        sudo reboot

15. Login again and resize the file system:

        # From you deskop/laptop:
        ssh ubuntu@ubuntu.local
        sudo resize2fs /dev/mmcblk0p2

16. Install a swap file:

        sudo apt-get install -y dphys-swapfile

17. Make sure that you have the linux-firmware (should be already done).

        sudo apt-get install -y linux-firmware

18. Now is a good time to update your system:

        sudo apt-get update
        sudo apt-get upgrade

19. Make sure the file `/etc/modules-load.d/raspi-camera.conf` exists:

        # This command is no longer needed; it has already been done
        # with the standard image.
        sudo sh -c 'echo "bcm2835-v4l2 gst_v4l2src_is_broken=1" > /etc/modules-load.d/raspi-camera.conf'

20. Create a catkin workspace:

        # This should already have been done in the kernel image:
        cd ~
        mkdir -p catkin_ws/src
        cd catkin_ws
        catkin_make

21. Install your one or more of you favorite editor(s):

        # This has already been done in the latest kernel image:
        sudo apt-get install -y vim     # For you vi folks
        sudo apt-get install -y emacs   # For you emacs folks

22. Edit `/etc/hostname` and change the hostname from `ubuntu` to something
    else like `my_robot`, `funbot`, etc.  (This is step will be replaced
    with Kurt's configuration stuff.)

23. Edit `/etc/hosts` and change the line `127.0.1.1 ubuntu` to:

        # Again, eventually Kurt's configuration code will do this:
        127.0.1.1  new_hostname new_hostname.local

24. Add the following lines to the end of `~/.bashrc`:

        # Again, Kurt's configuration code will eventually do this:
        source ~/catkin_ws/devel/setup.bash
        export ROS_HOSTNAME=`cat /etc/hostname`.local
        export ROS_MASTER_URI=http://`cat /etc/hostname`.local:11311

25. Rerun `~/bash.rc`:

        source ~/.bashrc

26. Using the `env | grep ROS` command verify that `ROS_HOSTNAME` is
    `XXX.local` where `XXX` is the hostname you put into `/etc/hostname`.
    Likewise, verify that `ROS_MASTER_URI` is `http://XXX.local:113111`
    where `XXX` is the same hostname.

27. Install some additional software:

        # This has already been done in the latest kernel image:
        sudo apt-get install -y wpasupplicant minicom setserial mgetty wireless-tools
        sudo apt-get install -y --reinstall build-essential git

28. Install some more ROS packages:

        # This has already been done in the latest kernel image:
        sudo apt-get install -y ros-indigo-ros-tutorials ros-indigo-joystick-drivers python-serial              
        sudo apt-get install -y ros-indigo-serial ros-indigo-navigation ros-indigo-tf-conversions
        sudo apt-get install -y ros-indigo-robot-model ros-indigo-tf2-geometry-msgs

        cd ~/catkin_ws/src # to pull code that will be compiled
        git clone https://github.com/DLu/navigation_layers.git
        git clone https://github.com/ros/robot_state_publisher.git
        #git clone https://github.com/bosch-ros-pkg/usb_cam.git

        git clone https://github.com/ros/robot_model.git  # required for crash fix for now

29. Install some Ubiquity Robotics packages:

        #git clone https://github.com/hbrobotics/ros_arduino_bridge.git
        git clone https://github.com/UbiquityRobotics/ros_arduino_bridge.git
        git clone https://github.com/UbiquityRobotics/joystick_input.git
        git clone https://github.com/UbiquityRobotics/fiducials.git
        # Optional for Wayne's experimental config file stuff right now:
        git clone http://github.com/UbiquityRobotics/robot-configurations.git
        (cd ~/catkin_ws ; catkin_make)

30. Set your git user name and E-mail address:

        # We should modify Kurt's configurations stuff to do this:
        git config --global user.email "your.email@whatever"
        git config --global user.name "First Last"

    where you fill in the appropriate fields in the quotes.

31. Setup the baud rate on the serial port by adding the stty line to
    `rc.local`.  Add the line at the end just before the `exit 0`.

        # We still need to do this automagically:
        sudo vi /etc/rc.local 
        stty -F /dev/ttyAMA0  115200

32. Remove delays if you want to avoid 2 minute bootup sleeps that
    are not really a RasPi issue.

        sudo vi /etc/init/failsafe.conf
        # Now remove the 40 and 59 sec sleeps right after message
        # 'Waiting for network configuration'


### Installing USB WiFi dongles:

The are two common USB WiFi dongles.  One is based on the Realtek 8192
chip set and the other is based on the Realtek 8188 chips set.  The 8192
seems to work rather well, but the 8188 seems to cause more problems.

1. Remove the the persistent rules UDev rule:

        # This has already been done in the latest kernel image:
        sudo rm /etc/udev/rules.d/70-persistent-net.rules

2. Install the 8188EU kernel module:

        # This has already been done in the latest kernel image:
        cd /lib/modules/`uname -r`
        sudo ln -s kernel/drivers/staging/rtl8188eu/rtl8188eu.ko
        sudo modprobe r8188eu

3. Plug in your WiFi dongle and figure which kind you got:

        lsusb | grep -i realtek

   The 8188 gives:

        Bus 001 Device 004: ID 0bda:8179 Realtek Semiconductor Corp.

   and the 8192 gives:

        Bus 001 Device 004: ID 0bda:8178 Realtek Semiconductor Corp. RTL8192CU 802.11n WLAN Adapter

   You may get something else.

4. Now create/edit `/etc/network/interfaces` and add the following
   lines to your file:

        # WiFi dongle
        allow-hotplug wlan0
        auto wlan0
        iface wlan0 inet dhcp
          wpa-ssid "SSID"     # Replace SSS with your network SSID
          wpa-psk  "PWD"      # Replace PWD with your network password

5. Reboot.

        sudo reboot

6. Run `ifconfig` to see if device `wlan0` shows up:

        ifconfig

   If `wlan0` does not show up, something did not go right.  Feel the pain.


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

## Constructing the Kernel Image from Scratch

This Ubiquity/ROS/Ubuntu Kernel image is constructed with
two scripts:

* `rpi2-build-image.sh` which does most of the building,  and

* `cleanup.sh` does the wrap-up work.

It should be possible to run these scripts on a either
Ubuntu 14.04LTS kernel running 64-bit x86 architecture
or on a Raspberry Pi 2 (hereafter shortened to RasPi2.)

This shell script is run as follows:

        sudo rm -rf /srv
        cd {directory that contains rpi2-build-image.sh}
        sudo ./rpi2-build-image.sh

> * When running rpi2-build-image.sh on an intel computer, if I use the
> script as it stands, Very quickly get a failure in the script at the 
> beginning .  I get the error
>
> "Couldn't download dists/trusty/main/binary-amd64/Packages" that were 
>   clearly not amd64 packages (i.e. raspberrypi-bootloader, etc.)
> 
> I also ran the script pointing to a local mirror.  In that case, 
> I received an error that those particular files could not be authenticated.  
> The apt-get in the script at line ~103 used the "-y" parameter without the 
> "--force-yes" parameter so the script failed.  I ended up editing that line
> to force the yes and the script ran fine.  I would note that ubuntu 
> documentation states using "--force-yes" is somewhat dangerous.
> -- {Kurt} *


As of now, this script fails when it trys to unmount
`/srv/rpi2/trusty/build/chroot/proc`.  The work around
is to reboot the machine:

        sudo reboot

Now finish everything off by running the remainder of the
script that is now sitting in `.../cleanup.sh`.

        sudo cleanup.sh

The result show up in `/srv/rpi2/trusty/` as two files
with suffixes that end in `.img` and `.bmap`.  There
are two things that can be done:

* The files can be compressed into a `.zip` file and put
  on a internet so that other people can download and
  install the image onto a micro-SD card.  Here is the
  command that does it:

        zip rpi2_kernel.zip *.img *.bmap

  The resulting `rpi2_kernel.zip` file can be put up on
  a server.  Please feel free to change `rpi_kernel` to
  something with a bit more information.

* The files can be copied directly onto a micro-SD card
  using the bmap-tools listed above:

        sudo apt-get install -y bmap-tools
        sudo bmaptool copy --bmap *.bmap *.img /dev/XXXX

  where XXXX is the appropriate raw device name for the
  micro-SD card.


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

## WiFi Networking Issues

The problem is linux saves hard lan and wifi dongle macs and keeps
adding devices so you end up with say `eth2` but your
`/etc/network/interfaces` only has entry for `eth0`.

Here are some notes for sorting this out.  Tthe last half of this
set of cheet-sheet notes.


[Linux Network Specific]( https://help.ubuntu.com/lts/serverguide/network-configuration.html)

* `ifconfig   [eth0]`:
  Shows network interface(s) that have been configured or
  maybe partly configured (like WiFi that cannot find network)

* `sudo ifconfig eth0 10.0.0.100 netmask 255.255.255.0` :
  You can temp configure one till reboot.

* `/etc/hostname`:
  This is where system hostname is defined on reboot.
  Also edit `/etc/hosts` if you do change host name!

* `/etc/network/interfaces`:
  Entries tie hardware like `eth0` and `wlan0` to be of a specific
  network port type. KEY file, see other places in doc.

* `/etc/resolv.conf`:
  Holds DNS resolutions (ip addresses).  Best to use tool to do this
  `dmesg | grep eth` Can show if eth0 or wlan0 was re-named due to
  new flash image.  Come up with no net and edit `/etc/network/interfaces`

* `/etc/udev/rules.d/70-persistent-net.rules`:
  Edit this if you have left over `eth0` or `wlan0` from prior image
  so you get nice clean `eth0` and `wlan0`.
  This file has mac address to names like `wlan0`, `wlan1` and `eth0`
  and so on. Clean it out to have fresh discovery.

* `sudo ifup eth0`:
  Manually enable newly added interface from recent
  `/etc/network/interfaces`.  Can also use `sudo ifdown eth0`.

Sorting out linux images on ubuntu when networks get added:

* Boot up with hdmi monitor and usb keyboard usually (if no
  network no ssh ability)

* `sudo vi /etc/hosts` and `/etc/hostname` and set correct hostname
  for your system.

* `ifconfig` from here we want to see the eth0 and when wifi
  dongle is in `wlan0`.

* To hard-reset things, `sudo vi /etc/udev/rules.d/70-persistent-net.rules`

  1. Remove all the lines below top 2 lines of general comments as it
     is here that you may find your ethernet was nammed `eth2` or
     something and `/etc/network/interfaces` does not match so you
     get no network

  2. `sudo vi /etc/network/interfaces` and have just the use of `eth0`
     and `wlan0`, NOT higher numbers.

* Reboot and again on hdmi monitor.  Do `ifconfig` and hopefully
  you have `eth0` now and if you had wifi and config in
  `/etc/network/interfaces` for `wlan0` you also have wifi now

## Bring 8188 WiFi Dongle Notes:

WiFi with Kurts Kernel working for me today.

One tiny change from your modprobe so when I was in 
/lib/modules/3.18.0-24-rpi2 I used like you did and saw 
he .ko so did this:

        ln -s kernel/drivers/staging/rtl8188eu/rtl8188eu.ko

Difference was I had to then use :

        modprobe r8188eu
        # you had said modprobe rtl8188eu which does not work for me.

After above you should be able to see

        iwconfig  show wlan0

but it will still say unassociated because `/etc/network/interfaces`
not set yet so I then put these EXACT lines at END of
`/etc/network/interfaces` right from Kurt.

        # WiFi dongle
        allow-hotplug wlan0
        auto wlan0
        iface wlan0 inet dhcp
          wpa-ssid "2WIRE270"
          wpa-psk  "d4c33208f8fdc ... from wpa_passphrase 5aa32"

After reboot you should NOT see `iwconfig` showing unassociated
or similar word right after it says `wlan0`:

Also, I do NOT see `/dev/wlan0` (I too am puzzled??)
but with lsmod I see this line matching my modprobe argument:

        r8188eu               408198  0


`lsusb` gives me this:

        Bus 001 Device 004: ID 0bda:8179 Realtek Semiconductor Corp.

## Installing ROS on Ubuntu on VirtualBox

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

* It makes sense to install some editors:

        sudo apt-get intall vim emacs

That kind of wraps it all up.


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
p        git clone https://github.com/ros-drivers/gscam
        sudo apt-get install -y ros-indigo-image-common
        sudo apt-get install -y gstreamer0.10-plugins-good
        sudo apt-get install -y libgstreamer0.10-dev
        sudo apt-get install -y libgstreamer-plugins-base0.10-dev

        # Install joystick drivers and ROS Arduino Bridge:
        git clone https://github.com/UbiquityRobotics/joystick_drivers.git
        git clone https://github.com/UbiquityRobotics/ros_arduino_bridge

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
