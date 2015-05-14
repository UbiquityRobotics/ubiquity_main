# ubiquity-misc

Miscellaneous Stuff Used by the Ubiquity Team

## `rpi2-build-image.sh`

This shell script is used to build the Ubuntu 14.04LTS
kernel image that is used by the Ubiquity team.  This shell
script needs to be run on an Raspberry Pi 2 (hereafter shorted
to RasPi2.)  This script can be run under any RasPi2 kernel
that supports `apt-get`.  Thus, you can go to the
  [Raspberry Pi Downloads](https://www.raspberrypi.org/downloads/)
page, download Raspian on to a micro-SD card, and run this script
under Raspian to generate Ubuntu 14.04LTS kernel image that runs
on a RasPi2.  This script also runs fine on the generate Ubuntu
14.04LTS kernel.  (Yes, it is recursive!)

This shell script is run on the RasPi2 as follows:

	sudo rm -rf /srv
	cd {directory that contains rpi2-build-image.sh}
        sudo ./rpi2-build-image.sh

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

* The files can be copied directly onto a micro-SD card.
  This can be done directly from the RasPi2 using a appropriate
  USB to micro-SD adaptor.  The command looks as follows:

        sudo apt-get install -y bmap-tools
        sudo bmaptool copy --bmap *.bmap *.img /dev/XXXX

  where XXXX is the appropriate raw device name for the
  micro-SD card.

  The way you determine what XXXX is as follows:

  1. Do the following command:

        sudo blkid

  2. Insert the USB to micro-SD adaptor into one of the RasPi2 ports
     and repeat the command:

        sudo blkid

  3. Now visually commpare the results.

     If see a one or two new lines show up that look like:

        /dev/sdX#: UUID="..." TYPE="..."

     The `X` will be a lower case letter (typically `a` or `b`)
     and the `#` will be a digit like `0` or `1`.
     The `/dev/XXX` for the `bmaptool copy ...` command will be
     `/dev/sdX` where `X` is the lower case letter.

     Alternativley, if you see one or two lines that look like:

        /dev/mmcblk@p# UUID="..." TYPE="..."

     where both `@` and `#` are digits like `0` or `1`.
     The `/dev/XXX` for the `bmaptool copy ...` will be `/dev/mmcblk@`
     (i.e. no `p#`.)
    
Once this micro-SD card is plugged into the RasPi2 and
booted, you should be able to log into the RasPi2 over
the Ethernet as:

        ssh -l ubuntu ubuntu.local

It will proably ask a question or two (answer `yes`) and
then a `... password:`  The password is `ubuntu`.  You should
see a prompt that looks like:

        ubuntu@ubuntu:~$

You are in.  Now you need to do some additional steps.  These
steps can be found in
  [Raspberry Pi 2 ARM Ubuntu](https://wiki.ubuntu.com/ARM/RaspberryPi).
The steps are repeated here:

1. Expand the 2nd partion to the full size of the micro-SD card:

        sudo fdisk /dev/mmcblk0
        # Delete 2nd partition with (d,2)
        # Recreate 2nd partition with (n,p,2,enter,enter)
        # Write the new 2nd partition out (w)
        #
        # Now immediately reboot:
        sudo reboot

2. Resize the file system:

        sudo resize2fs /dev/mmcblk0p2

3. Install a swap file:

        sudo apt-get install dphys-swapfile

4. Make sure that you have the linux-firmware (should be already done).

        sudo apt-get install linux_firmware

5. Make sure the file `/etc/modules-load.d/raspi-camera.conf`:

        sudo sh -c 'echo "bcm2835-v4l2 gst_v4l2src_is_broken=1" > /etc/modules-load.d/raspi-camera.conf'

6. Create a catkin workspace:

        cd ~
        mkdir -p catkin_ws/src
        cd catkin_ws
        catkin_make

7. Add `~/devel/setup.bash` to the end of `~/.bashrc`:

        echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc


## `gscam` Notes

The following commands:

        sudo apt-get build-essential g++
        cd ~
        mkdir -p catkin_ws/src
        cd catkin_ws
        catkin_make
        source devel/setup.bash
        (cd src ;  git clone https://github.com/ros-drivers/gscam)

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

