# Download and Installing the Ubiquity-Ubuntu-ROS System Image

The Ubuquity/Ubuntu/ROS System Image is a Ubuntu 14.04LTS
System for the ARM7 hard float architecture.  The process
described below installs or configures many Ubuntu and ROS
packages along with a ROS catkin workspace.  The purpose is
to ensure that all of the Ubiquity robot platforms start
from a common software base.

This image only works for Ubuntu 14.04LTS (i.e. "Trusty").
Furthermore, these instructions are written assuming that
your laptop/desktop is running at least Ubuntu 14.04.

The most common way of install the image is to download
the image to your laptop/desktop and copy it onto a micro-SD card.
The micro-SD card is plugged into a Raspberry Pi 2 and the
Raspberry Pi 2 is powered up.  Please do the following steps:

1. Get a micro-SD card that is at least 16GB in size.  In addition,
   make sure you have either a USB to micro-SD adapter or a memory
   card micro-SD adapter.  Plug the micro-SD card into the adapter.
   Do not plug the adapter into the computer.  If you are running
   Ubuntu under a virtual machine, we strongly recommend that you
   get the USB to micro-SD adapter.

2. Download the image using the following commands:

        cd /tmp      # Or someplace else if you choose...
        wget http://ubiquityrobotics.com/2015-10-14-ubuntu-trusty.zip
        # This takes a while, it is ~1.2GB in size.
        # Unpack it as follows:
        unzip *.zip

3. The next steps are kind of arcane Linux stuff.  They actually
   make sense to a long term Unix/Linux person, but look very
   strange to somebody who is not a Unix/Linux expert.  In the
   steps below you will see upper case text enclosed in curly
   braces (e.g. `{DRIVE_NUMBER}`, `{DRIVE_LETTER}`,
   `{PARTITION_NUMBER}`, etc.)  This formatting indicates that
   some letter, digit, etc, will show up insted of the text
   in the curly braces.

   For a memory card to micro-SD adapter, the device name looks
   as follows:

        /dev/mmcblk{DRIVE_NUMBER}

   where `{DRIVE_NUMBER}` is a single digit from `0` to `9`.

   For the USB to micro-SD adapter, the device name looks like:

        /dev/sd{DRIVE_LETTER}

   where `{DRIVE_LETTER}` is a single lower case letter from
   `a` to `z`.

   Run the commands below:

        # Keep your adapter card out of the computer:
        sudo blkid > /tmp/before
        # Now plug your adapter card into the computer:
        sudo blkid > /tmp/after
        # Now find out what changed:
        diff /tmp/before /tmp/after

   For the memory card to micro-SD adapter, you should get
   output that looks like:

        3a4,5
        > /dev/mmcblk0p1: SEC_TYPE="msdos" UUID="7723-9122" TYPE="vfat" 
        > /dev/mmcblk0p2: UUID="e73646c3-4134-4793-b4bf-49a8ced6b0cb" TYPE="ext4" 
   For this example output, your device is `/dev/mmcblk0`.  Do *NOT*
   not tack a `p0` or `p1` onto your device name.

   For the USB to micro-SD adaptor, you should get some putput
   that looks like:

        3a4,5
        > /dev/sdb1: SEC_TYPE="msdos" UUID="7723-9122" TYPE="vfat" 
        > /dev/sdb2: UUID="e73646c3-4134-4793-b4bf-49a8ced6b0cb" TYPE="ext4 

   For this example output, your device is `/dev/sdb`.  Again, *NOT*
   tack the `0` or `1` onto the end of the device name.

4. For the USB to micro-SD adapter *ONLY*, unmount both of the
   partions.  Ignore all error messages:

        sudo umount /dev/sd{DRIVE_LETTER}1
        # Ignore any error ouput
        sudo umount /dev/sd{DRIVE_LETTER}2
        # Ignore any error output

   Where you use exactly the same `{DRIVE_LETTER}` that you determined
   in the previous step:

5. Copy the `.img` file onto the micro-SD card.
   Run the following commands (Remember to replace `/dev/XXXX`.):

        # In the same directory as the previous steps (i.e. `/tmp`):
        sudo apt-get install -y bmap-tools
        sudo bmaptool copy --bmap *.bmap *.img {DEVICE}
        sudo sync

    where `{DEVICE}` is either `/dev/mmcblk{DRIVE_NUMBER}` or
    `/dev/sd{DRIVE_LETTER}`.

6. Remove the micro-SD card from the adaptor and plug it into
   Raspberry Pi 2 micor-SD slot.  This slot is on the *back*
   of the Raspberry Pi 2.

7. Connect an RJ45 Ethernet cable between your Raspberry Pi 2 and
    your router.

8. You need to bring up "zeroconf".  "zeroconf" is system of
   software tools that will allow you to access the Raspberry Pi 2
   using the name `ubuntu.local` rather than an internet addess
   (e.g. 192.168.1.123 .)  On you desktop/laptop do the following:

        # Install the `avahi` Linux "zeroconf" server:
        sudo apt-get install libnss-mdns
        # To ensure that it installed correctly (note the use of back quotes
        # around `hostname`:
        ping -c 5 `hostname`.local
        # You should get 5 ping responses.  If not, "zeroconf" is
        # not working.

9. Apply power to the Raspberry Pi 2 via the micro-USB
   connector on the edge of the the Raspberry Pi 2.

10. Wait about a minute.  The LED's on the Raspberry Pi 2
    should stop blinking.

11. Connect to the Raspberry Pi 2 from your laptop desktop:

        # Log into your Raspberry Pi under user name `ubuntu`:
        ssh ubuntu@ubuntu.local
        # If you asked a yes/no questions, answer `yes`.
        # Password is `ubuntu` (which is the same as the user name):
        # You should see a prompt that looks like:
        ubuntu@ubuntu:~$

12. Expand the 2nd partion to the full size of the micro-SD card:

        sudo fdisk /dev/mmcblk0
        # Delete 2nd partition by typing:
        # The letter `d` followed the [Enter] key.
        # Now type the digit `2` followed by the [Enter] key.
        # Recreate 2nd partition by typing:
        # `n`, [Enter], `p`, [Enter], `2`, [Enter], [Enter]
        # Write the new 2nd partition out by typing:
        # `w`, [Enter].
        
        # Now immediately reboot:
        sudo reboot

13. Login again and resize the file system:

        # From you deskop/laptop:
        ssh ubuntu@ubuntu.local
        # Note the `p2` on the end of `/dev/mmcblk0p2`
        sudo resize2fs /dev/mmcblk0p2
        # For fun, see how much space is available on `/dev/mmcblk0p2`
        df -kh

14. Install a swap file:

        sudo apt-get install -y dphys-swapfile
        # Now immediately reboot:
        sudo reboot

15. Now is a good time to update your system:

        # From you deskop/laptop:
        ssh ubuntu@ubuntu.local
        # Now update and upgrade:
        sudo apt-get update
        sudo apt-get -y upgrade

16. Change the hostname and set up the WiFi:

        cd ~/catkin_ws/src/ubiquity_main
        # Update the code (it changes):
        git pull
        sudo ./configure.py
        # Follow the menu to change the host name an optionally configure
        # the WiFi.
        sudo reboot
        # Note, use the new host name when you log back in.
        ssh ubuntu@new_host_name.local

17. Set your git user name and E-mail address.  (Eventually this will
    be done from `configure.py`):

        # We should modify `configure.py` stuff to do this:
        git config --global user.email "your.email@whatever"
        git config --global user.name "First Last"
        git config --global push.default simple

    where you fill in the appropriate fields in the quotes.

18. This is where add packages to install that did not make
    it into the latest system image.  Please add the following
    packages:

        sudo apt-get install -y chrony
        sudo apt-get install -y ros-indigo-tf2-kdl # ROS Arduino Bridge only
        sudo apt-get install -y ros-indigo-joy-input

19. Fix .git permissions:

        cd ~/catkin_ws/src/ubiquity_main
        sudo chown ubuntu -R .
        sudo chgrp ubuntu -R .

20. For Magni platform only:

        cd ~/catkin_ws/src
        sudo apt-get install -y ros-indigo-hardware-interface
        sudo apt-get install -y ros-indigo-controller-manager
        git clone https://github.com/UbiquityRobotics/ubiquity_motor.git
        (cd .. ; catkin_make)

You are done.

## Constructing the System Image from Scratch

This section is the documentation of how to build the system
`.img` file.  This is definitely documentation for experts only.

This Ubiquity/ROS/Ubuntu System image is constructed with
two scripts:

* `rpi2-build-image.sh` which does most of the building,  and

* `cleanup.sh` does the wrap-up work.

It should be possible to run these scripts on a either
Ubuntu 14.04LTS system running 64-bit x86 architecture
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


