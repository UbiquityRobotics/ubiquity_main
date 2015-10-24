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

1. Get a micro-SD card that is at least 8GB is size.  Frankly,
   we recommend a minimum of 16GB.

2. Get either a USB/micro-SD adapter or regular SD to micro-SD
   card adaptor and plug your micro-SD card into it.

3. On your laptop/desktop, run the following command on your
   Ubuntu 14.04LTS system from the command line:

        sudo blkid

   This will print a bunch of stuff out.

4. Insert your micro-SD adapter into your desktop and run
   the same command again:

        sudo blkid

   Again, this will print a bunch of stuff out.

5. For the `bmaptool copy` command further below, we will need the
   correct location to copy the Ubiquity/Unbuntu/ROS system to.
   This will have a form of `/dev/XXXX`, where `XXXX` depends upon your system.

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
        wget http://gramlich.net/2015-10-14-ubuntu-trusty.zip
        # This takes a while, it is ~1.2GB.

7. Unpacking the zip file below should result in a *.img and *.bmap file.

        unzip *.zip

8. Copy the .img file onto the micro-SD card.  This is the
   place where you will type in the value for `/dev/XXXX`
   that was determined in step 5 above.  Run the following
   commands (Remember to replace `/dev/XXXX`.):

        # In the same directory as the previous step:
        sudo apt-get install -y bmap-tools
        # Ralph Hipps says that he had to unmount USB to micro-SD
        # cards first.  Do this for `/dev/sdXXXX` devices only!
        # Do *NOT* attempt to unmount a `/dev/mmcblk` device:
        sudo umount /dev/XXXX
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
        # Note the `p2` on the end of `/dev/mmcblk0p2`
        sudo resize2fs /dev/mmcblk0p2
        # For fun, see how much space is available on `/dev/mmcblk0p2`
        df -kh

16. Install a swap file:

        sudo apt-get install -y dphys-swapfile
        # Now immediately reboot:
        sudo reboot

17. Now is a good time to update your system:

        # From you deskop/laptop:
        ssh ubuntu@ubuntu.local
        # Now update and upgrade:
        sudo apt-get update
        sudo apt-get -y upgrade

18. Change the hostname and set up the WiFi:

        cd ~/catkin_ws/src/ubiquity_main
        # Update the code (it changes):
        git pull
        sudo ./configure.py
        # Follow the menu to change the host name an optionally configure
        # the WiFi.
        sudo reboot
        # Note, use the new host name when you log back in.
        ssh ubuntu@new_host_name.local

19. Set your git user name and E-mail address.  (Eventually this will
    be done from `configure.py`):

        # We should modify `configure.py` stuff to do this:
        git config --global user.email "your.email@whatever"
        git config --global user.name "First Last"

    where you fill in the appropriate fields in the quotes.

20. This is where add packages to install that did not make
    it into the latest system image.  Please add the following
    packages:

        sudo apt-get install chrony
        sudo apt-get install ros-indigo-tf2-kdl # ROS Arduino Bridge only
        sudo apt-get install ros-indigo-joy-input


## Constructing the System Image from Scratch

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


