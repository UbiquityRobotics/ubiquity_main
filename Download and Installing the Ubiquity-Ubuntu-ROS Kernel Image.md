### Download and Installing the Ubiquity-Ubuntu-ROS Kernel Image

The Ubuquity/Ubuntu/ROS Kernel Image is a Ubuntu 14.04LTS
Kernel for the ARM7 hard float architecture.  This image
has many Ubuntu and ROS packages preinstalled along with
a ROS catkin workspace.  The primary purpose of this image
is to ensure that all of the Ubiquity robot platforms start
from a common software base.

This kernel image only works for Ubuntu 14.04LTS
(i.e. "Trusty").


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
		# Then make the system by typing the following line, with the parentheses
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

33. Pull in xacro for magni_robot tools

        sudo apt-get install ros-indigo-xacro

