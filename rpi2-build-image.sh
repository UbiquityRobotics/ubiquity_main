#!/bin/sh

# TO DO:
# apt-get bmap_tools and zip [done]
# apt-get curl [done]
# Make sure hostname stuff is fixed [done]
# clone raspicam [done]
# Add if statements around hardware specifics [done]
# Clone ubiquity_launches [done]
# Clone ubiquity_main [done]
# Clone userland [done]
# Download and install rpi-update [done]
# Build userland (do as root first to install) [done]
# chown -r ubuntu userland [done]
# build raspicam [done]
# Run rpi-update [done]
# Update /etc/network/interfaces to support multiple eth's [done]

########################################################################
# rpi2-build-image
# Copyright (C) 2015 Ryan Finnie <ryan@finnie.org>
# Copyright (C) 2015 Wayne Gramlich <wayne@gramlich.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
########################################################################

set -e
set -x

# Verify that we are being run as root:
if [ $EUID -ne 0 ] ; then
  echo "This script must be run as root"
  exit 1
fi

RELEASE=trusty
BASEDIR=/var/rpi2/${RELEASE}
BUILDDIR=${BASEDIR}/build
# I use a local caching proxy to save time/bandwidth; in this mode, the
# local mirror is used to download almost everything, then the standard
# http://ports.ubuntu.com/ is replaced at the end for distribution.
LOCAL_MIRROR="http://10.0.1.105:3142/ports.ubuntu.com/"

# Don't clobber an old build:
if [ -e "$BUILDDIR" ]; then
  echo "$BUILDDIR exists, not proceeding"
  exit 1
fi

# Determine if we are running on an ARM architecture
IS_ARM=false
if [ "`uname -p | sed s/armv7.*/armv7/`" = "armv7" ] ; then IS_ARM=true ; fi

# Set up environment:
export TZ=UTC
R=${BUILDDIR}/chroot
mkdir -p $R

# Base debootstrap:
apt-get update
apt-get -y install ubuntu-keyring debootstrap
if [ -n "$LOCAL_MIRROR" ]; then
  debootstrap $RELEASE $R $LOCAL_MIRROR
else
  debootstrap $RELEASE $R http://ports.ubuntu.com/
fi

# Mount required filesystems:
mount -t proc none $R/proc
mount -t sysfs none $R/sys

# Set up initial sources.list
if [ -n "$LOCAL_MIRROR" ]; then
  cat <<EOM >$R/etc/apt/sources.list
deb ${LOCAL_MIRROR} ${RELEASE} main restricted universe multiverse
# deb-src ${LOCAL_MIRROR} ${RELEASE} main restricted universe multiverse

deb ${LOCAL_MIRROR} ${RELEASE}-updates main restricted universe multiverse
# deb-src ${LOCAL_MIRROR} ${RELEASE}-updates main restricted universe multiverse

deb ${LOCAL_MIRROR} ${RELEASE}-security main restricted universe multiverse
# deb-src ${LOCAL_MIRROR} ${RELEASE}-security main restricted universe multiverse

deb ${LOCAL_MIRROR} ${RELEASE}-backports main restricted universe multiverse
# deb-src ${LOCAL_MIRROR} ${RELEASE}-backports main restricted universe multiverse
EOM
else
  cat <<EOM >$R/etc/apt/sources.list
deb http://ports.ubuntu.com/ ${RELEASE} main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE} main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-updates main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-updates main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-security main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-security main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-backports main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-backports main restricted universe multiverse
EOM
fi
chroot $R apt-get update
chroot $R apt-get -y -u dist-upgrade

# Install the RPi PPA:
chroot $R apt-get -y install software-properties-common ubuntu-keyring
chroot $R apt-add-repository -y ppa:fo0bar/rpi2
chroot $R apt-get update

# Standard packages:
chroot $R apt-get -y install ubuntu-standard initramfs-tools raspberrypi-bootloader-nokernel language-pack-en

# Add packages to enable "zeroconf" and the secure shell server:
chroot $R apt-get -y install libnss-mdns openssh-server

# Install ROS packages:
chroot $R update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
echo "deb http://10.0.1.105:3142/packages.ros.org/ros/ubuntu trusty main" > $R/etc/apt/sources.list.d/ros-latest.list
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | chroot $R apt-key add -
chroot $R apt-get update
chroot $R apt-get -y --force-yes install ros-indigo-ros-base

# Install some more ROS stuff:
chroot $R apt-get install -y --force-yes  \
  bmap-tools          \
  build-essential       \
  curl            \
  devscripts          \
  equivs          \
  emacs           \
  fakeroot          \
  gdebi-core          \
  joystick          \
  mgetty          \
  minicom         \
  python-bloom          \
  python-serial         \
  ros-indigo-compressed-image-transport   \
  ros-indigo-joy        \
  ros-indigo-joystick-drivers     \
  ros-indigo-joystick-drivers     \
  ros-indigo-navigation       \
  ros-indigo-robot-model      \
  ros-indigo-ros-tutorials      \
  ros-indigo-serial       \
  ros-indigo-teleop-twist-joy     \
  ros-indigo-tf-conversions     \
  ros-indigo-tf2-geometry-msgs      \
  ros-indigo-turtlebot-teleop     \
  ros-indigo-xacro        \
  ros-indigo-yocs-velocity-smoother   \
  setserial         \
  vim           \
  wireless-tools        \
  wpasupplicant         \
  zip

chroot $R apt-get install -y --force-yes network-manager python-networkmanager

# Install some more ROS stuff:
chroot $R apt-get -y --force-yes install python-rosdep

# Kernel installation:
# Install flash-kernel last so it doesn't try (and fail) to detect the
# platform in the chroot:
chroot $R apt-get -y --no-install-recommends install linux-image-rpi2
chroot $R apt-get -y install flash-kernel
VMLINUZ="$(ls -1 $R/boot/vmlinuz-* | sort | tail -n 1)"
[ -z "$VMLINUZ" ] && exit 1
cp $VMLINUZ $R/boot/firmware/kernel7.img

# Set up fstab:
cat <<EOM >$R/etc/fstab
proc            /proc           proc    defaults          0       0
/dev/mmcblk0p2  /               ext4    defaults,noatime  0       1
/dev/mmcblk0p1  /boot/firmware  vfat    defaults          0       2
EOM

# Set up hosts:
echo ubuntu >$R/etc/hostname
cat <<EOM >$R/etc/hosts
127.0.0.1       localhost
127.0.0.1       ubuntu
127.0.0.1       ubuntu.local
::1             localhost ip6-localhost ip6-loopback
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters
EOM

# Set up default user:
chroot $R adduser --gecos "Ubuntu user" --add_extra_groups --disabled-password ubuntu
chroot $R usermod -a -G sudo,adm,netdev -p '$6$iTPEdlv4$HSmYhiw2FmvQfueq32X30NqsYKpGDoTAUV2mzmHEgP/1B7rV3vfsjZKnAWn6M2d.V2UsPuZ2nWHg1iqzIu/nF/' ubuntu

# Install GUI
chroot $R apt-get -y install lubuntu-desktop
chroot $R apt-get -y install xterm

# Restore standard sources.list if a local mirror was used:
if [ -n "$LOCAL_MIRROR" ]; then
  cat <<EOM >$R/etc/apt/sources.list
deb http://ports.ubuntu.com/ ${RELEASE} main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE} main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-updates main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-updates main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-security main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-security main restricted universe multiverse

deb http://ports.ubuntu.com/ ${RELEASE}-backports main restricted universe multiverse
# deb-src http://ports.ubuntu.com/ ${RELEASE}-backports main restricted universe multiverse
EOM
echo "deb http://packages.ros.org/ros/ubuntu trusty main" > $R/etc/apt/sources.list.d/ros-latest.list
chroot $R apt-get update
fi

# Clean cached downloads:
chroot $R apt-get clean

# Set up interfaces:
cat <<EOM >$R/etc/network/interfaces
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

# The loopback network interface:
auto lo
iface lo inet loopback

# The primary network interface(s):
allow-hotplug eth0
iface eth0 inet dhcp
allow-hotplug eth1
iface eth1 inet dhcp
allow-hotplug eth2
iface eth2 inet dhcp
allow-hotplug eth3
iface eth3 inet dhcp
EOM

# Set up firmware config:
cat <<EOM >$R/boot/firmware/config.txt
# For more options and information see 
# http://www.raspberrypi.org/documentation/configuration/config-txt.md
# Some settings may impact device functionality. See link above for details

# uncomment if you get no picture on HDMI for a default "safe" mode
#hdmi_safe=1

# uncomment this if your display has a black border of unused pixels visible
# and your display can output without overscan
#disable_overscan=1

# uncomment the following to adjust overscan. Use positive numbers if console
# goes off screen, and negative if there is too much border
#overscan_left=16
#overscan_right=16
#overscan_top=16
#overscan_bottom=16

# uncomment to force a console size. By default it will be display's size minus
# overscan.
#framebuffer_width=1280
#framebuffer_height=720

# uncomment if hdmi display is not detected and composite is being output
#hdmi_force_hotplug=1

# uncomment to force a specific HDMI mode (this will force VGA)
#hdmi_group=1
#hdmi_mode=1

# uncomment to force a HDMI mode rather than DVI. This can make audio work in
# DMT (computer monitor) modes
#hdmi_drive=2

# uncomment to increase signal to HDMI, if you have interference, blanking, or
# no display
#config_hdmi_boost=4

# uncomment for composite PAL
#sdtv_mode=2

#uncomment to overclock the arm. 700 MHz is the default.
#arm_freq=800

# Enable camera port
start_file=start_x.elf
fixup_file=fixup_x.dat
start_x=1
gpu_mem=128
EOM

ln -sf firmware/config.txt $R/boot/config.txt
echo 'dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootwait' > $R/boot/firmware/cmdline.txt
ln -sf firmware/cmdline.txt $R/boot/cmdline.txt

# Why are we doing this?
# Load sound module on boot:
cat <<EOM >$R/lib/modules-load.d/rpi2.conf
snd_bcm2835
bcm2708_rng
EOM

# If we don't load sound, we don' need to do this:
# Blacklist platform modules not applicable to the RPi2:
cat <<EOM >$R/etc/modprobe.d/rpi2.conf
blacklist snd_soc_pcm512x_i2c
blacklist snd_soc_pcm512x
blacklist snd_soc_tas5713
blacklist snd_soc_wm8804
EOM

# Fix up `/etc/init/failsafe.conf` to disable a couple of sleeps:
sed -i "s/sleep 40/#sleep 40/" $R/etc/init/failsafe.conf
sed -i "s/sleep 59/#sleep 59/" $R/etc/init/failsafe.conf

# Insert `stty -F /dev/ttyAMA0 115200` at the beginning of `/etc/rc.local`:
sed -i "1istty -F /dev/ttyAMA0 115200" $R/etc/rc.local 

# Install the 8188 keneral module (* will expand to the corret directory name):
chroot $R ln -s /lib/modules/*/kernel/drivers/staging/rtl8188eu/rtl8188eu.ko /lib/modules/*/rtl8188eu.ko

chroot $R rosdep init
# Execute `rosdep update` as user `ubuntu`:
chroot $R su ubuntu -c "rosdep update"

# Set up the ROS enviroment for user `ubuntu`:
cat bashrc_tail >> $R/home/ubuntu/.bashrc

# Create the raspberry pi camera module load script:
chroot $R mkdir -p /etc/modules-load.d
echo 'bcm2835-v4l2 gst_v4l2src_is_broken=1' > $R/etc/modules-load.d/raspi-camera.conf

# Add Ubiquity repository:
echo "deb http://packages.ubiquityrobotics.com/ trusty main" > $R/etc/apt/sources.list.d/ubiquityrobotics-latest.list
# It would be nice if we did a wget to get the B5A652C1...
chroot $R apt-key adv --keyserver keyserver.ubuntu.com --recv-keys B5A652C1
chroot $R apt-get update

# Finish making the Rasperry Pi camera work:
chroot $R apt-get install -y --force-yes linux-firmware

# Remove persistent net rules from UDev:
chroot $R rm -f /etc/udev/rules.d/70-persistent-net.rules

# Make sure that `/dev/vchiq` is accessible to user `ubuntu`:
echo 'SUBSYSTEM=="vchiq",GROUP="video",MODE="0660"' > $R/etc/udev/rules.d/10-vchiq-permissions.rules
chroot $R usermod -a -G video `whoami`

cat <<EOM >$R/etc/polkit-1/localauthority/50-local.d/nm.pkla
[network-manager]
Identity=unix-group:netdev
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=no
ResultActive=yes
EOM
        
# Build the catkin workspace, grab some repositories and build them:
chroot $R su ubuntu -c "mkdir -p ~/catkin_ws/src"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/DLu/navigation_layers.git)"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/UbiquityRobotics/raspicam_node.git)"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/UbiquityRobotics/ubiquity_launches.git)"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/UbiquityRobotics/ubiquity_main.git)"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/UbiquityRobotics/bus_server.git)"
chroot $R su ubuntu -c "(cd ~/catkin_ws/src ; git clone https://github.com/raspberrypi/userland.git)"

# ARM specific stuff:
if $IS_ARM ; then								\
  # Compile and install userland as super user:					\
  chroot $R su root -c "(cd /home/ubuntu/catkin_ws/src/userland; ./buildme)" ;	\
  # Now change ownership and group to be `ubuntu`:   				\
  chown -R ubuntu $R/home/ubuntu/catkin_ws/src/userland ;			\
  chgrp -R ubuntu $R/home/ubuntu/catkin_ws/src/userland ;			\
  	   	  								\
  # Now build everything: ;			      				\
  #chroot $R su ubuntu -c "(cd ~/catkin_ws ; /opt/ros/indigo/bin/catkin_make)" ;	\
  	       	      	      		    				        \
  # Install rpi-update and update the Raspberry Pi firmware: ;		      	\
  chroot $R sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update ; \
  chroot $R sudo chmod +x /usr/bin/rpi-update ;	      				\
  chroot $R sudo /usr/bin/rpi-update ;			      			\
fi

# Clean cached downloads:
chroot $R apt-get clean

# Unmount mounted filesystems (rsyslog must be halted to do this):
echo "build current fails on umount; reboot and run clean-up.sh to finish build"
umount $R/proc
umount $R/sys

