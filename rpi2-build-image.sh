#!/bin/sh

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
BASEDIR=/srv/rpi2/${RELEASE}
BUILDDIR=${BASEDIR}/build
# I use a local caching proxy to save time/bandwidth; in this mode, the
# local mirror is used to download almost everything, then the standard
# http://ports.ubuntu.com/ is replaced at the end for distribution.
#LOCAL_MIRROR=""

# Don't clobber an old build
if [ -e "$BUILDDIR" ]; then
  echo "$BUILDDIR exists, not proceeding"
  exit 1
fi

# Set up environment
export TZ=UTC
R=${BUILDDIR}/chroot
mkdir -p $R

# Base debootstrap
apt-get -y install ubuntu-keyring debootstrap
if [ -n "$LOCAL_MIRROR" ]; then
  debootstrap $RELEASE $R $LOCAL_MIRROR
else
  debootstrap $RELEASE $R http://ports.ubuntu.com/
fi

# Mount required filesystems
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

# Install the RPi PPA
chroot $R apt-get -y install software-properties-common ubuntu-keyring
chroot $R apt-add-repository -y ppa:fo0bar/rpi2
chroot $R apt-get update

# Standard packages
chroot $R apt-get -y install ubuntu-standard initramfs-tools raspberrypi-bootloader-nokernel language-pack-en

# Add packages to enable "zeroconf" and the secures shell server:
chroot $R apt-get -y install libnss-mdns openssh-server

# Install ROS packages:
chroot $R update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
echo "deb http://packages.ros.org/ros/ubuntu trusty main" > $R/etc/apt/sources.list.d/ros-latest.list
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | chroot $R apt-key add -
chroot $R apt-get update
chroot $R apt-get -y install ros-indigo-ros-base

# Kernel installation
# Install flash-kernel last so it doesn't try (and fail) to detect the
# platform in the chroot.
chroot $R apt-get -y --no-install-recommends install linux-image-rpi2
chroot $R apt-get -y install flash-kernel
VMLINUZ="$(ls -1 $R/boot/vmlinuz-* | sort | tail -n 1)"
[ -z "$VMLINUZ" ] && exit 1
cp $VMLINUZ $R/boot/firmware/kernel7.img

# Set up fstab
cat <<EOM >$R/etc/fstab
proc            /proc           proc    defaults          0       0
/dev/mmcblk0p2  /               ext4    defaults,noatime  0       1
/dev/mmcblk0p1  /boot/firmware  vfat    defaults          0       2
EOM

# Set up hosts
echo ubuntu >$R/etc/hostname
cat <<EOM >$R/etc/hosts
127.0.0.1       localhost
::1             localhost ip6-localhost ip6-loopback
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters

127.0.1.1       ubuntu
EOM

# Set up default user
chroot $R adduser --gecos "Ubuntu user" --add_extra_groups --disabled-password ubuntu
chroot $R usermod -a -G sudo,adm -p '$6$iTPEdlv4$HSmYhiw2FmvQfueq32X30NqsYKpGDoTAUV2mzmHEgP/1B7rV3vfsjZKnAWn6M2d.V2UsPuZ2nWHg1iqzIu/nF/' ubuntu

# Restore standard sources.list if a local mirror was used
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
chroot $R apt-get update
fi

# Clean cached downloads
chroot $R apt-get clean

# Set up interfaces
cat <<EOM >$R/etc/network/interfaces
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

# The loopback network interface
auto lo
iface lo inet loopback

# The primary network interface
allow-hotplug eth0
iface eth0 inet dhcp
EOM

# Set up firmware config
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
start_x=1
gpu_mem=128
EOM
ln -sf firmware/config.txt $R/boot/config.txt
echo 'dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootwait' > $R/boot/firmware/cmdline.txt
ln -sf firmware/cmdline.txt $R/boot/cmdline.txt

# Load sound module on boot
cat <<EOM >$R/lib/modules-load.d/rpi2.conf
snd_bcm2835
bcm2708_rng
EOM

# Blacklist platform modules not applicable to the RPi2
cat <<EOM >$R/etc/modprobe.d/rpi2.conf
blacklist snd_soc_pcm512x_i2c
blacklist snd_soc_pcm512x
blacklist snd_soc_tas5713
blacklist snd_soc_wm8804
EOM

# Do some more ROS installation stuff:
chroot $R apt-get -y install python-rosdep
chroot $R rosdep init
# Execute `rosdep update` as user `ubuntu`:
chroot $R su ubuntu -c "rosdep update"
# Set up the ros enviroment for user `ubuntu`:
chroot $R su ubuntu -c "echo source /opt/ros/indigo/setup.bash >> ~/.bashrc"
chroot $R apt-get -y install python-rosinstall

# Create the raspberry pi camera module load script:
chroot $R mkdir -p /etc/modules-load.d
echo 'bcm2835-v4l2 gst_v4l2src_is_broken=1' > $R/etc/modules-load.d/raspi-camera.conf

# Add Ubiquity repository:
echo "deb http://packages.ubiquityrobotics.com/ v4 main" > $R/etc/apt/sources.list.d/ubiquityrobotics-latest.list
chroot $R apt-key adv --keyserver keyserver.ubuntu.com --recv-keys B2FA8835
chroot $R apt-get update

# Finish making the Rasperry Pi camera work:
chroot $R apt-get install -y ubiquity-indigo-gscam linux-firmware

# Unmount mounted filesystems (rsyslog must be halted to do this):
echo "build current fails on umount; reboot and run clean-up.sh to finish build"
umount $R/proc
umount $R/sys

# Clean up files
rm -f $R/etc/apt/sources.list.save
rm -f $R/etc/resolvconf/resolv.conf.d/original
rm -rf $R/run
mkdir -p $R/run
rm -f $R/etc/*-
rm -f $R/root/.bash_history
rm -rf $R/tmp/*
rm -f $R/var/lib/urandom/random-seed
[ -L $R/var/lib/dbus/machine-id ] || rm -f $R/var/lib/dbus/machine-id
rm -f $R/etc/machine-id

# Build the image file
# Initial image was 1.75GiB: (Note 1792 = 1.75*1024; also 3536896=1792 * 2048)
# New image is      3.00GiB: (Note 3072 = 3.00*1024; also 6291456=3072 * 2048)
DATE="$(date +%Y-%m-%d)"
dd if=/dev/zero of="$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" bs=1M count=1
dd if=/dev/zero of="$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" bs=1M count=0 seek=3072
sfdisk -f "$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" <<EOM
unit: sectors

1 : start=     2048, size=   131072, Id= c, bootable
2 : start=   133120, size=  6291456, Id=83
3 : start=        0, size=        0, Id= 0
4 : start=        0, size=        0, Id= 0
EOM
VFAT_LOOP="$(losetup -o 1M --sizelimit 64M -f --show $BASEDIR/${DATE}-ubuntu-${RELEASE}.img)"
EXT4_LOOP="$(losetup -o 65M --sizelimit 3072M -f --show $BASEDIR/${DATE}-ubuntu-${RELEASE}.img)"
mkfs.vfat "$VFAT_LOOP"
mkfs.ext4 "$EXT4_LOOP"
MOUNTDIR="$BUILDDIR/mount"
mkdir -p "$MOUNTDIR"
mount "$EXT4_LOOP" "$MOUNTDIR"
mkdir -p "$MOUNTDIR/boot/firmware"
mount "$VFAT_LOOP" "$MOUNTDIR/boot/firmware"
rsync -a "$R/" "$MOUNTDIR/"
umount "$MOUNTDIR/boot/firmware"
umount "$MOUNTDIR"
losetup -d "$EXT4_LOOP"
losetup -d "$VFAT_LOOP"

# Always use bmaptool to save disk space:
apt-get -y install bmap-tools
bmaptool create -o "$BASEDIR/${DATE}-ubuntu-${RELEASE}.bmap" "$BASEDIR/${DATE}-ubuntu-${RELEASE}.img"

# Done!