#!/bin/sh

set -e
set -x

RELEASE=trusty
BASEDIR=/srv/rpi2/${RELEASE}
BUILDDIR=${BASEDIR}/build
R=${BUILDDIR}/chroot

# Verify that we are being run as root:
if [ $EUID -ne 0 ] ; then
  echo "This script must be run as root"
  exit 1
fi

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
# New New image is      4.00GiB: (Note 4096 = 4.00*1024; also 8388608=4096 * 2048)
DATE="$(date +%Y-%m-%d)"
dd if=/dev/zero of="$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" bs=1M count=1
dd if=/dev/zero of="$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" bs=1M count=0 seek=4096
sfdisk -f "$BASEDIR/${DATE}-ubuntu-${RELEASE}.img" <<EOM
unit: sectors

1 : start=     2048, size=   131072, Id= c, bootable
2 : start=   133120, size=  8388608, Id=83
3 : start=        0, size=        0, Id= 0
4 : start=        0, size=        0, Id= 0
EOM
VFAT_LOOP="$(losetup -o 1M --sizelimit 64M -f --show $BASEDIR/${DATE}-ubuntu-${RELEASE}.img)"
EXT4_LOOP="$(losetup -o 65M --sizelimit 4096M -f --show $BASEDIR/${DATE}-ubuntu-${RELEASE}.img)"
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
