# ubiquity-misc

Miscellaneous Stuff Used by the Ubiquity Team

## `rpi2-build-image.sh`

This shell script is run as root to build the Ubuntu 14.04LTS
kernel image that is used by the Ubiquity team.  This shell
script needs to be run on an Raspberry Pi 2 (hereafter shorted
to RasPi2.)  This script can be run under any RasPi2 kernel
that supports `apt-get`.  Thus, you can go to the
  [Raspberry Pi Downloads](https://www.raspberrypi.org/downloads/)
page and load Raspian on to a micro-SD card.  Once Raspian
is running, you can run this script and it will generate a
Ubuntu 14.04LTS kernel image that runs on a RasPi2.

The command is run as follows:

        sudo .../rpi2-build-image.sh

where `.../` is the directory that contains `rpi2-build-image.sh`.

