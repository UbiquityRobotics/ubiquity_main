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

You are in.

     
