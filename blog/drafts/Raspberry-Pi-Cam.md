---
layout: post
title: Raspberry Pi Camera on Ubuntu
author: Wayne Gramlich
---

Frankly, there was an element of luck required to figure out
how to get the Raspberry Pi camera working under Ubuntu.
The reason why is described below.

The raspicam node uses the Raspbery Pi video camera interface
API (Application Programming Interface) to access the image data.
The Raspberry Pi camera is actually connected to the GPU (Graphics
Processing Unit) where specialized graphics processing units
can manipulate the image before it is forwarded on to the
main ARM7 cores.  The Raspberry Pi GPU is closed source
proprietary code is only shipped as a binary blob that is
loaded into the GPU at processor boot up time.

In general, the Raspberry Pi camera is fully supported by
the Raspberry Pi foundation.  However, in order to support
their older products (the model A, model B, and model B+),
they have had to develop (with user help) their own Linux
distribution called Raspian.  The Raspberry Pi camera is
fully supported by the Raspian Linux distribution.
ROS is only supported by the Ubuntu Linux distribution.
Since the `raspicam_node` is for ROS, it must be run on
a Ubuntu Linux distribution.  While the Ubuntu Linux distribution
is not fully supported by the Raspberry Pi Foundation,
the Raspberry Pi foundation is actually OK with the Ubuntu
Linux distribution and does provide some limited support.

Both the Ubuntu and Raspian distributions use the same
packaging system called Debian Packages.  To make things
confusing, the Raspian Linux distribution is based on the
Debian Linux distribution.  While the debian package formats
are the same between both the Ubuntu and Raspian Linux
distributions, the underlaying Debian packages are *NOT*
100% inter-operable.  Sometimes the Ubuntu Linux distribution
installs files in a different location than the Raspian
distribution does.  This is great fun.

There are two ways that firmware is provided to the Raspberry Pi.
There is a package called `raspberrypi-bootloader-nokernel` which
provides the binary blob *and* there is a program called `rpi-update`
that can do so as well.  The bootloader stuff tends to be older
than the `rpi-update` method.

After compiling `userland` using the `buildme` script, the
`raspicam_node` code compiles without errors.  When the
`raspicam_node` was first run it failed.  After running the
`rpi-update` program it worked for a while.  As other people
tryed to install `raspicam_node`, it simply did not work.
After much sleuthing it became clear that the Raspberry Pi
foundation had accidentally checked in a firmware version that
did not work.

Our temporary solution is to use slightly older firmware,
until whatever problem that caused `raspicam_node` to not work,
to start working again.

It was a small miracle is that there was a small window were
we figured out the firmware worked, and we managed to find that
window.  Once we figured out that it was firmware, we were on
solid ground to finding a work around. Initially we used the
command below:

        sudo rpi-update cad980c560b6c240fdaf6cb4b7703921b18114e3

to install a specific firmware that worked.  These days, the
latest released firmware version is used, because it works.
So, we no longer explicitly specify the firmware version.

If you are curious about firmware revision numbers, there is a
[post](http://raspberrypi.stackexchange.com/questions/29991/how-do-i-find-the-firmware-repository-commit-which-matches-the-firmware-version)
about them.
