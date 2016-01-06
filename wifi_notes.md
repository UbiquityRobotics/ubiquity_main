# Ubiquity_Main

WiFi Material

### Installing USB WiFi dongles:

The are two common USB WiFi dongles.  One is based on the Realtek 8192
chip set and the other is based on the Realtek 8188 chips set.  The 8192
seems to work rather well, but the 8188 seems to cause more problems.

1. Remove the the persistent rules UDev rule:

        # This has already been done in the latest kernel image:
        sudo rm /etc/udev/rules.d/70-persistent-net.rules

2. Install the 8188EU kernel module:

        # This has already been done in the latest kernel image:
        cd /lib/modules/`uname -r`
        sudo ln -s kernel/drivers/staging/rtl8188eu/rtl8188eu.ko
        sudo modprobe r8188eu

3. Plug in your WiFi dongle and figure which kind you got:

        lsusb | grep -i realtek

   The 8188 gives:

        Bus 001 Device 004: ID 0bda:8179 Realtek Semiconductor Corp.

   and the 8192 gives:

        Bus 001 Device 004: ID 0bda:8178 Realtek Semiconductor Corp. RTL8192CU 802.11n WLAN Adapter

   You may get something else.

4. Now create/edit `/etc/network/interfaces` and add the following
   lines to your file:

        # WiFi dongle
        allow-hotplug wlan0
        auto wlan0
        iface wlan0 inet dhcp
          wpa-ssid "SSID"     # Replace SSS with your network SSID
          wpa-psk  "PWD"      # Replace PWD with your network password

5. Reboot.

        sudo reboot

6. Run `ifconfig` to see if device `wlan0` shows up:

        ifconfig

   If `wlan0` does not show up, something did not go right.  Feel the pain.


## Older Stuff

The stuff below is getting old and probably needs to be deleted...

## WiFi Networking Issues

The problem is linux saves hard lan and wifi dongle macs and keeps
adding devices so you end up with say `eth2` but your
`/etc/network/interfaces` only has entry for `eth0`.

Here are some notes for sorting this out.  Tthe last half of this
set of cheet-sheet notes.


[Linux Network Specific]( https://help.ubuntu.com/lts/serverguide/network-configuration.html)

* `ifconfig   [eth0]`:
  Shows network interface(s) that have been configured or
  maybe partly configured (like WiFi that cannot find network)

* `sudo ifconfig eth0 10.0.0.100 netmask 255.255.255.0` :
  You can temp configure one till reboot.

* `/etc/hostname`:
  This is where system hostname is defined on reboot.
  Also edit `/etc/hosts` if you do change host name!

* `/etc/network/interfaces`:
  Entries tie hardware like `eth0` and `wlan0` to be of a specific
  network port type. KEY file, see other places in doc.

* `/etc/resolv.conf`:
  Holds DNS resolutions (ip addresses).  Best to use tool to do this
  `dmesg | grep eth` Can show if eth0 or wlan0 was re-named due to
  new flash image.  Come up with no net and edit `/etc/network/interfaces`

* `/etc/udev/rules.d/70-persistent-net.rules`:
  Edit this if you have left over `eth0` or `wlan0` from prior image
  so you get nice clean `eth0` and `wlan0`.
  This file has mac address to names like `wlan0`, `wlan1` and `eth0`
  and so on. Clean it out to have fresh discovery.

  (Random commnet: The `/etc/udev/rules.d/70-persistent-net.rules` is
  generated from `/lib/udev/rules.d/75-persistent-net.rules`.  To
  disble the generation of `70-persistent-net.rules`, the following
  will do the trick:

        sudo echo "# disable net-generator rules in /lib/udev" > /etc/udev/rules.d/75-persistent-net-generator.rules

  This works because the rule file in `/etc/udev/rules.d` takes
  precedence over the rule file in `/lib/udev/rules.d`.

* `sudo ifup eth0`:
  Manually enable newly added interface from recent
  `/etc/network/interfaces`.  Can also use `sudo ifdown eth0`.

Sorting out linux images on ubuntu when networks get added:

* Boot up with hdmi monitor and usb keyboard usually (if no
  network no ssh ability)

* `sudo vi /etc/hosts` and `/etc/hostname` and set correct hostname
  for your system.

* `ifconfig` from here we want to see the eth0 and when wifi
  dongle is in `wlan0`.

* To hard-reset things, `sudo vi /etc/udev/rules.d/70-persistent-net.rules`

  1. Remove all the lines below top 2 lines of general comments as it
     is here that you may find your ethernet was nammed `eth2` or
     something and `/etc/network/interfaces` does not match so you
     get no network

  2. `sudo vi /etc/network/interfaces` and have just the use of `eth0`
     and `wlan0`, NOT higher numbers.

* Reboot and again on hdmi monitor.  Do `ifconfig` and hopefully
  you have `eth0` now and if you had wifi and config in
  `/etc/network/interfaces` for `wlan0` you also have wifi now

## Bring 8188 WiFi Dongle Notes:

WiFi with Kurts Kernel working for me today.

One tiny change from your modprobe so when I was in 
/lib/modules/3.18.0-24-rpi2 I used like you did and saw 
he .ko so did this:

        ln -s kernel/drivers/staging/rtl8188eu/rtl8188eu.ko

Difference was I had to then use :

        modprobe r8188eu
        # you had said modprobe rtl8188eu which does not work for me.

After above you should be able to see

        iwconfig  show wlan0

but it will still say unassociated because `/etc/network/interfaces`
not set yet so I then put these EXACT lines at END of
`/etc/network/interfaces` right from Kurt.

        # WiFi dongle
        allow-hotplug wlan0
        auto wlan0
        iface wlan0 inet dhcp
          wpa-ssid "2WIRE270"
          wpa-psk  "d4c33208f8fdc ... from wpa_passphrase 5aa32"

After reboot you should NOT see `iwconfig` showing unassociated
or similar word right after it says `wlan0`:

Also, I do NOT see `/dev/wlan0` (I too am puzzled??)
but with lsmod I see this line matching my modprobe argument:

        r8188eu               408198  0


`lsusb` gives me this:

        Bus 001 Device 004: ID 0bda:8179 Realtek Semiconductor Corp.


### Message From Michael Wimble

(The following message was posted by Micahel Wimble on HBRobotics.)

In writing device drivers on my Pi, such as for the robot motor
controller or using the LIDAR sensor, I have to deal with Linux's
way of naming devices, especially USB devices. If you plug in the
RPLIDAR device, it might be named ttyUSB0 at one time, but if the
system reboots, or if the device fails, is unplugged or otherwise
causes a reconnection sequence to occur, it might be named ttyUSB1.
This makes it difficult to write a driver which can even find the
name of the device to start with. I don't want some person to have
to discover the device name and tell the robot. And I'd rather not
programmatically do the discovery by walking the various kernel
device tree files. And I don't have to.

It's possible to write a couple of lines in a system configuration
file that says, essentially, if a USB (or other) device gets plugged
in, look to see if has a specific vendor ID and device ID (other tests
are possible). If so, beside doing the usual wonky device naming
(ttyUSB0), also create a symbolic to that wonky name, but give the
symbolic name one that is stable. So, when my Roboclaw device gets
into a bad state, I can force it to restart (not what I'm going to
talk about here) and regardless of how Linux would normally name it,
I can always refer to it as "/dev/roboclaw".

If you are trying to deal with a USB device, and already know the
manufacture's ID and device ID for the device, Here's what you do.

In /etc/udev/rules.d, there can be any number of files. The files
are processed in lexical order—so "10-FixupRoboClaw.rules" is processed
before "12-AddMagicalDevices.rules". The files must have a name that
ends in ".rules". The contents of each file is one or more lines that
starts with a test which, if found to be true, causes actions at the
end of the line to be executed. Here is one of my lines I put in the
file "/etc/udev/rules.d/11-usb.rules".

        ATTR{idVendor}=="03eb", ATTR{idProduct}=="2404", SYMLINK+="roboclaw", MODE="660"


Which says that if any device (not just a USB device) is discovered
to have an attribute called "idVendor" with a value of "03eb", and
an attribute called "idProduct" with a value of 2404 (these are the
values for my 2x30A Roboclaw device), then add a soft link (SYMLINK)
called "roboclaw" to the device directory (/dev) and also change the
access privileges to "660", which allows my code to be able to access
the device without requiring root access.

I have another rule in that same file which creates "/dev/lidar" as
a device name for my LIDAR. The LIDAR sometimes gets assigned as
"/dev/ttyUSB0", and sometimes as "/dev/ttyUSB1", but "/dev/lidar"
always names the LIDAR and I can use that name in my program to
reference the LIDAR and not need human discovery or programmatic
code to inform the robot of the device name of the LIDAR.


Note that ALL of the "rules" files are processed and ALL of the
actions are performed when the test succeeds. This is not a situation
where the first rule that matches gets applied and later rules are
ignored. But the actions are performed as ordered by the file name,
and lines within the file, as mentioned above.

Your only problem then is to come up with something that can be
tested to make the rule apply only to the desired device. 

For USB devices, there are several ways to figure this out. The
easiest is to just unplug the device and then plug it back in,
then use "sudo dmeg" to look at the last lines of the system log
which will show something like:


        [  281.023743] usb 1-1.5.4: new full-speed USB device number 17 using dwc_otg
        [  281.147156] usb 1-1.5.4: New USB device found, idVendor=03eb, idProduct=2404
        [  281.147185] usb 1-1.5.4: New USB device strings: Mfr=1, Product=2, SerialNumber=0
        [  281.147203] usb 1-1.5.4: Product: USB Roboclaw 2x30A
        [  281.186577] cdc_acm 1-1.5.4:1.0: ttyACM0: USB ACM device

which shows me that my Roboclaw has an "idVendor" attribute with
a value of "03eb", and an "idProduct" attribute with a value of
"2404". Which is what I used in the example above.

There is a lot more you can do with the "udev" system. And there
are tools to help you debug things when they don't work, albeit
weak tools. My tip is you should look at existing rules files to
see what others have done. Besides the files in "/etc/udev/rules.d",
there are more rules in "/lib/udev/rules.d". Be VERY careful about
the spelling of everything, including whether the plural form or
singular form issued.

For further reading, you might start with:

* [Writing udev rules](http://www.reactivated.net/writing_udev_rules.html)

* [How can I match a ttyUSBX device to a usb serial device](http://unix.stackexchange.com/questions/81754/how-can-i-match-a-ttyusbx-device-to-a-usb-serial-device)

* [Debugging Udev](https://wiki.ubuntu.com/DebuggingUdev)

### Additional Links

* [notes1](http://natisbad.org/dyn-net/index.html)

* [notes2](https://wiki.archlinux.org/index.php/Wireless_network_configuration)

## 5GHz WiFi Dongles

Most USB WiFi Dongles that support 801.11b/g/n only support
2.4GHz.  If you want a USB WiFi Dongle, you need to look for
a "dual-band" dongle.  Eventually, a web page for 
[Installing RTL8812AU](http://blog.danielscrivano.com/installing-rtl8812au-on-li
nux-for-wireless-dual-band-usb-adapters/)
was found.

The instructions when followed on an AMD64 architecture work just fine.
So now the trick is to get them to work on a RasPi2.  Let the **PAIN**
begin!!!

Guess what, the instructions do not work on the RasPi2.  The
Linux header files are wrong.

### Setting the Date and Time:

Make will choke unless the date and time is set reasonably.

The first step is to make sure that you have your RasPi2 plugged
into a network that access the rest of the Internet.

So, we install `ntpdate`:

        sudo apt-get install ntpdate

It is critcal to set the time zone:

        sudo dpkg-reconfigure tzdata

Now restart the ntp server:

        sudo service ntp restart

Now see if you got a new date:

        date

That was pretty tedious...

### Trying to get correct header files

Next reinstall everything as per the
[building RTL8812AU Driver](http://ubuntuforums.org/showthread.php?t=2292112&p=13344716)
information.

After all that, we run make and get:

        make ARCH=armv7l CROSS_COMPILE= -C /lib/modules/3.18.0-25-rpi2/build M=/home/ubuntu/git_downloads/rtl8812AU_8821AU_linux  modules
        make[1]: Entering directory `/usr/src/linux-headers-3.18.0-25-rpi2'
        Makefile:620: arch/armv7l/Makefile: No such file or directory
        make[1]: *** No rule to make target `arch/armv7l/Makefile'.  Stop.
        make[1]: Leaving directory `/usr/src/linux-headers-3.18.0-25-rpi2'
        make: *** [modules] Error 2

Indeed, our header files do not have the `arch/armv7l/Makefile`.
It looks like the header files in the debian repositories are a
tad incomplete.

So off to the
[Raspberry Pi forums](https://www.raspberrypi.org/forums/viewtopic.php?f=71&t=17666).

It looks like the
[RasPi2 ARM7 Kernel](https://wiki.ubuntu.com/ARM/RaspberryPi)
(scroll down to  "Kernel") is a weird mixture of 3.18 plus
some patches.  The documentation for building the kernel is
not present.





