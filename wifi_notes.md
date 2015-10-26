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


