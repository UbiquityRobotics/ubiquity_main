# 2016 Tasks

## DARPA Contract

*  to use garage bathroom for optics lab

* Assemble parts list

* Do figures for proposal

## Software Development

* We need to be able to export the robot catkin workspace as
  a samba mount.  Likewise, we need to figure out how to mount
  the samba mount on the laptop/desktop.

* We need to figure out how to enable winpdb to allow a laptop/desktop
  to remotely debug ROS nodes running on the robot.  Likewise,
  we need to do the same for gdb debugging C/C++ ROS nodes.
  The ROS launch file magic needs to be supported by the launch
  file infrastructure.

* RasPi2 LXDE:
  Get everybody to agree to install lubuntu (i.e. LXDE)
  onto the RasPi2 system image.  I'm happy with requiring
  the user to either log into a special account to trigger
  the window system (or type startx).

* Construct a virtual box image that can be loaded onto
  Windows (which versions?), MacOS, and Linux (in particular
  Red Hat.)  The image would uses lubuntu instead of ubiquity
  to be compatible with the RasPi2 system image.

* Refactor launch files to be platform neutral.

* Bring up Joe's speech stuff launch files.

* Package Ubiquity projects to build binaries that can be
  distributed via Ubiquity server.

* Modifiy configure.py to allow for the creation of users
  with passwords.  Make sure that we document the process
  of setting up the keyring so that ssh/scp work.

## WiFi

* Bring Up WiFi network with 3 dual band routers

* Get WiFi roaming to work.

* Wi Fi management.  Network manager has been a total pain
  in the tush because it does not allow the installation of
  a wifi access point unless it is within signal range.
  Rohan may have found a
  [work-around](http://askubuntu.com/questions/675771/add-multiple-wifi-networks-on-command-line).
  It looks like the passwords can be hashed using `wpa_passphrase`.

* Build a list of Ubiquity team member SSID/PassPhrases to
  preinstall.

## Robots

### ROSberry Pi Bot (RosPiBot)

* Create a "robot" that can demo fiducial localization
  using a RasPi2 and camera.

### BotVac

* Create the outline for BotVac that will hold the RasPi2
  and camera.

* Get move base working.

### Loki

* Install batteries.

* Manufacture and install arms.

* Deploy Loki's.

* Get move base working.

## Server

* Bring up both an 64bit x86 PPA and a 32-bit ARM PPA.

## Marketing

* Build Makerfaire Booth

* Ubiquity/ROS Videos



