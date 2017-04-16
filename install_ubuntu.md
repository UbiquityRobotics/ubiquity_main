# Install Ubuntu 16.04LTS

{This discussion needs to go somewhere, but not here.)
Now comes the Ubunutu desktop environments discussion.  Ubuntu supports
multiple desktop environments.  (By the way, this is actually pretty cool.)
The default desktop environment that ships with Ubuntu 16.04LTS is called
Unity.  All examples in this document for the host Ubuntu desktop will
be done using Unity.  However, if you prefer running a different
Ubuntu desktop environment on the host computer you are welcome to do
so.  It is strongly recommended that you choose one with full graphics
acceleration, since several of the ROS development tools use graphics
acceleration (particularly, RViz).  The Ubuntu desktop environments that
support graphics acceleration are Unity, Gnome 3, Sinnamon, KDE, and LXQt.
Ubuntu Mate can support graphics acceleration.


Every once in a while, something goes a little screwy on your
mobile robot and you put up on blocks on your bench (making it
non-mobile).  In this situation it is safe to plug an HDMI display,
USB keyboard, and UBS mouse into the processor board on UR robot.
When you reboot the processor, the display, keyboard, and mouse will
be detected and the 



If you are so inclined, you can install multiple desktop environments
on the same machine if it has adequate hardware.  However, only one
desktop at a time can be run.  When you login, can select which desktop
you want to run.  Sometimes when 

There actually multiple desktop environments available for Ubuntu.
There are three that will be mentioned here:

* LXDE: This is called a minimal desktop and works well on small
  memory limited processors like the Raspberry Pi 3.

* Unity: Until recently, this desktop was heavily developed by Canonical.
  Development has essentially ceased and Canonical will no longer releasing
  new versions of it.

* Gnome3: This environment has been around quite a while and will deployed
  for future releases of Ubuntu.

The Ubuntu 16.04LTS release ships with Unity by default.
However, it will be the last LTS release that ships with Unity.  
