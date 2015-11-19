

The file in /etc/init  ttyUSB1.conf   --- this file on host robot pc Rpi it 
might be some other USB#  usually USB0 is the motor controller.

# ttyUSB1 (zigbee) - getty
#
# This service maintains a getty on ttyUSB1 from the point the system is
# started until it is shut down again.

start on stopped rc RUNLEVEL=[2345] and (
           not-container or
           container CONTAINER=lxc or
           container CONTAINER=lxc-libvirt)
stop on runlevel [!2345]

respawn exec /sbin/getty -8 9600 ttyUSB1

#end of file


On remote computer run minicom:


       minicom -D /dev/ttyUSB0 -b 9600 -8




------  That's all folks! ----
