# Synchronizing time with NTP

NTP can be used to synchronize the time between machines, which is important if sensor data is sent from one to the other.

Install ntp on both machines:

    $ sudo apt install ntp

If the machines are on a private network, a local clock must be added to the 'master' machine (the one with the best clock).
On the master machine, insert the following into /etc/ntp.conf:

    server  127.127.1.0     # local clock
    fudge   127.127.1.0 stratum 10

The other machine can either use:

    $ sudo ntpdate master

Where master is the hostname or IP address of the master machine.  Note that `ntpdate` will fail if ntp is running on the local machine.
Alternatively,  include the following into /etc/ntp.conf:

    server master

And run ntp.
