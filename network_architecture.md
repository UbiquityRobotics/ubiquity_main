# Ubiquity Network Architecture

We have a bunch of major issues to wrestle to the ground:

* Each robot needs a unique hostname so that zeroconf can
  give each robot a unique IP address.

* Use some sort of `avahi-browse` functionality to find all robots.

* Allow user to switch easily between robots.

* Each robot needs to be able to log into a bunch of different
  WiFi access points.

* Key management.  SSH.  WiFi.

* Time synchroniziation.  (What about mutliple robots?)

* Wrestle the WiFi dongle issues to the ground.

* Make sure we work with multiple SSID's and single replicated SSID's.

* Need dongle that supports WiFi client and access point on RasPi2

* How do we do multi-robot applications?

