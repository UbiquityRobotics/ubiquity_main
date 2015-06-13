Robot Network Configuration Functional Specification
===

##System Requirements

##Functional requirements 
####Overview
This document  describes how we will provide ubiquity robotics customers, the ability communicate with a robot and integrate it into the customer's LAN.

####Initial Program Load State Flow

The robot needs to work in two distinct and separate environments:

 1. Within a known local area network such as at home or in the office.
 2. Not within a known local area network.

When the robot cannot find a previously known LAN, it MUST fail into running its own wireless ethernet connection as an access point to its own network.  

> The default factory, out-of-the-box, configuration will **NOT** know of any local area networks.

The configuration utility must provide the ability to configure the robot to convert into "access point" mode even if it sees a known network with autoconnect information.

> Utility will be able to make the mode setting persistent across poweroff/restart of the robot's operating system.

```flow
st=>start: robot initial program load
op_ap=>operation: Access Point
op_wc=>operation: Wireless Client
op_find=>operation: find autoconnect network
cond_find=>condition: Found?
cond_mod=>condition: Client Mode? 
vs. access point
cond_connect=>condition: Success?
sub_ap=>subroutine: Become Access Point
sub_cl=>subroutine: Become Network Client
e=>end: run

st->cond_mod
cond_mod(yes)->op_find->cond_find
cond_mod(no)->sub_ap->e
cond_find(yes)->sub_cl->cond_connect
cond_find(no)->sub_ap->e
cond_connect(yes)->e
cond_connect(no)->sub_ap
```

###Factory default state

Ubiquity robots, out of the box, will ship with ubuntu 14.04 based on Wayne's ubiquity-misc script.  The OS will be configured with the following parameters affecting networking capabilities of the robot.

Parameter/Variable | Allowed Values | Default Value
--- | ---
hostname | [63 bytes text](https://tools.ietf.org/html/rfc1123#page-13) | "robot"
Access Mode | access point \|\| client | client
Service Set Identifier | up to 32 bytes | "robot"
Hide Service Set Identifier | true \|\| false | false
Network Passphrase | string | "change_on_install"

Open issue:  We can constrain host names such that we might simplify the UI and directly map the name to the SSID[^1]?  While simplifying the UI, this might also have unintended consequences if someone manually reconfigures the robot instead of going through the configurator.

####hostapd support
We need to add hostapd to the base OS creation script.  The 'hostapd' daemon operates an access point and authentication set of services.  It gives the base OS the capability of publishing an SSID as it's own network access point. 

####dhcp server support
We need to add isc-dhcp-server to the base OS creation script.  

> Written with [StackEdit](https://stackedit.io/).

***

######Footnotes
[^1]: SSID values are constrained to 32 octets.  Substantially less memory than hostname.  SSID directly support unicode while DNS systems require [punycode](https://tools.ietf.org/html/rfc3492) conversions for hostnames.

