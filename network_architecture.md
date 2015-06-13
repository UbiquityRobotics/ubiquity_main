Ubiquity Network Architecture
=========================
## **Overview** ##
Ubiquity Robots are built to integrate with the [Robot Operating System](http://www.ros.org/about-ros/) (ROS) which is, itself, a peer-to-peer publish/subscribe environment which uses mainly the TCP/IP protocol stack of the operating system to communicate.  

This document is the repository for the communications definitions, requirements, extensions,  agreements, and outstanding open issues for each Ubiquity Robotics robot in support of communicating between ROS nodes.   The current version of this document assumes only support of the ubuntu linux operating system on the robots.  

## **Objectives and Goals** ##

Our goal is to make the out-of-box experience in setting up and initially running an Ubiquity Robotics robot as simple as possible.  Our robots are open robot platforms for people to test, prototype and learn about robotics.  While we expect our customers will have an understanding of TCP/IP communications communications, we will strive to offer robots which can be stood up in 15 minutes or less with 100% success by a relatively lay person with only a minimal understanding of networking.  

####Supported network Protocols
We expected the normal or common operating environment for a robot to be a wireless WiFi local area network.  Every robot will have a processor on the robot.  That processor on the robot will be running [roscore](http://wiki.ros.org/roscore).  e.g. computer on the robot is where the robot's master URI exits.  We plan for a typical home or office WiFi installation.  In most, if not all, cases, this LAN will be connected to the Internet.  


We will constrain and define as a "basic" configuration capability.

 1. Supports communications with an IPv4 protocol stack
 2. Supports dynamic host configuration protocol (DHCP)
 3. Allows for Zeroconf address resolution

####Release deliverables
We, at a minimum, will further expect a user to want to communicate between ROS nodes running on computers which are off-robot or between multiple robots.  Basic use cases and demonstrations are for such examples as controlling a robot via teleop or watching an RVIS visualization of the robot.  The current iteration of this document and above objectives and goals only takes into consider a wireless local area network (LAN).  

Networking is the communications pipe between two endpoints - (1) the general purpose computer running on the robot and (2) a computer running additional ROS nodes.
 
As a part of our networking architecture, we will provide the following deliverables.

Base Hardware
: Robots will come with a general purpose computer on the robot.  

Base OS
: We will provide the ability to include the default operating system for the computer running on the robot.  This is the fruits of Wayne's "build image" script(s) in the [ubiquity-misc repository](https://github.com/UbiquityRobotics/ubiquity-misc).

Robot Configurator
: We will provide as a part of the **Base OS** and available for apt-get installation, Rohan's configuration program to simplify the administration and the out-of-box user experience.

####Requirement terms
MUST
: We will support the requirement in this release

WANT
: A "nice to have" capability.  We will not hold up a release to get this into the current release.

FUTURE
: Synonym of **WANT** that is scheduled for a future release.

NOT IN
:  A capability of which we are aware but we do not intend to provide any services or utilities in support of the capability.

| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| Wifi LAN support | 1 | **Robot Hardware** | MUST |
| Network Scope LAN | 1.1 | **Base OS, Configurator** | MUST |
| Network Scope Internet | 1.2 | **Configurator** | NOT IN |
| Network Scope inter LAN via Internet | 1.3 | **Configurator** | NOT IN |

Deliverable #1 is defining a constraint on the scope of our work.  That is to say, we are providing a base computer on the robot.  The computer MUST have a wireless Ethernet connection.  The computer is running a base operating system (OS) which we will provide out of the box (Currently this is Ubuntu 14.04).  And we will provide tools and services to make it simple to configure the robot to connect with another computer on the LAN.  We will leave the ability to connect with computers beyond the LAN to end user manual configuration.  We will strive to NOT include any component that would restrict that "Advanced" manual configuration


![Network Architecture](http://kchristo.homeip.net/img/net_arch.png)

Protocol Support
-------
This bucket of issues contains issues regarding what communications protocols our navigations stack will work with.  

| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| IP version 4 support | 2 | **Base OS, Configurator** | MUST |
| IP version 6 support | 2.1 | **Base OS, Configurator** | NOT IN |

Ubiquity Robotics plans are for support of only Internet Protocol version 4 to be configured.  It is understood, the ROS platform supports protocol independent communications.  However, I have only found examples of TCP and UDP communications (See: [TCPROS](http://wiki.ros.org/ROS/TCPROS) and [UDPROS](http://wiki.ros.org/ROS/UDPROS)).  Further, only basic sockets and packets are required.  I have not found use of SCTP nor any broadcast or multicast support in UDP thought there was a [discussion on answers](http://answers.ros.org/question/50323/udpros-unicast-broadcast-or-multicast/) that this would be useful.

For this reason, I beleive it would be sensible that our basic configurations and installations be limited to standing up a robot in an environment with an IPv4 stack. That is we do not need to ensure configuration and maintenance of IPv6 addressing.

Please send a use case to my email for removing this constraint in a future release. 
| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| Multicast support | 3 | **Base OS** | MUST[^1] |

In support of the Zeroconf requirement (below), we need to support multicasting.  

| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| Default Wireless Access Point | 4 | **Base OS[^2]** | MUST |
| Configure as wireless access point | 4.1 | **Configurator** | MUST |
| Configure WAP SSID | 4.2 | **Configurator** | MUST |
The out-of-the-box OS which we ship must configure the attached WiFi hardware as a wireless access point for it's own network.  The configuration utility can then be employed to change the connector to support the customer's LAN.  There are further corollaries that the configuration can easily return to acting as a access point.  

For the use case of multiple robots, each acting as a wireless access point, we will be able to configure the SSID published by the robot when it is in this mode.  

 
Address Resolution
-------
This bucket contains the issues which touch upon using and applying friendly host names and mapping them to the correct internet protocol address.


| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| Default connect via DHCP | 4 | **Base OS** | MUST |

Customer's networks will, as a rule, have dynamic host configuration protocol (DHCP) address pools for their access points and home networks.  The basic configuration tools will not attempt to force a static IP address configuration when discovering and joining the LAN in which it will be running.  

| Requirement | ID | Deliverable(s) | Priority   |
| :------- | :----: | :---: | :---: |
| Default connect via DHCP | 4 | **Base OS** | MUST |


Open Issue:  Do we provide through documentation, how to configure static addressing?

| Requirement | ID | Deliverable(s) | Priority |
| :------- | :----: | :---: | :---: |
| Configurable hostname | 5 | **Base OS, Configurator** | MUST |
Customers must be able to run their robot on the same network as other ROS robots.  This requirement in conjunction with support of address resolutions means we will need to be able to easily configure a host name for the robot.

| Requirement | ID | Deliverable(s) | Priority |
| :------- | :----: | :---: | :---: |
| Configurable fully qualified name | 5.1 | **Base OS, Configurator** | NOT IN |
| Integration with external Resolvers | 5.2 | **Base OS, Configurator** | NOT IN |

| Requirement | ID | Deliverable(s) | Priority |
| :------- | :----: | :---: | :---: |
| Work with Zeroconf | 6 | **Base OS** | MUST |

> Written with [StackEdit](https://stackedit.io/).

---------------------------------------------------------
---------------------------------------------------------

Original document with notes from meeting held Thursday 6/4/2015 at the Elephant Bar

# Ubiquity Network Architecture

We have a bunch of major issues to wrestle to the ground:

* Each robot needs a unique hostname so that zeroconf can
  give each robot a unique IP address. 
  > [klc] We will need to support a mode where robot is the Access point.  The robot will need to support DHCP.
  > [klc] We will only support IPv4 Configurations
  > [klc] We will only support the fully qualified zeroconf naming (.local) We will **NOT** attempt to tackle management of FQDN, DNS integration or NIS.

* Use some sort of `avahi-browse` functionality to find all robots.

* Allow user to switch easily between robots.
> [klc] Need clarification.  Looks like requirement is only "make it easy to support multiple robots on a single, shared, LAN.

* Each robot needs to be able to log into a bunch of different
  WiFi access points.

* Key management.  SSH.  WiFi.
> [klc] Wayne expects this is a part of the configurator.  What services?  UI implications

* Time synchroniziation.  (What about mutliple robots?)
> [klc] robots ship with batteries on the RPi 2

* Wrestle the WiFi dongle issues to the ground.

* Make sure we work with multiple SSID's and single replicated SSID's.
> [klc] Need to support node hopping

* Need dongle that supports WiFi client and access point on RasPi2

* How do we do multi-robot applications?
> [klc] Research ROS Bridge






***
Footnotes:
[^1]: Many network infrastructures are setup to deny multicasting or zeroconf packets. 
[^2]: This will be the default configuration for an "out-of-the-box" robot.
