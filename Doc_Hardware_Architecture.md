# Ubiquity Hardware/Software Architecture

The two key requirements of the Ubiquity hardware and software
architecture are:

* it *MUST* be totally ROS (Robot Operarting System) centric, and

* it must support easy replacement of the main processor boad.

At this point in time, ROS only runs under a Ubuntu release
of Linux.  The Kinetic release runs will under the 16.04LTS
Ubuntu release.

Only two processor architectures that support Ubuntu releases
are IA32/AMD64 processor archictecture and the ARM7 processor
architecture. Thus, whatever main compuation board is used, must
have one of these three processor architectures running on it.
Realistically, most people are using AMD64 in preference to IA32
these days, so the architecture choices are AMD64 and ARM7.

For the AMD64 architecture, the standand form factors for main
processor boards are some sort of x86 laptop, some sort
mini/nano/pico/mobile-ITX motherboard, and the Intel Joule.  
For the ARM7 architecture, there are plenty of single board 
computers, like the Raspberry Pi, Banana Pro, Beagle Bone Black, 
Nvidia Tegra, etc.

What is clear is that industry churns out a truly amazing number
of new SBC's every year.  A consequence of the annual SBC churn
is that older SBC versions go out of production to make room for
the new SBC computers.  Thus, the hardware architecture must
gracefully allow the SBC's to be replaced every 12 to 36 months.

Given that we are using an SBC, there are really only a limited
of number of connectors that make sense to connect to:

* Serial port:
  Serial ports are still provide in some flavor on most SBC's.
  Some support RS-232 signaling levels and others use simpler
  5/3.3/1.8 voltage signaling.

* Parallel port:  Classic parallel ports are nearly extinct.

* Ethernet: Ethernet is well defined, but requires a fairly
  large embedded microcontroller to support an Ethernet software
  stack.

* USB: USB is going strong.  USB slaves have lots of support.

* I2C: I2C runs at either 100kbps or 400 kbps.  It is an
  open collector bus with separate clock and data lines.
  While many SBC's have the ability to talk to the I2C bus,
  it is not a forgone conclusion that user level programs
  can access the I2C bus.  Worse than that 

* SPI: ..

* CAN ..

* MIPI CSI: ...

> *Mention that Ethernet, USB and Blue Tooth are hardware portals
> to get into ROS instead of just serial. -Wayne*

> *Mention that there can be mulitple serial ports are allowed. -Wayne*

> *Provide more detail on hardware development API's vs
> user development API's. -Wayne*

The top level Ubiquity Hardareware/Software architecture
diagram is shown below:

<BlockQuote>
<Img Src="Ubiquity_Hardware_Software_Architecture.png"
Alt="Ubiquity Hardware/Software Architecture">
</BlockQuote>

Each light blue rectangle corresponds to a hardware
assembly.  In the diagram above, the following hardware
assemblies exist:

* Raspberry Pi 3: This is were ROS is run.  The ROSCore node
  runs here.

* bus_raspberry_pi: This is a module the interfaces the
  Raspberry Pi 3 to the bus.

* bus_sonar_10: This module can control up to 10 HC-SR04
  sonar modules.  There are two of these modules on a Freya.

* bus_bridge_encoders_sonar: This module has the electronics
  to drive two motors and encoders.  (It also can drive two
  HC-SR04 sonar modules, but that is not actually used.)

* WiFi Access Point:  This module provides a wireless access
  point so that the robot can communicate with the desktop/laptop.

* Desktop/Laptop: This is the machine on which software is
  developed and debugged.  While ROS runs on this machine as well,
  it accesses the ROSCore node on the Raspberry Pi 2.

The ovals correspond to ROS nodes.  There is a one-to-one
relationship between the bus hardware boards and ros nodes.
The `node_server` node is associated with the `bus_raspberry_pi`
node.

The bus_server_node is a traffic cop that multiplexs messges
from other ROS nodes to the bus through the bus_raspberry_pi
hardware module and back.

There are two commuication protocols:

* 8N1: The 8N1 protocol is 1 start bit, 8 data bits,
  no parity and 1 stop bit.  The Raspberry Pi 2 UART
  supports this protocol with an on chip UART.

* 9N1: The 9N1 protocol is 1 start bit, 9 data bits,
  no parity and 1 stop pit.  The all of the modules connected
  to the hardwar bus talk 9N1 protocol.

The Raspberry Pi2 on chip UART does not support 9N1 mode;
hence, the requirement for the bus_raspberry_pi module to
convert between the two protocols.

The bus architecture has gone through many many iterations:

* [Robobricks](http://gramlich.net/projects/robobricks/index.html):
  This version was built around PIC12 modules.  It actually
  actually sold by Robot Store for a while back in the early
  2000's back before Robot Store was acquire by Jameco.

* [Robobricks2](http://gramlich.net/projects/rb2/index.html):
  This verison is built around PIC16 modules.

* [Robus/Makerbus](http://gramlich.net/projects/robus/index.html):
  This version is built around NXP ARM Cortex 3 modules
  (LPC17xx series.)  These boards are all surface mount.
  There were two names that we played around with -- Makerbus
  and Robus.  They both describe the same bus.

The ubiquity modules are all simply called "bus" and are
based around Atmel ATmega microcontrollers to be compatible
with the Arduino<Sup>&tm;</Sup> community.  These modules
are both electrically and software compatible with the
Robus/Makerbus modules.

Please read the
  [MakerBus Specifications](http://gramlich.net/projects/robus/specifications.html)
to get all of the electrical, software, and mechancal details.

Associated with this project is a software tool to help
configure the modules called
  [configurator](http://gramlich.net/projects/configurator/index.html).
Please note, that this is not the same thing that Rohan talks
about when he says "configurator".  The MakerBus/Robus configurator
is going to be depricated and switched over to use ROS configuration
technology.

The ultimate architecture is that each hardware module presents
its functionality that is organized as a bunch of registers.
Some registers are written to at start up time to perform
configuration.  After configuration, registers are written
to trigger actuators and read to read sensors.

All of the register information for a hardware module is described
in a single .xml file.  It lists each register, it type (Boolean,
Byte, Short, Integer, Long, etc.), a textual desciption, the register
index, etc.  A stand-alone batch processor reads the .xml file and
generates a whole bunch of files needed by ROS to talk to the hardware
module.  The initial code is currently living in the
  [bus_slave](https://github.com/UbiquityRobotics/bus_slave)
repository in the `bus_code_generatory.py` file.  This code is
going to be expanded and likely moved to its own repository.

The requirements of the bus software architecture are:

* Easy addition of new modules.

* Integration with ROS static parameters and dynamic configuration.

* Bus discovery whereby the bus is scanned to determine which
  modules are on it.

* Bus firmware update whereby the firemware can be updated from ROS.

* Code generation where boiler plate code is generated to make
  it easier to maintain code.

* Documentation generation where documentation files are automatically
  generated.

* Testing support is generated so it is easy to test modules.

Priorities:

* Write `.xml` regsiter definition files for `bus_sonar10`,
  `bus_bridge_encoders_sonar`, and `bus_loki`. (Priority: High)

* Write .xml documentaiont. (Priority: High)

* Modify `bus_slave/bus_code_generator.py` to generate C++
  code fragments that can be specialized. (Priority: Medium)

* Replace ROS Arduino Bridge. (Priority: Medium)

  * Get bus_server_server.py to talk to other modules.

  * Get parameters to work.

  * Create odometry topic publisher.

  * Create sonar range topic publisher.

  * Create cmd_vel topic subscriber.

* Get over the wire firmware upgrade to work. (Priority: Medium)

* Get dynamic reconfiguration working. (Priority: Medium)

* Rewrite bus_server_server.py in C++. (Priority: Low)

* Rewrite topic publisher and subscribers in C++. (Priority: Low)

* Get bus reset to work. (Priority: Low)

* Get discovery to work. (Priority: Low)


