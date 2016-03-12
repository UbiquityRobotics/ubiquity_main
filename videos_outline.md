# Ubiquity ROS Video

## Video 1: Introduction to Ubiquity

* Intro of Wayne as HBRC president and member of UR team.
* Show UR splash page.
* Intro of UR.
  * We believe that robots will be ubiquitous.
  * In 1977, Star Wars was filled with "droids".
  * We call them "Bots".
  * Our slogan is "Bots Everywhere!"
  * Introduce our robots:
    * Stage (simulator)
    * ROS Pi Bot
    * BotVac
    * Freya
    * Loki
    * Magni
  * All of our robots run ROS (Robot Operating System)
  * We'll talk more about ROS next time

Script:

* [Show UR splash]
* "Hi, I'm WG."
* "I discovered the SV HBRC in the mid-90's and I've been doing robots ever since."
  [Show HBRC logo]
* "Indeed, I got drafted to be HBRC president about 10 years ago."
* "For the past 4 years, I've been working with a group of like minded
  people to work on a concept we call 'Ubiquitous Robots'".
  [Show group shot in front of Robot Shed]
* "The 'UR' vision is that robots will be as prevalent as cars, TV's, and computers".
* 'We believe that robots will come in a variety of sizes, shapes, and functionality,
  just like 'droids in a Star Wars movie."
  [Show still shot of R2D2 and C3PO.]
* "To pull off this vision, robots need to share some common software so that one
  robotic application can be run on a number of different robots."
* "This is called a Robot Operating System and luckily another group of people
  have already developed ROS."
  [Show OSRF Logo]
* "We just want just wanted to buy a ROS capable robot off the shelf."
* "When we priced what was available, we had a serious case of sticker shock."
* "Thus, we decided to build our own."
* "We ultimately came up with a three prong approach to develop robot applications."
* "First, you do your initial development using a robot simulator."
  [Show Stage]
* "Second, you do the next level of development on a small inexpesive robot.]
  [Show Loki]
* "Third, once that is starting to work, you upgrade the platform to a
  more robust and powerful platform".
  [Show Magni]
* "If you visit our web site, you will find instructions on how do download
  and run a ROS robot simulator for free."
  [Show web site]
* "Our low cost entry level ROS robot is called Loki and it consits of two
  motor, encoder and wheel assemblies, a ring of 16 sonars to detect obsticles,
  an optional arm, an Arduino compatible processor to control the motors, sonars,
  and arm, and a Raspberry Pi computer with both a WiFi dongle and a camera
  that runs ROS."
  [Show Loki]
* "Our heavy duty platform is called Magni and consists of two electric
  scooter motor/wheel assemblies, a power management and motor control board,
  some lead acid batteries, and again, a Raspberry Pi computer with Wifi to
  run ROS on."
  [Show Magni]
* "Using this three prongOne of our team members was able to develop a voice activated robot driving
  application."
  [Show 4 screens - Simulator, Loki, Magni, and Wayne]
* "Robot forward 1 meter"
  [All 3 robots move forward 1 meter]
* "Robot left 45 degrees"
  [All 3 robots turn left 45 degrees]
* "Robot forward 50 centimeters"
  [All 3 robots move forward 50 centimeters]
* "In the next video in this series, I will talk more about ROS."
* "Until then, I want to close with a question and comment:
* "'What robot application do *you* want to develop?'.  Let's get started now!."
  [UR Splash Screen]

## Video 2: ROS: Robot Operating System

* ROS stands for 'Robot Operating System'
* Created by Willow Garage (now closed) which was funded
  by an early Google employee.
* Currently maintained by Open Source Robotics Foundation.
* Show stage demo running move base (i.e. the Girts demo.)
* Explain that "Stage" is a simulator with a robot, walls,
  a Lidar (explain what a lidar is).
* Show move base in operation.
* Now show `ros_rqt` graph of nodes and topics.
* Explain about nodes, topics, and services.
  * Node is a *nix process
  * Node's talk to one another via topics and services
    * A topic is multi-cast.
    * A service is a remote procedure call.
* Also explain about parameters.
* All nodes, topics, services, and parameters have a namespace.
* Show `rosnode list`, `rostopic list`, `rosservice list`, and
  `rosparam list`.
* Lastly, ROS launch files are used to fire off all of the
  nodes and plug them all together.
* Next is installing via virtual box.

Script:

* [UR splash screen]
* "Hello, I am Wayne Gramlich and welcome back to Ubiqity Robotics."
* "In this video, I'm going to spend some time doing a brief overview
  ROS -- Robot Operating System."
* "ROS was developed in Silicon Valley at Willow Garage, a robotics
  think tank founded by an early Google employee."
* "The two robots that Willow Garage is most famous for developing are
  the PR2 and the TurtleBot."
  [show pictures of PR2 and TurtleBot]
* "Enough history, 'What is ROS?'"
* "Ultimately, ROS is a confederation of processes that together
  run a robot platform.  They communicate with one another via ROS
  commuication protocol."
* "The four primary ROS concepts are 1) nodes, 2) topics, 3) services,
  and 4) parameters."
* "There is a fifth topic called launch files which is used to glue
  everything together."
* "Let's first talk about nodes and topics."
* "In ROS terminology, a ROS node is a program runs in parallel
  other ROS nodes."
* "In ROS terminology, a ROS topic is a communication channel by which
  ROS nodes communicate with one another."
* "We'll show a diagram of the nodes and topics that implement the robot
  simulator."
  [show rqt_graph output]
* "In this picture there are 12 ROS nodes represented by horizontal ovals.
  When the mouse is over an oval, it highlights the node in red."
  [show red node highlight]
* "The nodes usually communicate with one another via ROS topics.
  The best analogy for ROS topic is that it is analogous to a radio channel.
  For a radio channel there can be multiple nodes that transmit on the
  channel and there can multiple recepiants as well.  In ROS terminology,
  a transmitter is a ROS topic publisher and a receiver is a ROS topic
  subscriber."
* "When the mouse is over ROS node, the topics that the node subscribes
  to are highlighted in blue and the topics that it publishes are highlighted
  in green."
* "The next concept

## Video 3: Installing ROS Via Virtualbox

* ROS currently only runs under Ubuntu Linux.
* If you are running some other operating system (e.g. Windows,
  MacOs, Non-Ubuntu Linux, Solaris), use virutalbox.
* Visit [Oracle VirtualBox](https://www.virtualbox.org/) and download.
* Download VirtualBox image from UR.
* Download the Extension Pack (Is that need for VB 5.x?)
* Start virtualbox image.
* Explain that we are running lubuntu/LXDE.
* Login as user ubuntu and user ubuntu
* Start a terminal window
* Fire off stage demo again
* Show it working again.
* Show `rqt_graph` again.
* Show `rosnode list`, `rostopic list`, `rosservice list`,
  `rosparam list` again.

## Video 4: 1st ROS Program: Keyboard Teleop

* Talk about github.com.
* Create a github account.
* Fork a github repository for the template.
* Create a catkin workspace.
* Explain what a catkin is and how it relates to Willow Garage
* Go to src and do a git clone
* cd into dir and run `urcatkin_make`
* Run program using roslaunch
* Explain that we always use Python for beginners
* Show `f xxx` and `r xxx` commands to go forward and turn right.
* Bring up an editor (vi or emacs) and look at code.
* Walk people through the code.
* Show how to add `b xxx` backward and command.
* Explain about Ubuntu (i.e. Debian) packages
* Explain about ROS packages.

## Video 5: 2nd ROS program: Keyboard Move Base

* Show basic program.
* Add way point recording
* Add way point "goto"

## Video 6: ROSberry Pi Bot / Ceiling Fiducials

* Show ceiling fiducial and explain its components
* Show ceiling fiducials on ceiling
* Load up micro-SD card
* Plug in Ethernet cable
* (Optional) plug in HTMI cable and USB keyboard
* Power up. Login. Set host, WiFi, and user.
* On laptop use `ssh`

## Video 7: Networking and WiFi

* Hostnames *must* be unique
* We use zerconf => requires common subnet
* WiFi is a bit tricky
* Dual-band 2.4GHz/5GHz
* Configuring WiFi on Robot
* Use a good WiFi/router

## Video 9: BotVac

## Video 10: Loki

## Video 11: Magni

