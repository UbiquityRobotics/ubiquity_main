# Ubiquity Software Development Architecture

* Software development needs to occur on desktop/laptop machines.
  We need to make access to the robot easy.

* Perhaps use NFS to mount a common `src` directory between
  the laptop/desktop file system and the robot file system.
  Allow catkin_make to build on both the robot (ARM7) and the
  desktop/laptop (amd64) using same `src` directory.

* Create an `rx` command that reads the ROS_MASTER_URI, extracts
  the robot host name, and does `ssh user@ROBOT_HOSTNAME.local command`.

* We need to start using branches for development (e.g. indigo-wayne,
  indigo-mark, indigo-joe, indigo-kurt, indigo-rohan, etc.)

* Debugging C++ Node.  Need to run a node under the debugger and use
  a remote debugger UI.  Possibilities: gdb -tui, DDD, Kdevelop, etc.?

* Debugging Python Node. Node to run a node with a debugger.

* Document explaining developement is needed.

* Need some multi repository tool -- multi_git_status, multi_commit

* Support combine Arduino IDE and Arduino.Makefile development.

* We need a coherent place to store configuration files, launch
  files, etc.  We need a coherent .launch file strategy.

