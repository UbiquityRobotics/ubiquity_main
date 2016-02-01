# Robot Launch Repositories

A robot launch repository is a `git` repository that
primarily contains ROS `.launch` files and associated
robot configuration files (e.g. `.yaml`, `.urdf`, etc.)
A combination of shell scripts and
[ROS `.launch`](http://wiki.ros.org/roslaunch) files are
used to start and configure one or more
[ROS nodes](http://wiki.ros.org/Nodes), which when properly
configured will result in the desired robot behavior.

By imposing some structure on the organization of these
launch repositories, we improve the ability to reproduce
robot behaviors among all robot developers who share the
same robot launch repository.

## Overall Architecture

Ultimately, there are a bunch of shell scripts and launch files. 
The shell scripts will typically call a launch file with one
or more launch file arguments set.

This repository is broken into some categories:

* `bin`: The `bin` directory contains a bunch of executable
  shell scripts that fire off a launch file.  It is expected
  that you will place this `bin` directory in your path.

* `n_*`: The n_* directories contain a ROS launch files and
  associated configuration files needed to launch a single
  ROS Node.

* `m_*`: The m_* directories contain a ROS launch that
  will start Multiple ROS nodes.

* `rviz_*`: The `rviz_*` directories are used to launch the
  RViz program configured to view a corresponding robot program.
  These launch files are typically executed on your laptop/desktop,
  since most robots do not have a display head.

* `view_*`: The `view_*` directories are used to launch the
  `rqt_image_view` program configured to view a specific image
  topic.  These launch files are typically executed on your
  laptop/desktop, since most robots do not have a display head.

* others: Other directories contain miscellaneous launch files.

## ROS Launch File Issues

There are several issues about ROS launch files that need to
be discussed:

* The `<arg>` tag is used heavily needs to be fully understood.

* There are two common launch file suffixes -- `.launch` and
  `.launch.xml`.

* Launch file parameterization allows the same launch files
  to be used for different robot platforms and configurations.

### The `<arg>` Tag

Some documentation for ROS launch files can be by following
the hypertext links below:

* [ROS Launch Overview](http://wiki.ros.org/roslaunch) provides an
  over view of the ROS launch file architecture.

* [ROS launch XML file format](http://wiki.ros.org/roslaunch/XML)
  provides documentation of the XML format used for writing
  ROS launch files.

* [ROS launch `<arg>` tag](http://wiki.ros.org/roslaunch/XML/arg)
  is the documentation for the `<arg ... >` tag.

The `<arg>` tag is singled out because it is used in the
launch files to pass around the robot platform information.
If you do not understand the `<arg>`, you will not understand
the launch files.

The `<arg>` tag has three forms:

* `<arg name="required" />`: This specifies a launch file input name.
  Think of this as an argument variable for routine.

* `<arg name="optional" default="value" />`: This specifies a launch
  file input name with a default value that will be used if not
  is specified at "call" time.

* `<arg name="foo" value="bar" />`: This form has two different usages.
  When at the level immediately inside of a `<launch> ... </launch>`
  pair, this form defines a convenience value that can be used to
  improve overall legibility.  Think of this as a kind of a macro
  definition.  The second form occurs immediately inside of a
  `<include> ... </include>` pair.  This form is like passing arguments
  into a routine call.

Huh? What is going on here?  Let's do some examples!  Here is
a chunk of Python code that defines a routine:

        def n_fiducial_slam(robot_base, fiducial_size=".150"):
          short = "a somewhat long string"

This function is named `n_fiducial_slam` and has two arguments --
`robot_base` and `fiducial_size`.  `robot_base` is a required
argument which if not present at routine call time will cause
a run-time error.  `fiducial_size` is an optional argument that
does not need to specified in the routine call, but it can be
specified if you want.  `short` is a local variable that can
be used to reduce typing.  The corresponding launch file syntax is:

        <launch>
          <!-- Required arguments: -->
          <arg name="robot_base" />
          
          <!-- Optional arguments: -->
          <arg name="fiducial_size" default=".200" />
          
          <!-- Convenience arguments: -->
          <arg name="short value="a somewhat long string" />
          
          ...
          
        </launch>

The `<include> ... </include>` tag pair is how one launch
file accesses another launch file.  It is similar to a
routine call.  In python, the following line:

        n_fiducial_detect("loki", fiducial_size=".200")

would be written in launch file syntax as:

        ...
        
        <include file=".../n_fiducial_detect.launch">
          <arg name="robot_base" value="loki" />
          <arg name="fiducial_size" value=".200" />
        </include>
        
        ...

Now that you know how to set an argument, the only other
issue is how to access it.  That is done using substitution
arguments.  The syntax is:

        $(arg name)

where `name` is the argument name.  Using the "call" example
above, the following string:

        "base=$(arg robot_base) size=$(arg fiducial_size) short='$(arg short)'"

would expand to:

        "base=loki size=.200 short='a somewhat long string'"

The substitution syntax can only occur inside of XML attribute
strings.

Finally, you pass arguments into launch files from the `roslaunch`
command via the following syntax:

        roslaunch ubiquity_launches n_fiducial_detect robot_base:="magni"

It is the `:=` is detected and conceptually converted into an
`<arg>` tag.  The line above would be represented in a launch
file as:

        <include file="$(find ubiquity_launches)/n_fiducial_detect.launch">
          <arg name="robot_base" value="magni" />
        </include>

Hopefully this explanation of the `<arg>` tag is a little
more informative that the official ROS documentation.

### Launch File Suffixes:

There are two ROS launch file suffixes:

* `.launch`: This launch file will be discovered by the `roslaunch`
  command.

* `.launch.xml`: This launch file will not be discovered by
  the `roslaunch` command.

The [`roslaunch`](http://wiki.ros.org/roslaunch) command
has the following basic structure:

        roslaunch ROS_PACKAGE LAUNCH_NAME.launch

Basically, the `roslaunch` command searches a given ROS package
(i.e. `ROS_PACKAGE`) for a `.launch` file (i.e. `LAUNCH_NAME.launch`.)
One neat thing about `roslaunch` is that it implements
[tab completion](https://en.wikipedia.org/wiki/Command-line_completion]
whereby it will reduce overall typing by allowing you few
characters of the package name and/or launch file name followed
by a tab character to cause the `roslaunch` to fill in as
many unambiguous characters as possible.  When it comes to finding
`.launch` files, `roslaunch` recursively visits all of the
directories and sub-directories in a ROS package and identifies
every file that ends in `.launch`.  It does not matter what
package sub-directory the .launch file is in, `roslaunch` will
find it.  It is really that simple. 

A robot launch file repository will have many launch files.
Many of these file are not likely to only be used via
the `<include>` tag in some other launch files.  These
launch files use the `.launch.xml` suffix, so that when
you are using tab completion for `roslaunch`, they do not
up as one of the possible completions.  That is all that
is going on here.  To let `roslaunch` show a launch file
via tab complete, use the `.launch` suffix; otherwise,
use the `.launch.xml` suffix to keep `roslaunch` from
showing the launch file via tab completion.  It is that easy.

### Launch File Parameterization

The goal of a robot launch repository is to provide
high quality launch files that work across multiple
robot platforms and configurations.  It would be
possible to build monolithic launch files that do not
use any `<include>` directives.  The reason for not
doing that is because you would have lots of replicated
text across multiple launch files.  Fixing a problem
in one launch file would have to be manually propagated
to all the other launch files.  This would be a maintenance
nightmare.

The solution is to break the launch files into a number
of smaller launch files and create the robot configuration
via composition as described in the
[roslaunch Architecture](http://wiki.ros.org/roslaunch/Architecture).

To get additional reuse, the launch files need to be
parameterized such that the same launch file can be
used for multiple robots.

For example, the most common parameter is the robot base name.
This is called the `robot_base` parameter and it expected to be
given a robot base name (e.g. `loki`, `magni`, `botvac`, etc.)
This argument is used to select between different parameter
files (e.g. `loki.yaml` vs. `magni.yaml`, or `loki.urdf` vs.
`magni.urdf`, etc.)

## Repository Organization

Now that we have covered the various launch file issues
we can intelligently discuss the repository organization
This broken into:

* the top level directory structure,

* individual launch sub-directory structure,

* Basic file structures, 

### Top Level Directory Structure

The top level directory structure is as follows:

        repository_name/
            README.md    # Documentation for **ALL** launch files
            bin/         # Executable shell scripts
            n_*/         # Single ROS node launch files
            m_*/         # Multiple ROS node launch files
            rviz_*/      # RViz specific launch files

We use some simple directory naming conventions to clue
people into what kind of launch file is in a directory.

* `bin`: This directory contains a bunch of short shell
  scripts.  It is expected that you will choose to place
  this `bin` directory into your `$PATH`.

* `n_`: If a launch directory starts with `n_`, the associated
  launch file starts a single ROS node.  The `n` in `n_` is
  short for Node.

* `m_`: If a launch directory starts with `m_`, the associated
  launch file fires up multiple ROS nodes.  The `m` in `m_` is
  short for Multiple.

* `rviz_*`: If a launch file starts with `rviz_`, the associated
  launch file is used to launch the
  [RViz](http://wiki.ros.org/rviz) visualization tool on a
  laptop/desktop.  The launch file properly configures `rviz`.

While this structure will evolve over time, currently it is
not very complicated.

### Launch Sub-Directory Structure

Each launch sub-directory is organized as follows:

        LAUNCH_DIR_NAME/
            launch/   # Usually one `.launch` (or `.launch.xml`) file
            params/   # One or more `.yaml` (or other) parameter files
            rviz/     # One or more `.rviz` configuration files
            urdf/     # One or more `.urdf` configuration files

It is not clear why there is an additional level of sub-directory
for each different data type, but the turtlebot launch directories
have this structure, so it was decided to copy it.  (There may be
some subtle interaction with [ROS `bloom`](http://wiki.ros.org/bloom)
that is not yet fully understood.  Alternatively, it could be
monkey see, monkey do.)

### Basic File Structure:

There are two primary files in this repository -- short
shell scripts and launch files.

The shell scripts are usually only a few lines and look
as follows:

        #!/usr/bin/env bash
        roslaunch ubiquity_launches LAUNCH_NAME.launch robot_base:=BASE

where:

* LAUNCH_NAME is the name of a `.launch` (or `.launch.xml`) fiel, and

* BASE is a robot base name (e.g. `loki`, `magni`, etc.)

The structure of a `.launch` (or `.launch.xml`) file is as follows:

        <!-- Comment explaining what the launch file does. -->
        <launch>
          <!-- Required Arguments -->
          <arg name="required_argument_name1" />
          <arg name="required_argument_name2" />
          ...
          &nbsp;
          <!-- Convenience Arguments -->
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="convenience_argument_name1" value="value1" />
          <arg name="convenience_argument_name2" value="value2" />
          ...
          &nbsp;
          <!-- Includes -->
          <include file="...">
            <arg name="arg1" value="value1" />
            <arg name="arg2" value="value2" />
            ...
          </include>
          ...
          *nbsp;
          <!-- Nodes -->
          <node pkg="package_name" type="executable_name"
           name="ros_node_name" ... >
            <!-- Optional env, remap, rosparam, and param tags -->
          </node>
          *nbsp;
        </launch>

The first convenience argument is almost always:

          <arg name="root" value="$(find ubiquity_launches)" />

The `root` argument provides the root path to the
`ubiquity_launches` directory.  From there the include file
can find parameter files.

### Using `git` Branches to Experiment

There is not much to say here.  If you want to tweak things
to experiment, you can use `git` to get a copy of the files,
create a branch and modify things to your hearts content.
If you can make a the case that you configuration works
better that what is currently in the `ubiqutiy_launches`
git repository, please submit a pull request back to the
master `ubiquity_launches` git repository.

## Future Possibilities

There are a number of future possibilities:

* Currently, the `README.md` file is manually updated whenever
  a new `bin` file or launch sub-directory is added.  This could
  be automated with a program that reads the data out of each
  `bin` file and launch sub-directory.

* ROS 2.0 is going to have more introspection.  At some point in
  time it is likely that there will be graphical tools to help
  assemble the nodes that make up a robot behavior.  This will
  almost certainly impact this launch repository architecture.

### Debugging Support

We want to be able to run Python and C/C++ debuggers locally
on the robot, but ashow debugger running in a window the
laptop/desktop.  This is done using an `xterm` window which
via the X11 protocol forwarding via an SSH connection.
The article
[How X Over SSH really works](http://dustwell.com/how-x-over-ssh-really-works.html)
helps to explain what is going on.  The bottom line is that
we need the following:

* One `ssh -X` connection between the remote machine to the
  robot to cause the secure shell daemon (i.e. `sshd`) to turn on.

* We need to set the `DISPLAY` environment variable to the
  value created by previous connection.  It will be of the
  form `localhost:NN.0', where NN is 10 or higher.

In addition, it is necessary to ensure that `/etc/ssh/ssh_config`
is edited to enable `ForwardX11` and `ForwardX11Trusted`.

        ForwardX11 yes
        ForwardX11Trusted yes

Frankly, given how mature X11 is, the documentation that
explains all of this is still quite gnarly.

Once all the X11 stuff works, the documentation for using
[ROS Launch Debug Support](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)
can be visited.

Now that we have X11 set up, we need to get `xterm` installed
on the robot:

        sudo apt-get install xterm

Once `xterm` is installed, it can be run as follows:

        xterm -e program_name argument1 ... argumentN

This will run *program* with *argument1* through *argumentN* passed
in and ther result will pop up on the laptop.

For example, to run the python `pudb` debugger:

        xterm -e python -m pudb.run my_python.py

Similarly, to run gdb with the TUI (Text User Interface):

        xterm -e gdb -tui my_program

You can also specify font size on the `xterm` command line.
First run:

        xlsfonts

to list the fonts.  Now, you can set the font size as:

        xterm -fn 9x15bold ...

to set the font to `9x15bold`.

Here are some `gdb` and `pudb` links:

* [gdb Text User Interface](https://sourceware.org/gdb/onlinedocs/gdb/TUI.html)

* [Python Debugging Wiki](https://wiki.python.org/moin/PythonDebuggingTools)

* [Intro to pdb](http://heather.cs.ucdavis.edu/~matloff/pudb.html)

* [happy pudb user](https://asmeurersympy.wordpress.com/2010/06/04/pudb-a-better-python-debugger/)

* [more happy](https://asmeurersympy.wordpress.com/2011/08/08/hacking-pudb-now-an-even-better-python-debugger/)

* [Old screencast](https://vimeo.com/5255125)

An article on
[Passwordless SSH Login](http://www.tecmint.com/ssh-passwordless-login-using-ssh-keygen-in-5-easy-steps/)
was useful.  The steps below get the job:

        ssh-keygen -t rsa
        # Type [Enter] for all prompts:
        ssh ubuntu@robot.local mkdir -p .ssh
        # Type in "yes" and "password" as possible:
        cat ~/.ssh/id_rsa.pub | ssh ubuntu@robot.local 'cat >> .ssh/authorized_keys'
        ssh ubuntu@robot.local "chmod 700 .ssh; chmod 640 .ssh/authorizied_keys"
        ssh ubuntu@robot.local
        # No password is needed now.

### File System Sharing

For a variety of reasons, it would be nice to allow the developer
to directly access the catkin workspace on the robot from the
laptop/desktop.  

Of the various network file system protocols out there, the
`sshfs` seems to be the easiest one to use.  To set up `sshf`,
do the following:

        sudo apt-get install sshfs
        sudo gpasswd -a $USER fuse

To mount the robot `~/catkin_ws/src` on top of the local `~/catkin_ws/src`:

        sshfs ubuntu@robot.local:/home/ubuntu/catkin_ws/src ~/catkin_ws/src -o nonempty

To unmount:

        fusermount -u ~/catkin_ws/src

There is additional documentation on
[installing SSHFS](https://help.ubuntu.com/community/SSHFS)
available.

## Launch Files for Multiple Processors

The ROS launch files support the ability to launch node on
more than one processor.  This done in conjunction with the
`<machine .../>` tag.  Each `<machine .../>` tag specifies
a machine name some addition configuration information for
each machine.  The 
[<machine> tag](http://wiki.ros.org/roslaunch/XML/machine)
attributes we require are:

* `name="machine_name"`:
   This attribute specifies the machine name that used to reference
   the machine elsewhere in the launch file.

* `address="network_address":
   This is the DNS address for the processor.

* `user="user_name"`:
   This attribute specifies the user name to use to log into
   machine.

* `default="true|false|never":
   In general, we never want a non-robotic machine to be default.
   Thus, the remote machine is set to `never`.  For the robot,
   it is a reasonable default, so we set it `true`.

* `env-loader="full_path_for_environment_script":
  When a remote node is executed, `~/.bashrc` is **NOT** executed.
  Instead, the file specified by this attribute is executed
  to set the needed environment variables.  This file then executes
  its arguments using `exec "$@"`.  This file is a bit tricky
  an is discussed further below.

The networking aspects of the `<machine .../>` tag are a bit
non-obvious.  The `address="..."` attribute is actually pretty
straight forward.  Since we use the zeroconf system, the address
to stuff into this attribute will always end with `.local` (e.g.
`robot.local`, `laptop.local`, etc.)  The more subtle issue
occurs with the `env-loader="..."` attribute.  When, the `roslaunch`
system attempts to start a node, it does **NOT** run `~/.bashrc`
beforehand.  Thus, the `ROS_HOSTNAME` environment variable
is not set *unless* it is set in the `env-loader` script.
In addition, the catkin search path is not set up either.
The miniminal `env-loader` script would look something like:

        #!/bin/sh
        export DISPLAY=localhost:12.0
        export ROS_MASTER_URI=http://betty.local:11311
        export ROS_HOSTNAME=`hostname`.local
        . /home/ubuntu/catkin_ws/devel/setup.sh
        exec "$@"

where both the `ROS_HOSTNAME` environment variable gets set
and the standard catkin workspace is in the search path.

There is note on
[roslaunch of remote processors](http://answers.ros.org/question/10725/question-about-roslaunch-and-remote-processes/)
over in
[ROS answers](http://answers.ros.org/questions/)
that was quite helpful for figuring these issues out.

In order for the ROS launch architecure to to work, we
need to have password free `ssh` working.  In addition,
the `ROSLAUNCH_SSH_UNKNOWN` environment variable needs
to be set to `1`.

        export ROSLAUNCH_SSH_UNKNOWN=1

{talk about node="..." here}

This should be added to your `~/.bashrc` file for your
laptop/desktop.

One down side of the "<Machine.../>" tag architecture
in ROS launch files, is there there is no real place to
specify where `roscore` should be run.  The solution we
have adopte is to do the following:

        ssh -X ubuntu@robot.local roscore

{Talk about scraping the X11 protocol environment variables here.}

One nice thing about ROS launch, is that when you type
Control-C, it takes care of shutting all the nodes down
on both the local process or and

### Platform Neutral Launch Files

We want to support a variety of different robotic platforms
(after all we are *Ubiquity Robotics*.)  For now, the following
platforms are envisioned:

* Stage: This is a 2-dimensional simulator that can be run without
  requiring any robot hardware.

* Loki: This is a small and relatively small robot platform with
  limited payload capability.

* Magni: This is a larger platform with more significant payload
  capabilities.

We want the launch files to be platform neutral.

What we want is that when we run the a Ubiquity Robotics
program on your laptop/desktop, the launch files will
determine which platform (e.g. Stage, Loki, Magni) to use
and fire all the necessary ROS nodes to bring it up.  In
addition any appropriate visualization tools (e.g RViz)
will come up on the laptop/desktop appropriately configured.
Much of this behavior will be set up by a shell script
before running `roslaunch`.

The way that the user specifies the robot to run is via
the `ROS_MASTER_URI` environment variable.  Since Ubiquity
Robotics extensively uses the `zeroconf` system, the
format of the `ROS_MASTER_URI` is:

        http://HOSTNAME.local:11311

where `HOSTNAME` is the host name of the processor that is
(or will be) running roscore.

The difference between a laptop/desktop and a robot is that
the laptop/desktop has a display head and the robot does not.
Linux currently uses the X11 window system to manage the display.
Critical to this is the `DISPLAY` environment variable.  If 
a shell script finds that the `DISPLAY` environment variable
is present, we assume that the we a laptop/desktop.  Conversely,
if `DISPLAY` is not present, it is assumed that the script is
running on a native robot.

Lastly, the `ROS_HOSTNAME` environment variable, specifies
the name to use to access the local host.  For Ubiquity
Robotics, this will have the form:

        HOSTNAME.local

where `HOSTNAME` is the host name for the processor.

Using the `DISPLAY` and `ROS_MASTER_URI` environment variables
we can figure out the following:

        DISPLAY  ROS_HOSTNAME    ROS_MASTER_URI              LAPTOP  ROBOT
        ====================================================================
        empty    robot.local     http://robot.local:13311    none    robot
        :0       laptop.local    http://laptop.local:13311   laptop  none
        :0       laptop.local    http://robot.local:13311    laptop  robot

The next step is to determine the what platform the robot is.
We are going to do this with a probe program.  This program
will attach to the appropriate serial port and send commands
down the line to see whether is a Loki or Magni platform.
If there is no serial port, probe will return "Stage" as the
platform (for now.)

### Modifiying roslaunch:

Currently, `roslaunch` does not do two things that we need
regarding the `<machine .../>` tag:

* It does not establish a connection to the to the remote
  machine with X11 forwarding.   We really really want X11
  forwarding turned on.  It would be nice to have a
  `ROSLAUNCH_SSH_X11_FORWARDING` environment variable that
  when set to `1`  enables X11 forwarding.  This would result
  in the remote  process having the `DISPLAY` environment
  variable set to `localhost:N.0` with some value of N greater
  than 10.

* There is no way to specify that `roscore` should be launched
  on a remote machine.  Again, we really really need this.
  The proposal would be to add a `roscore` attribute to the
  `<machine .../>" tag that would ask `roslaunch` to start
  `roscore` on the remote machine.

The `roslaunch` program can be found in the 
[ROS `roscom` package](http://wiki.ros.org/roslaunch)
which is available in source form via:

        cd .../catkin_ws/src
        git clone https://github.com/ros/ros_comm

This will show up in the `ros_comm` directory.

We need to fork the `ros_comm` package into the `UbiquityRobotics`
repository.

The code for `roslaunch` can be found in
`.../catkin_ws/src/ros_comm/tools/roslaunch/src/roslaunch`.
The code that is responsible for remotely launching nodes
on remote machine nodes is in `remotoeprocess.py`.  This
code uses the Python SSH library called `paramiko`.  This
library does implement the X11 forwarding protocol, but it
does not implement the stuff to enable the protocol.  This
is documented in a 
[stackoverflow paramiko X11 forwarding post](https://github.com/ros/ros_comm).
If we integrate the following code taken from the post,
should be able to do X11 forwarding back to the display machine.

The last thing to integrate to provide a mechanism to force roscore
to be run on a remote machine.  This is done via the `load_roscore`
routine in `config.py`.  One thought is to add an attribute for the
`<machine roscore="1" ... />' that forces roscore to be run on a
remote machine.

### Latest Launch Files/Scripts etc.

The following file is `.../ubiquity_launches/bin/platform_probe.py` and
figures out which platform it is:

        #!/usr/bin/env python
        
        import os
        import os.path
        
        def main():
            is_raspberry_pi = os.path.exists("/dev/ttyAMA0")
            #print("is_raspberry_pi={0}".format(is_raspberry_pi))
            has_usb_serial = os.path.exists("/dev/ttyUSB0")
            #print("has_usb_serial={0}".format(has_usb_serial))
        
            # This is really kudgey (technical term) for now:
            platform = "stage"
            if is_raspberry_pi:
                platform = "loki"
                if has_usb_serial:
                    platform = "magni"
            print("platform:={0}".format(platform))
        
        if __name__ == "__main__":
            main()

The file below is `.../ubiquity_launches/bin/urtest.sh` and shows how to
establish X11 forwarding and integrate it with platform probing prior
to doing the roslaunch:

        #!/bin/sh
        
        # Extract robot URL from ROS_MASTER_URI environment varaible:
        ROBOT_HOST=`echo $ROS_MASTER_URI | sed -e s,http://,, | sed -e s,:11311,,`
        #echo ROBOT_HOST=$ROBOT_HOST
        
        # Probe to find out what kind of platform the robot is:
        PLATFORM=`ssh ubuntu@$ROBOT_HOST /home/ubuntu/catkin_ws/src/ubiquity_launches/bin/platform_probe.py`
        #echo PLATFORM=$PLATFORM
        
        # Clear out file we use to indicate when the X11 channel is up:
        rm -f /tmp/x11_up
        
        # When we background the process, we want to be sure it gets killed 
        # off when we type control-C.  The following two trap commands do this:
        trap "exit" INT TERM
        trap "kill 0" EXIT
        
        # Now open the X11 channel:
        (ssh -Y ubuntu@$ROBOT_HOST \
          'echo export DISPLAY=$DISPLAY > /tmp/display.sh ; echo $DISPLAY; sleep 260' \
          1>&2) 2>/tmp/x11up &
        
        # Now we wait for something to be written into `/tmp/x11up` before
        # firing off the roslaunch:
        while [ ! -s /tmp/x11up ] ; \
            do sleep 1;             \
            done
        
        # Now we can run the roslaunch command specifying both the PLATFORM
        # and ROBOT_HOST:
        echo roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST
        roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST

The ROS launch file needs an enviroment variable loader file.  This is called
`ros_setup.sh` and it looks as follows:

        #!/bin/sh
        
        export DISPLAY=localhost:12.0
        export ROS_MASTER_URI=http://betty.local:11311
        export ROS_HOSTNAME=`hostname`.local
        while [ ! -f /tmp/display.sh ] ; do sleep 1 ; done
        . /tmp/display.sh
        . /home/ubuntu/catkin_ws/devel/setup.sh
        exec "$@"

The final location for this file is stilling being thought about.
Also, it is not clear that the `while` loop is needed any longer.

Lastly, here is what the `test.launch` file looks like:

        <launch>
          <!-- Required Arguments -->
          <arg name="robot_host" />
          <arg name="platform" />
        
          <machine name="robot"
           address="$(arg robot_host)" user="ubuntu"
           env-loader="/home/ubuntu/ros_setup.sh" />
        
          <node name="x11_channel" pkg="ubiquity_launches" type="x11_channel.sh" />
          <node machine="robot" name="robot_loop"
           pkg="ubiquity_launches" type="robot.py"
           launch-prefix="xterm -fn 9x15bold -e python -m pudb.run" />
        </launch>

