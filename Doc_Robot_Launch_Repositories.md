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
on the robot, but show debugger running in a window the
laptop/desktop.  This is usally an `xterm` window.

The trick is to get the silly X11 security stuff out of the way
so that `xterm` will work.  This is done using `ssh` with the
`-X` option specified as follows:

        ssh -X user@robot.local command arg1 ... argN

which sets some environment variables like:

 * XDG_SESSION_COOKIE
 * SSH_CLIENT
 * SSH_TTY
 * SSH_CONNECTION
 * DISPLAY

and probably a few more that matter.  Once these variables are
set, when the `xterm` program is run on the robot, it will pop
a window up on the desktop/laptop.

In order for the `-X` option to work, we need to ensure that
the Ubuntu `xorg` and `xauth` packages are installed on the
laptop/desktop:

        sudo apt-get install xorg xauth

In addition, it is necessary to ensure that `/etc/ssh/ssh_config`
is edited to enable `ForwardX11` and `ForwardX11Trusted`.

        ForwardX11 yes
        ForwardX11Trusted yes

It may be necessary to configure `/etc/ssh/ssh_config` to
enable `ForwaredX11 yes`, `ForwardX11Trused yes`, etc.  Frankly,
given how mature X11 is, the documentation that explains all of
this is still quite gnarly.

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

The original plan was to simply mount the robot catkin workspace
on the laptop using a network file sharing protocol like NFS, Samba,
sshfs, etc.  The plan was to mount the robot catkin workspace
directly onto the catkin workspace on the laptop.  The problem
with this idea is that the laptop/desktop will almost certainly
be running some form x86 processor architecture, whereas the robot
will probably be running some sort of ARM processor architecture.
Total chaos will result if `catkin_make` is run on the same workspace
with different processor architectures.  We need something a little
more robust.

The next proposal is to have each laptop/desktop and each robot
have a directory called `~/willow` directory.
For each desktop/laptop and for each robot, we could have
a directory under `~/willow`.  Something like:

        ~/willow/laptop1/catkin_ws
        ...
        ~/willow/loptopN/catkin_ws
        ~/willow/robot1/catkin_ws
        ...
        ~/willow/robotM/catkin_ws

Where `laptopI` is the host name for the I'th laptop
and `robotI` is the host name for the I'th robot.

We should put a wrapper around catkin_make to ensure that
we are running the correct processor architecture for a
given catkin workspace.

Of the various network file system protocols out there,
the `sshfs` seems to be the one to use.  To set up `sshf`,
do the following:

        sudo apt-get install sshfs
        sudo gpasswd -a $USER fuse

To mount xxx:

        sshfs ubuntu@robotI.local:/home/ubuntu/willow/robotI/catkin_ws /home/`whoami`/willow/robotI/catkin_ws

To unmount:

        fusermount -u /home/ubuntu/willow/robotI/catkin_ws

There is more documentation on
[installing SSHFS](https://help.ubuntu.com/community/SSHFS)
available.

## Platform Neutral Launch Files

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

