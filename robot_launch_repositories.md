# Robot Launch Repositories

A robot launch repository is a `git` repository that
primarily contains ROS `.launch` files and associated
robot configuration files (e.g. `.yaml`, `.urdf`, etc.)
A [ROS `.launch`](http://wiki.ros.org/roslaunch) file is
used to start and configure one or more
[ROS nodes](http://wiki.ros.org/Nodes), which when properly
configured will result in the desired robot behavior.

By imposing some structure on the organization of these
launch repositories, we improve the ability to reproduce
robot behaviors among all robot developers who share the
same robot launch repository.

## Overall Architecture

Ultimately, there are a bunch of shell scripts,
where each shell script sets a small number of
[environment variables](https://en.wikipedia.org/wiki/Environment_variable),
and then uses a ROS `roslaunch` command to initiate a
cascade of `.launch` files.  While environment variables
are not the most robust way of designing a system, ROS uses
them extensively and, more importantly, they are actually
supported inside of `.launch` files.  An example shell script
would be like the `loki_sonars` shell file shown below:

        #!/usr/bin/env bash
        export ROBOT_BASE=loki
        roslaunch ubiquity_launches sonars.launch

The environment variable `ROBOT_BASE` is used in various
`.launch` files to select configuration files that are
specific to a particular robot platform.  For example,
for robots named `freya`, `loki`, and `magni`, there
would be corresponding `freya.yaml`, `loki.yaml`, and
`magni.yaml` files.

The over structure of the launch repository is:

        REPO_NAME/
            CMakeLists.txt # Standard CMakeList for catkin_make
            Package.xml    # Standard ROS package manifest
            README.md      # Documentation for everything
            bin/           # Directory of symbolic links to shell scripts
            LAUNCH_DIR1    # First launch directory
            ...
            LAUNCH_DIRn    # n'th launch directory.

It is required that the `REPO_NAME/bin/` directory be added to
your `PATH` environment variable.  This allows you to type `loki_sonars`
(for example) and it fires off the loki version of sonars demo.

Each `LAUNCH_DIRi` directory is approximately structured as:

        LAUNCH_DIRi/
            launch/
                # One or more `.launch` files
            bin/
                # One or more shell scripts.  Usually one
                # script per robot platform.
            params/
                # One or more `.yaml` parameter files
            rviz/
                # One or more `.rviz` configuration files
            ...

It is not clear why there is an additional level of sub-directory,
but the turtlebot launch directories have this structure, so it
was decided to copy it.  There may be some subtle interaction
with [ROS `bloom`](http://wiki.ros.org/bloom) that is not yet
fully understood.

There are two flavors launch files:

* Inclusion launch files: These launch files have a file name
  suffix of `.launch.xml` and are meant to be accessed via an
  `<Include ... />` directive in another launch file.
  These launch files will typically launch a single ROS node,
  or a group of related nodes.

* Top level launch files: These launch files have a file name
  suffix of `.launch` and are expected to be accessible via
  the `ros_launch` command.  It is also permitted to access
  the launch files via the `<Include ... />` inside of other
  launch files.

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
directory the .launch file is in.  It is really that simple. 

We use some simple directory naming conventions to clue
people into what kind of launch file is in a directory.

* `n_`: If a launch directory starts with `n_`, the embedded
  launch file  starts a single ROS node.  The `n` in `n_` is
  short for Node.  Thus, `n_ros_arduino_bridge` starts the
  ROS Arduino Bridge ROS node.  An `n_` directory has an embedded
  launch file that ends in `.launch.xml` file suffix
  (i.e. `roslaunch` will not automatically find it.)

* `m_`: If a launch directory starts with `m_`, the embedded
  launch file fires up multiple ROS nodes.  The `m` in `m_` is
  short for Multiple.  This is usually done by including  other
  `n_` and `m_` directory based launch files.  Again, the file
  name suffix for `m_` directory based launch files is `launch.xml`.

* All others: All other launch directories are top level directories
  that are supposed to have .launch files that should be visible
  to `roslaunch`.  Thus, each embedded launch file ends with a
  `.launch.xml` file suffix.

It very important for debugging purposes that it be easy to
bring up (`rviz`)[http://wiki.ros.org/rviz].  Luckily, rviz
can be started using a `.launch` file as well.  It is strongly
encouraged that each top level launch directory have an associated
`rviz` launch file that brings up `rviz` properly configured to
view what is going on.  An example `rviz` launch file is shown
below.

## `n_` Launch Files

The ROS Arduino Bridge is used by a variety of different projects
to talk to an Arduino processor.  The directory consists of

        ubiquity_launches/
            n_robot_state_publisher/
                launch/
                    n_robot_state_publisher.launch.xml
                urdf/
                    loki.urdf
                    magni.urdf

The `n_robot_state_publisher.launch.xml` file looks as follows:

        <launch>
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="node" value="robot_state_publisher" />
          <arg name="base" value="$(env ROBOT_BASE)" />
          <param name="robot_state_publisher"
           textfile="$(arg root)/$n_(arg node)/urdf/$(arg base).urdf" />
          <node name="$(arg node)" pkg="$(arg node)" type="$(arg node)" />
        </launch>

## `m_` Launch Files

        <launch>
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="rsp" value="robot_state_publisher" />
          <arg name="ttj" value="teleop_twist_joy" />
          <include
           file="$(arg root)/n_$(arg rsp)/launch/n_$(arg rsp).launch.xml" />
          <include
           file="$(arg root)/n_joy/launch/n_joy.launch.xml") />
          <include
           file="$(arg root)/n_$(arg ttj)/launch/n_$(arg ttj).launch.xml") />
        </launch>

## Top Level Launch Files

        <launch>
          <arg name="root" value="$(find ubiquity_launches)" />
          <include
           file="$(arg root)/sonars/launch/m_sonars.launch.xml" />
        </launch>

## `rviz` launch files:

        <launch>
          <arg name="root" value=$(find ubiquity_launches)" />
          <node pkg="rviz" type="rviz" name="rviz"
           args="-d $(arg root)/sonars/rviz/sonars.rviz" />
        </launch>

