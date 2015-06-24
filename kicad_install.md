# Installing KiCAD

KiCAD is a free and open source program for doing schematic
capture and printed circuit board layout.  The KiCAD development
team found that providing "stable" binary releases was more
effort than they had resources for.  Thus, they encourage people
to download the source and build from source.

This document is were we will attempt to provide the steps for
downloading and building from source.  These instructions are
for Ubuntu only:

Perform the following steps:

1. Download some software that we need:

        sudo apt-get install build-essential gcc g++
        sudo cmake cmake-qu-qui
        # More packages are needed here
        sudo apt-get install mesa-common-dev  # OpenGL
        sudo apt-get install wx-common wx-gtk3.0-dev  # WxWidgets
        zlib1g-dev  # Zlib compresion library
        # Note: the lines above could have errors in them:

2. Download the source code:

        # Create a directory:
        cd ...
        mkdir kicad
        cd kicad
        # This is huge ~1.5GB and takes a while to download:
        git clone https://github.com/KiCad/kicad-source-mirror.git
        cd kicad-source-mirror

3. Fire up `cmake-gui`:

        mkdir -p build
        cmake-gui &
        # Fill in "Where is the source code:" as `.../kicad-source-mirror`
        # Fill in "Where to build the bianies:" as `.../kicad-source-mirror/build`
        # Click on the [Configure] button
        # Click on the [Generate] button
        # Exit cmake-gui

4. Build from source:

        cd build
        time make
        # This takes a while.  CMake prints out percentage estimates though.

5. If nothihng explodes, install the result

        sudo make install

6. Verify that KiCAD comes up.

        cd .../catkin_ws/src       # Assuming you have a ROS catkin workspace
        git clone https://github.com/UbiquityRobotics/bus_sonar10.git
        cd bus_sonar10/rev_d
        kicad bus_sonar10.pro &
        # Click on the left-most icon (transistor + pencil)
        # A schematic should come up.
        # Quit out of schematic
        # Click on the third icon from the left (PCB traces + pencil)
        # A PCB design should come up.
        # Quit out of PCB design

That should do it.

