# Ubiquity_Main


This repository serves as a starting place where documents are kept
along with some of the shell scripts used by the Ubiquity team.

This repository also constains files used by the Ubiquity team
that have no other obvious place to be put.  For now,
this repository contains all of the documentation and shell
files used to the Ubiqutiy/Ubuntu/ROS Kernel image.

## Modifications to This README

If you read this documentation and it does not work for
you, please do the following:

1. Make a clone of this repository on you local desktop/laptop.

        cd /tmp           # Or someplace else of your choosing
        git clone https://github.com/UbiquityRobotics/ubiquity-misc.git
        cd ubiqutiy-misc

2. Edit this file (`README.md`) using your favorite editor.
   Insert you comments and/or questions directly into the file
   using the following format:

        > * ....
        > ....
        > .... -- {your_name} *

   By way of explanation, lines start with `>` are indented
   by the markdown processor.  The `*` causes the text to be
   italicized.

3. Stuff the question(s) back up to the repository:

        git add README.md
        git commit -m "Added some questions/comments."
        git push

## Manually Building a .deb from Scratch

The following commands show how to manually build a `.deb` from scratch:

        ################
        # Read:
        #  http://answers.ros.org/question/173804/generate-deb-from-ros-package/
        # Browse:
        #  https://wiki.debian.org/BuildingTutorial
        #  http://answers.ros.org/question/11315/creating-private-deb-packages-for-distribution/
        ################
        # Install required software:
        sudo apt-get install -y build-essential fakeroot devscripts equivs
        sudo apt-get install -y python-bloom gdebi-core
        ################
        # Start in the correct directory:
        cd .../catkin_ws/src/YOUR_PACKAGE # YOUR_PACKAGE==name of your package
        ################
        # Make sure that there is no `debian` directory
        rm -rf debian
        ################################################################
        # Run bloom to generate `debian` directory.  The `rosdebian`
        # argument will install files into /ros/indigo/...  In theory,
        # replacing `rosdebian` with `debian` will install into `/usr`,
        # but it does not seem to work that well:
        bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
        ################
        # Deal with dependencies:
        sudo mk-build-deps -i -r
        ################
        # Now build the package:
        fakeroot debian/rules clean
        fakeroot debian/rules binary
        ################
        # The package should be in `../ros-indigo-*.deb`.  It can be installed:
        sudo gdebi ../ros-indigo-YOUR-PACKAGE # Where YOUR-PACKAGE has '-', not '_'
	################
        # To totally remove, purge, and expunge:
        sudo apt-get purge ../ros-indigo-YOUR-PACKAGE


The Reprepro system is apparently used to deploy an apt-get repository
using reprepro:

* [https://wiki.debian.org/SettingUpSignedAptRepositoryWithReprepro](https://wiki.debian.org/SettingUpSignedAptRepositoryWithReprepro)

* [https://www.digitalocean.com/community/tutorials/how-to-use-reprepro-for-a-secure-package-repository-on-ubuntu-14-04](https://www.digitalocean.com/community/tutorials/how-to-use-reprepro-for-a-secure-package-repository-on-ubuntu-14-04)

## Scripts

### `mgit`

The `mgit` script a shell script that executes the same git command
across all all `src` sub-directories in a `catkin_ws` directory.
It can be executed executed in the `catkin_ws` directory or any
of the sub-directories under `catkin_ws`.

Examples:

List the status of each git repository the catkin workspace:

        mgit status

Pull the latest updates from all of the remote repositories:

        mgit pull

Perform a commit for all repositories that have had files
where a `git add` has been performed:

        mgit commit -m "..."

