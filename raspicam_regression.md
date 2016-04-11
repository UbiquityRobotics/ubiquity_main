Raspicam Regression:

1. Load 2015-10-14-ubuntu-trusty.zip onto micro-SD card.

2. Resize the partition table using fdisk.

3. Resize the file system.

4. Install the swap file.

5. Run `~/catkin_ws/src/ubuquity_launches/loki_raspicam` .  It works.
   By the way, the GPU firmware revision is `260bc9c7589b3359485fc02fed8f56d4c5eaad9a`.

6. Do a apt-get update/upgrade.  (The Ubiquity Robotics server is down.)  Reboot.

7. Run `~/catkin_ws/src/ubuquity_launches/loki_raspicam` .  It still works.
   By the way, the GPU firmware revision is `260bc9c7589b3359485fc02fed8f56d4c5eaad9a`.

Current Hypothesis, the problem does not really appear to be a GPU firmware issue.
It is more likely to be a `ubiquity_launches` regression.

This is the status of each ROS package in `~/catkin_ws/src/`:

* `navigation_layers`:
        commit de3f6706e50c7a37e68b3abfd51a36323c35e726
        Merge: 596398c 6f04065
        Author: David Lu!! <davidvlu@gmail.com>
        Date:   Mon Jun 15 18:25:15 2015 -0400

            Merge pull request #23 from DLu/dynamic_recon
    
            Dynamic recon

* `raspicam_node`:
        commit 55a1a34e41d5bfe7789bebb6bc2e155a7942db20
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Oct 11 17:13:28 2015 -0700

            Figured out the firmware revision issue.

* `robot_model`:
        commit 19ac39ac7324237485a27664183442a93319a269
        Author: William Woodall <william@osrfoundation.org>
        Date:   Fri Sep 11 11:11:17 2015 -0400

            1.11.8

* `robot_state_publisher`:
        commit 0b0cf78c8e4c00666fd5882908388dad80e87fba
        Merge: 388dd24 49bca4e
        Author: David Lu!! <davidvlu@gmail.com>
        Date:   Mon Mar 30 20:33:36 2015 -0500

            Merge pull request #26 from xqms/remove-debug
    
            get rid of argv[0] debug output on startup

* `ubiquity_launches`:
        commit e311a7fadd80818565dcdefdb6aa7b288ef15588
        Author: Wayne Gramlich <wayne@gramlich.net>
        Date:   Wed Oct 7 03:18:15 2015 +0000

            Update loki.yaml.

* `ubiquity_main`:
        commit 7575c7ee65a337ec64a237a44f8083b65f8e7307
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Tue Oct 13 19:42:20 2015 -0700

            Changes to the system image document.

* `userland`:
        commit 40e377862410371a9962db79b81fd4f0f266430a
        Merge: cc92dfd 01d7835
        Author: popcornmix <popcornmix@gmail.com>
        Date:   Mon Sep 21 12:59:42 2015 +0100

            Merge pull request #258 from maxnet/master
    
            Proposing VC_DISPLAY env variable

Start changing things:

1. Run `catkin_make` on the packages above to provide a baseline.  The first time
   `loki_raspicam` did not work.  Ran `roscore` seperately, and it works.

2. Did a `git pull` on:

   * `navigation_layers`:
        Up-to-date (i.e. no changes!)

   * `robot_model`:
        commit a7c9e1b071018a66bf16b3f220131b907409e358
        Author: Jackie Kay <jackie@osrfoundation.org>
        Date:   Tue Feb 23 10:46:28 2016 -0800

            1.11.10

   * `robot_state_publisher`:
        commit b38ebed5a853673e6aed8411cfc37e895e909d73
        Author: Jackie Kay <jackie@osrfoundation.org>
        Date:   Mon Mar 14 12:39:00 2016 -0700

        Restore default argument of tf_static

   * `sudo apt-get install ros-indigo-tf2-kdl`

   With `roscore` running, did a `catkin_make` followed by `loki_raspicam`.
   The first run failed, but the second succeeded.

3. Updated to versions of `raspicam_node` and `user_land` from raspberrypi.org:

   * `raspicam_node`:
        commit 44cb0940e5ce37b0e4ce35571cb8220bf74e8154
        Author: Rohan Agrawal <send2arohan@gmail.com>
        Date:   Wed Mar 9 07:45:31 2016 -0800

            Make structs char const instead of char
            Prevents lots of compile warnings

   * `userland`:
        commit 703a2c4b35e23ee44ad84db6b9c3f89c0a627143
        Author: Phil Elwell <phil@raspberrypi.org>
        Date:   Wed Mar 30 13:27:40 2016 +0100

            dtoverlay: Compile libfdt into libdtovl
    
            Compile the static library libfdt into libdtovl, rather than deferring
            its linking to clients of libdtovl.

   This failed!

4. Updated to versions of `raspicam_node` and `user_land` from
   `https://github.com/UbiquityRobotics/`:

   * `raspicam_node`:
        commit 44cb0940e5ce37b0e4ce35571cb8220bf74e8154
        Author: Rohan Agrawal <send2arohan@gmail.com>
        Date:   Wed Mar 9 07:45:31 2016 -0800

            Make structs char const instead of char
            Prevents lots of compile warnings

   * `userland`:
        commit 2a4af2192c0e161555fdb2a12e902b587166c4a6
        Merge: 0863709 cd6da13
        Author: popcornmix <popcornmix@gmail.com>
        Date:   Tue Feb 2 21:34:59 2016 +0000

            Merge pull request #287 from jasaw/fix_warnings
    
            RaspiStill: Fix compiler warnings

  This failed!


5. Reverted to original `raspicam_node` and `user_land` node:

   * `raspicam_node`:
        commit 55a1a34e41d5bfe7789bebb6bc2e155a7942db20
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Oct 11 17:13:28 2015 -0700

            Figured out the firmware revision issue.

   * `userland`:
        commit 40e377862410371a9962db79b81fd4f0f266430a
        Merge: cc92dfd 01d7835
        Author: popcornmix <popcornmix@gmail.com>
        Date:   Mon Sep 21 12:59:42 2015 +0100

            Merge pull request #258 from maxnet/master
    
            Proposing VC_DISPLAY env variable

   This worked!

6. Try to update `ubiquity_main` and `ubiquity_launches`:

   * `ubiquity_main`:
        commit 35e5f9a54e8cc495eb5bf16c1d701f72921d74d9
        Author: Joe Landau <jrlandau@verizon.net>
        Date:   Sun Mar 13 18:59:54 2016 -0700

            install Lubuntu instead of Ubuntu


   * `ubiquity_launches`:
        commit 898d715ae1cff2b1763d67fc8605d38ecb104f85
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Apr 3 08:43:08 2016 -0700

            Use ~/.bashrc instead of ~/.profile.

   * Add `loki_robot`:
        commit 33c8b84c9e0a6bf3ded7a5b26d75216f8a9330bb
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Mar 13 15:28:33 2016 -0700

            Got keyboard_drive to work for Loki.

   * Did:

        mkdir ~/.ros/ubiquity
        echo `!!` > /home/ubuntu/.ros/ubiquity/robot_dir_name.txt
        (cd /home ; sudo ln ubuntu wayne; ls -l)
        # Created ~/.ros_setup
        # Added `source ~/.ros_setup` to `~/.bashrc` at beginning

   Not working yet!

7. Revert `ubiquity_launches` back to October:

   * `ubiquity_launches`:
        commit e311a7fadd80818565dcdefdb6aa7b288ef15588
        Author: Wayne Gramlich <wayne@gramlich.net>
        Date:   Wed Oct 7 03:18:15 2015 +0000

            Update loki.yaml.

    * Run the node manually:
        rosrun raspicam raspicam_node _frame_rate:=30 _quality:=100
        rosservice call /raspicam_node/camera/start_capture

    Works!

The summary appears to be to keep `raspicam_node` and `userland` back
at the October time-frame.  All of the other ROS packages can be
at the lastest master.  The code for launching `raspicam_node`
from the latest `ubiquity_launches` appears to be broken.  It may
be the missing rosservice call.

The following command:

        for d in [a-z]* ; do echo $d ; (cd $d ; pwd; git log | head -3; echo ""); done

returns:

        loki_robot
        /home/ubuntu/catkin_ws/src/loki_robot
        commit 33c8b84c9e0a6bf3ded7a5b26d75216f8a9330bb
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Mar 13 15:28:33 2016 -0700

        navigation_layers
        /home/ubuntu/catkin_ws/src/navigation_layers
        commit de3f6706e50c7a37e68b3abfd51a36323c35e726
        Merge: 596398c 6f04065
        Author: David Lu!! <davidvlu@gmail.com>
        Date:   Mon Jun 15 18:25:15 2015 -0400

        raspicam_node
        /home/ubuntu/catkin_ws/src/raspicam_node
        commit 55a1a34e41d5bfe7789bebb6bc2e155a7942db20
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Oct 11 17:13:28 2015 -0700         <====

        robot_model
        /home/ubuntu/catkin_ws/src/robot_model
        commit a7c9e1b071018a66bf16b3f220131b907409e358
        Author: Jackie Kay <jackie@osrfoundation.org>
        Date:   Tue Feb 23 10:46:28 2016 -0800

        robot_state_publisher
        /home/ubuntu/catkin_ws/src/robot_state_publisher
        commit b38ebed5a853673e6aed8411cfc37e895e909d73
        Author: Jackie Kay <jackie@osrfoundation.org>
        Date:   Mon Mar 14 12:39:00 2016 -0700

        ubiquity_launches
        /home/ubuntu/catkin_ws/src/ubiquity_launches
        commit 898d715ae1cff2b1763d67fc8605d38ecb104f85
        Author: Wayne C. Gramlich <Wayne@Gramlich.Net>
        Date:   Sun Apr 3 08:43:08 2016 -0700

        ubiquity_main
        /home/ubuntu/catkin_ws/src/ubiquity_main
        commit 35e5f9a54e8cc495eb5bf16c1d701f72921d74d9
        Author: Joe Landau <jrlandau@verizon.net>
        Date:   Sun Mar 13 18:59:54 2016 -0700

        userland
        /home/ubuntu/catkin_ws/src/userland
        commit 40e377862410371a9962db79b81fd4f0f266430a
        Merge: cc92dfd 01d7835
        Author: popcornmix <popcornmix@gmail.com>
        Date:   Mon Sep 21 12:59:42 2015 +0100         <====

Actually, both `rosrun ubiquity_launches raspicam` and
`rosrun ubiquity_launches raspicam_view` both seem to be working now.

## Summary

I labeled a branch on both `raspicam_node` and `userland` called
`indigo-safe` that pulls the correct commit for the two nodes.

So, the commands that should put everything right are:

        # On the robot:
        rm -f ~/catkin_sw/src/raspicam_node
        rm -f ~/catkin_sw/src/userland
        git clone https://github.com/UbiquityRobotics/userland.git
        get clone https://github.com/UbiquityRobotics/raspicam_node.git
        (cd ~/catkin_sw/src/raspicam_node ; git checkout indigo_safe)
        (cd ~/catkin_sw/src/userland ; git checkout indigo_safe)
        (cd ~/catkin_sw ; catkin_make)
        more /boot/.firmware_revision
        # Should print out: 260bc9c7589b3359485fc02fed8f56d4c5eaad9a

To run :

        rosrun raspicam raspicam_node _frame_rate:=30 _quality:=100
        rosservice call /raspicam_node/camera/start_capture
