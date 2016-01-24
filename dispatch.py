#!/usr/bin/env python

import os
import os.path

# http://answers.ros.org/question/12434/using-roslaunch-to-run-distributed-systems-over-ssh/
# http://answers.ros.org/question/36600/how-to-tell-remote-machines-not-to-start-separate-roscores-when-using-roslaunch-fuerte-regression/
#http://answers.ros.org/question/10725/question-about-roslaunch-and-remote-processes/
def main():
    environ = os.environ
    #print("environ={0}".format(environ))
    home_host_name = "none!"
    if "ROS_HOSTNAME" in environ:
	ros_hostname = environ["ROS_HOSTNAME"]
	print("ros_home='{0}'".format(ros_hostname))
	if ros_hostname.endswith(".local"):
	    home_host_name = ros_hostname[:-6]
	    print("home_host_name={0}".format(home_host_name))

    robot_host_name = "none!!!"
    if "ROS_MASTER_URI" in environ:
	ros_master_uri = environ["ROS_MASTER_URI"]
	print("ros_master_uri={0}".format(ros_master_uri))
	if ros_master_uri.startswith("http://") and \
	  ros_master_uri.endswith(".local:11311"):
	    robot_host_name = ros_master_uri[7:-12]
	    print("robot_host_name={0}".format(robot_host_name))

    is_raspberry_pi = os.path.isfile("/dev/ttyACA0")
    print("is_raspberry_pi={0}".format(is_raspberry_pi))
    has_usb_serial = os.path.isfile("/dev/ttyUSB0")

    if home_host_name != robot_host_name:
	print("We are running on a laptop {0} to control robot {1}".
	  format(home_host_name, robot_host_name))
    elif is_raspberry_pi:
	platform = "stage"
	if is_raspberry_pi:
	    if has_usb_serial:
		platform = "magni"
	    else:
		platform = "loki"
	print("We are on robot {0} which is a {1} platform".
	  format(robot_host_name, platform))
    else:
	print("We are running on a lapto {0} to control stage robot".
	  format(home_host_name))

if __name__ == "__main__":
    #print("hello")
    main()
