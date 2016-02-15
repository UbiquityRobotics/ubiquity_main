---
layout: post
title: SSFS
---
Problem?
   We want to develop on the workstation laptop, not on the robot
Solution?
   Mount robot dir on workstation, with sshfs
Walkthough?
	Why not samba/nfs?
	Answer: overlay over existing file system + works
	mount src not catkin_ws
	do catkin_make on laptop and on robot to compile for both archs
	do git commit/push while mounted; unmount; git pull to rsynch laptop
