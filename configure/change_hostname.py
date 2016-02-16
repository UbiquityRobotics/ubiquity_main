#!/usr/bin/env python

# Copyright (c) 2016 by Wayne C. Gramlich, Rohan Agrawal.  All rights reserved.

import sys
import os

'''
Called as  change_hostname.py hostname_file hosts_file new_hostname
'''
assert isinstance(sys.argv[1], str)
assert isinstance(sys.argv[2], str)
assert isinstance(sys.argv[3], str)

hostname_filename = sys.argv[1]
hosts_filename = sys.argv[2]
new_hostname = sys.argv[3]

hostname_file = open(hostname_filename, "wa")
hostname_file.write("{0}\n".format(new_hostname))
hostname_file.close()


# Read hosts file:
if not os.path.isfile(hosts_filename):
	print("File '{0}' does not exist!".format(hosts_filename))
	#return 1
hosts_file = open(hosts_filename, "wa+")
hosts_lines = hosts_file.readlines()
#hosts_file.close()



# Sweep through *hosts_lines*:
have_localhost = False
have_hostname = False
have_zero_conf = False
insert_index = 0
for index in range(len(hosts_lines)):
	hosts_line = hosts_lines[index]
	if hosts_line.startswith("127.0.0.1"):
		# We have a loopback address:
		#print("old_host_line='{0}'".format(hosts_line))
		old_names = hosts_line.split()[1:]
		new_names = []
		for old_name in old_names:
			new_name = old_name
			if old_name.endswith("localhost"):
				have_localhost = True
				insert_index = index + 1
			elif old_name.endswith(".local"):
				new_name = "{0}.local".format(new_hostname)
				have_zero_conf = True
				insert_index = index + 1
			elif old_name == old_hostname:
				new_name = new_hostname
				have_hostname = True
				insert_index = index + 1
			else:
				pass
				new_names.append(new_name)
				host_line = ' '.join(["127.0.0.1"] + new_names)
				hosts_lines[index] = host_line
		#print("new_host_line='{0}'".format(host_line))

# Insert any missing bindings to loopback interface `127.0.0.1`:
if not have_localhost:
	hosts_lines.insert(insert_index, "127.0.0.1 localhost")
	insert_index += 1
	if not have_hostname:
		hosts_lines.insert(insert_index,
			"127.0.0.1 {0}".format(new_hostname))
		if not have_zero_conf:
			hosts_lines.insert(insert_index,
				"127.0.0.1 {0}.local".format(new_hostname))

# Write out the `/etc/hosts` file:
#hosts_file = open(hosts_filename, "wa")
for hosts_line in hosts_lines:
	hosts_file.write(hosts_line)
	hosts_file.write('\n')
hosts_file.close()