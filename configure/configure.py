#!/usr/bin/env python

# Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

# This program is used to configure a robot system.
#
# Currently, this program permits the following:
#
# * Changing the host name.
#
# * Configuring WiFi

import os
import os.path
import sys

import NetworkManager
import uuid

import subprocess

def main():

    # For debugging only:
    #root = ""
    root = "./"
    
    # Make sure we are root:
    #if os.geteuid() != 0:
	#exit("You should run this command as `sudo configure.py`")

    # Make sure that `*root*/etc/network/interfaces` exists:
    interfaces_file_name = "{0}/etc/network/interfaces".format(root)
    if not os.path.isfile(interfaces_file_name):
	print("The file '{0}' does not exist".format(interfaces_file_name))
	return 1
   
    # Read the hostname:
    hostname_file_name = "{0}/etc/hostname".format(root)
    if not os.path.isfile(hostname_file_name):
	print("File '{0}' does not exist!".format(hostname_file_name))
	return 1
    hostname_file = open(hostname_file_name, "ra")
    old_hostname = hostname_file.read().strip()
    hostname_file.close()
    new_hostname = old_hostname
    #print("hostname='{0}'".format(old_hostname))

    hosts_file_name = "{0}/etc/hosts".format(root)



    # For debugging only:
    if False:
	for index in range(len(hosts_lines)):
	    hosts_line = hosts_lines[index].strip()
	    hosts_lines[index] = hosts_line
	    print("'{0}'".format(hosts_line))

    # This is the user edit menu tree:
    while True:
	# List possibilities and prompt for input command:
	print("")
	print("[0]: Exit without save")
	print("[1]: Change current hostname ('{0}')".format(new_hostname))
	print("[2]: Manage hostname WiFi access points")
	print("[3]: Save everything and exit")
	try:
		command = int(raw_input("Command: ").strip())
	except:
		command = 999999

	# Dispatch on command:
	if command == 0:
	    # Exit without save:
	    break
	elif command == 1:
	    # Manage the hostname:
	    new_hostname = raw_input("New Hostname: ").strip()
	elif command == 2:
	    # Manage Wifi access points:
	    while True:
			# List possibilities and prompt for input command:
			print("")
			print("[0]: Exit WiFi access point mode")

			index = 0
			for x in NetworkManager.Settings.ListConnections():
				if x.GetSettings()['connection']['type'] == '802-11-wireless':
					index += 1
					print("[{0}]: Edit/delete ssid `{1}'". \
						format(index, x.GetSettings()['802-11-wireless']['ssid']))
			add_command = index + 1

			print("[{0}]: Add new Wifi access point". \
			    format(add_command))

			# Get Command
			try:
			    command = int(raw_input("Wifi Command: ").strip())
			except:
			    command = 99999

			# Process *command*:
			if command == 0:
			    # Go up one level:
			    break
			elif command == add_command:
			    # Get the new values:
			    print("")
			    print("Enter new WiFi access point values:")
			    ssid = \
			      raw_input("SSID (i.e. access point name): ").strip()
			    psk = \
			      raw_input("Access Point Password: ").strip()

			    # Create new *wifi* and append to *wifis*:
			    wifi = WiFi(comment=comment, ssid=ssid, psk=psk,
			      proto="RSN", key_mgmt="WPA-PSK", pairwise="CCMP",
			      auth_alg="OPEN", priority=priority)
			    wifis.append(wifi)
			elif 0 < command < add_command:
			    # Prompt for Wifi command:
			    wifis_index = command - 1
			    wifi = wifis[wifis_index]
			    while True:
				print("")
				print("Edit WiFi '{0}' access point".
				  format(wifi.ssid))
				print("[0] Done editing this Wifi access point")
				print("[1] Edit WiFi Name (currently '{0})'".
				  format(wifi.ssid))
				print("[2] Edit Wifi Password (currently '{0}')".
				  format(wifi.psk))
				print("[3] Delete entire '{0}' access point)".
				  format(wifi.ssid))
				try:
				    command = int(raw_input("Command: ").strip())
				except:
				    command = 999999

				# Dispatch on command:
	                        if command == 0:
				    # Done editing:
				    break
	                        elif command == 1:
				    # Edit description
				    wifi.comment = \
				      raw_input("New Description: ").strip()
				elif command == 2:
				    # Edit name:
				    wifi.ssid = \
				      raw_input("New name (SSID): ").strip()
				elif command == 3:
				    # Edit password:
				    wifi.psk = raw_input("New Password: ").strip()
				elif command == 4:
				    # Edit priority:
				    wifi.priority = raw_input("New priority: ").strip()
				elif command == 5:
				    # Delete entire wifi object:
				    del wifis[wifis_index]
				    break
	                        else:
				    print("Invalid command")
			else:
			    print("Invalid command")
	    else:
		print("Invalid command")
	elif command == 3:
		print("Need root to save changes to hostname, calling sudo")
		returncode = subprocess.call(["/usr/bin/sudo", "./change_hostname.py", hostname_file_name, hosts_file_name, new_hostname])
		break

	else:
	    print("Invalid comand")
    return 0

if __name__ == "__main__":
    main()