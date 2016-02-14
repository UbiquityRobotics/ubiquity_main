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

import subprocess

def main():

    # For debugging only:
    #root = ""
    root = "/tmp"
    
    # Make sure we are root:
    #if os.geteuid() != 0:
	#exit("You should run this command as `sudo configure.py`")

    # Make sure that `*root*/etc/network/interfaces` exists:
    interfaces_file_name = "{0}/etc/network/interfaces".format(root)
    if not os.path.isfile(interfaces_file_name):
	print("The file '{0}' does not exist".format(interfaces_file_name))
	return 1
   
    # Make sure `*root*/etc/wpa_supplicant` directory exists:
    supplicant_directory_name = "{0}/etc/wpa_supplicant".format(root)
    if not os.path.exists(supplicant_directory_name):
	try:
	    os.mkdir(supplicant_directory_name)
	except:
	    print("Unable to create '{0}' directory".
	      format(suplicant_directory_name))
	    return 1
    assert os.path.exists(supplicant_directory_name)

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
				print("[{0}]: Edit/delete ssid `{1}'". format(index, x.GetSettings()['802-11-wireless']['ssid']))
		add_command = index + 1
		print("[{0}]: Add new Wifi access point". \
		    format(add_command))
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
		    comment = \
		      raw_input("Description (i.e. location): ").strip()
		    ssid = \
		      raw_input("SSID (i.e. access point name): ").strip()
		    psk = \
		      raw_input("Access Point Password: ").strip()
		    priority = \
		      raw_input("Priority (1=low, 5=average, 9=high): ").strip()

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
			print("[1] Edit WiFi Description (currently '{0}')".
			  format(wifi.comment))
			print("[2] Edit WiFi Name (currently '{0})'".
			  format(wifi.ssid))
			print("[3] Edit Wifi Password (currently '{0}')".
			  format(wifi.psk))
			print("[4] Edit Wifi priority (currently '{0}')".
			  format(wifi.priority))
			print("[5] Delete entire '{0}' access point)".
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
<<<<<<< HEAD
	    returncode = subprocess.call(["/usr/bin/sudo", "./hostname.py", hostname_file_name, hosts_file_name, new_hostname])
=======
	 #    # Save and exit:

  #           # Write out contents for *conf_file_name*:
	 #    conf_file = open(conf_file_name, "wa")
	 #    conf_file.write(
	 #     "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\n")
	 #    conf_file.write("update_config=1\n\n")
	 #    for wifi in wifis:
  #               wifi.write(conf_file)
	 #    conf_file.close()

	 #    interfaces_file_name = "{0}/etc/network/interfaces".format(root)
	 #    interfaces_file = open(interfaces_file_name, "wa")
	 #    interfaces_file.write(
	 #      "# Include files from /etc/network/interfaces.d:\n")
	 #    interfaces_file.write(
	 #      "source-directory /etc/network/interfaces.d\n\n")

	 #    interfaces_file.write(
	 #      "# The loopback network interface\n")
	 #    interfaces_file.write(
	 #      "auto lo\n")
	 #    interfaces_file.write(
	 #      "iface lo inet loopback\n\n")


	 #    eth_count = 6
	 #    interfaces_file.write(
	 #      "# Wired interface(s). {0} for 70-persistent-rules-net.rules\n".
	 #      format(eth_count))
	 #    for index in range(eth_count):
		# interfaces_file.write(
		#   "allow-hotplug eth{0}\n".format(index))
		# interfaces_file.write(
		#   "iface eth{0} inet dhcp\n".format(index))
		# # Prioritize interface for gateway selection (the lower the metric,
		# # the higher the priority):
		# #interfaces_file.write(
		# #  "    up ifmetric eth{0} {1}\n".format(index, 100 + index))
	 #    interfaces_file.write("\n")

	 #    wlan_count = 6
	 #    interfaces_file.write(
	 #      "# WiFi Settings.  {0} for 70-persistent-rules-net.rules\n".
	 #      format(wlan_count))
	 #    for index in range(wlan_count):
		# interfaces_file.write(
		#   "allow-hotplug wlan{0}\n".format(index))
		# interfaces_file.write(
		#   "iface wlan{0} inet manual\n".format(index))
		# # Prioritize interface for gateway selection (the lower the metric,
		# # the higher the priority):
  #               #interfaces_file.write(
		# #    "    up ifmetric wlan{0} {1}\n".format(index, 200 + index))
		# interfaces_file.write(
		#   "wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf\n\n")

	 #    interfaces_file.write(
	 #      "# Get internet address via DHCP:\n")
	 #    interfaces_file.write(
	 #      "iface default inet dhcp\n\n")
	 #    interfaces_file.close()

	    returncode = subprocess.call(["/usr/bin/sudo", "./hostname.py", hostname_file_name, new_hostname])

	 #    # Sweep through *hosts_lines*:
	 #    have_localhost = False
	 #    have_hostname = False
	 #    have_zero_conf = False
	 #    insert_index = 0
	 #    for index in range(len(hosts_lines)):
		# hosts_line = hosts_lines[index]
		# if hosts_line.startswith("127.0.0.1"):
		#     # We have a loopback address:
		#     #print("old_host_line='{0}'".format(hosts_line))
		#     old_names = hosts_line.split()[1:]
		#     new_names = []
		#     for old_name in old_names:
		# 	new_name = old_name
		# 	if old_name.endswith("localhost"):
		# 	    have_localhost = True
		# 	    insert_index = index + 1
		# 	elif old_name.endswith(".local"):
		# 	    new_name = "{0}.local".format(new_hostname)
		# 	    have_zero_conf = True
		# 	    insert_index = index + 1
  #                       elif old_name == old_hostname:
		# 	    new_name = new_hostname
		# 	    have_hostname = True
		# 	    insert_index = index + 1
		# 	else:
		# 	    pass
		# 	new_names.append(new_name)
		#     host_line = ' '.join(["127.0.0.1"] + new_names)
		#     hosts_lines[index] = host_line
		#     #print("new_host_line='{0}'".format(host_line))

	 #    # Insert any missing bindings to loopback interface `127.0.0.1`:
	 #    if not have_localhost:
		# hosts_lines.insert(insert_index, "127.0.0.1 localhost")
  #               insert_index += 1
	 #    if not have_hostname:
		# hosts_lines.insert(insert_index,
		#   "127.0.0.1 {0}".format(new_hostname))
	 #    if not have_zero_conf:
		# hosts_lines.insert(insert_index,
		#   "127.0.0.1 {0}.local".format(new_hostname))

	 #    # Write out the `/etc/hosts` file:
	 #    hosts_file = open(hosts_file_name, "wa")
	 #    for hosts_line in hosts_lines:
		# hosts_file.write(hosts_line)
		# hosts_file.write('\n')
	 #    hosts_file.close()

>>>>>>> network_manager
	    break

	else:
	    print("Invalid comand")
    return 0

if __name__ == "__main__":
    main()

