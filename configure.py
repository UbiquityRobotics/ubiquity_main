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

def main():

    # For debugging only:
    root = ""
    #root = "/tmp"
    
    # Make sure we are root:
    if os.geteuid() != 0:
	exit("You should run this command as `sudo configure.py`")

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

    # Read hosts file:
    hosts_file_name = "{0}/etc/hosts".format(root)
    if not os.path.isfile(hosts_file_name):
	print("File '{0}' does not exist!".format(hosts_file_name))
	return 1
    hosts_file = open(hosts_file_name, "ra")
    hosts_lines = hosts_file.readlines()
    hosts_file.close()

    # For debugging only:
    if False:
	for index in range(len(hosts_lines)):
	    hosts_line = hosts_lines[index].strip()
	    hosts_lines[index] = hosts_line
	    print("'{0}'".format(hosts_line))

    # Read in wpa_supplicant if it exists:
    conf_file_name = \
      "{0}/wpa_supplicant.conf".format(supplicant_directory_name)
    wifis = []
    if os.path.exists(conf_file_name):
	# We have a .conf file; read it in:
	conf_file = open(conf_file_name, "ra")
	conf_lines = conf_file.readlines()
	conf_file.close()

	# Delete blank lines:
        while len(conf_lines) > 0 and len(conf_lines[0].strip()) == 0:
	    del conf_lines[0]

	# Match `ctrl_interface` field:
	if len(conf_lines) > 0 and conf_lines[0]. \
	  startswith("ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev"):
	    del conf_lines[0]
	else:
	    print("control_interface not found in {0}".format(conf_file_name))
	    return 1

	# Match `update_config` field:
	if len(conf_lines) > 0 and conf_lines[0].startswith("update_config=1"):
	    del conf_lines[0]
	else:
	    print("update_config not found in {0}".format(conf_file_name))
	    return 1

	# Parse as many *WiFi* objects as possible:
	while len(conf_lines) > 0:
	    # Skip blank line:
	    if len(conf_lines[0].strip()) == 0:
		# Delete blank line:
		del conf_lines[0]
	    else:
	        wifi = wifi_parse(conf_lines)
		if wifi != None:
		    wifis.append(wifi)

	# We have the various network interfaces:
	stdout = sys.stdout
	for wifi in wifis:
	    wifi.write(stdout)

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
		for wifi in wifis:
		    index += 1
		    print("[{0}]: Edit/delete ssid `{1}'".
		      format(index, wifi.ssid))
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

		    # Create new *wifi* and append to *wifis*:
		    wifi = WiFi(comment=comment, ssid=ssid, psk=psk,
		      proto="RSN", key_mgmt="WPA-PSK", pairwise="CCMP",
		      auth_alg="OPEN", priority="5")
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
			print("[4] Delete entire '{0}' access point)".
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
	    # Save and exit:

            # Write out contents for *conf_file_name*:
	    conf_file = open(conf_file_name, "wa")
	    conf_file.write(
	     "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\n")
	    conf_file.write("update_config=1\n\n")
	    for wifi in wifis:
                wifi.write(conf_file)
	    conf_file.close()

	    interfaces_file_name = "{0}/etc/network/interfaces".format(root)
	    interfaces_file = open(interfaces_file_name, "wa")
	    interfaces_file.write(
	      "# Include files from /etc/network/interfaces.d:\n")
	    interfaces_file.write(
	      "source-directory /etc/network/interfaces.d\n\n")

	    interfaces_file.write(
	      "# The loopback network interface\n")
	    interfaces_file.write(
	      "auto lo\n")
	    interfaces_file.write(
	      "iface lo inet loopback\n\n")


	    eth_count = 6
	    interfaces_file.write(
	      "# Wired interface(s). {0} for 70-persistent-rules-net.rules\n".
	      format(eth_count))
	    for index in range(eth_count):
		interfaces_file.write(
		  "allow-hotplug eth{0}\n".format(index))
		interfaces_file.write(
		  "iface eth{0} inet dhcp\n".format(index))
                # Prioritize interface for gateway selection (the lower the metric,
                # the higher the priority):
	        #interfaces_file.write(
		#    "    metric {0}\n".format(100 + index))
	    interfaces_file.write("\n")

	    wlan_count = 6
	    interfaces_file.write(
	      "# WiFi Settings.  {0} for 70-persistent-rules-net.rules\n".
	      format(wlan_count))
	    for index in range(wlan_count):
		interfaces_file.write(
		  "allow-hotplug wlan{0}\n".format(index))
		interfaces_file.write(
		  "iface wlan{0} inet manual\n".format(index))
                # Prioritize interface for gateway selection (the lower the metric,
                # the higher the priority):
	        interfaces_file.write(
		    "    metric {0}\n".format(200 + index))
	        interfaces_file.write(
	          "wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf\n\n")

	    interfaces_file.write(
	      "# Get internet address via DHCP:\n")
	    interfaces_file.write(
	      "iface default inet dhcp\n\n")
	    interfaces_file.close()

	    # Write out the `/etc/hostname` file:
	    hostname_file = open(hostname_file_name, "wa")
	    hostname_file.write("{0}\n".format(new_hostname))
	    hostname_file.close()

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
	    hosts_file = open(hosts_file_name, "wa")
	    for hosts_line in hosts_lines:
		hosts_file.write(hosts_line)
		hosts_file.write('\n')
	    hosts_file.close()

	    break
	else:
	    print("Invalid comand")
    return 0

def wifi_parse(lines):
    assert isinstance(lines, list)

    # Specify the pattern match:
    comment_pattern = "# "
    network_pattern = "network={"
    ssid_pattern = "ssid=\""
    psk_pattern = "psk=\""
    proto_pattern = "proto="
    key_mgmt_pattern = "key_mgmt="
    pairwise_pattern = "pairwise="
    auth_alg_pattern = "auth_alg="
    priority_pattern = "priority="

    # Deal with multiple files by using a list for each field name:
    comments = []
    ssids = []
    psks = []
    protos = []
    key_mgmts = []
    pairwises = []
    auth_algs = []
    priorities = []

    error = False
    while len(lines) > 0:
	# Grab a line off the front of *lines*:
	line = lines[0].strip()
	del lines[0]

	if len(line) == 0:
	    # Empty lines are skipped:
	    pass
	elif line.startswith(comment_pattern):
	    # We have a comment:
	    comments.append(line[2:])
	    print("Comment='{0}'".format(line))
	elif line.startswith(network_pattern):
	    # We have a comment:
	    pass
        elif line.startswith(ssid_pattern) and line.endswith("\""):
	    # We have an ssid:
	    ssids.append(line[len(ssid_pattern):-1])
        elif line.startswith(psk_pattern) and line.endswith("\""):
	    # We have an psk:
	    psks.append(line[len(psk_pattern):-1])
        elif line.startswith(pairwise_pattern):
	    # We have an proto:
	    pairwises.append(line[len(pairwise_pattern):])
        elif line.startswith(proto_pattern):
	    # We have an proto:
	    protos.append(line[len(proto_pattern):])
        elif line.startswith(key_mgmt_pattern):
	    # We have an key_mgmt:
	    key_mgmts.append(line[len(key_mgmt_pattern):])
        elif line.startswith(auth_alg_pattern):
	    # We have an auth_alg:
	    auth_algs.append(line[len(auth_alg_pattern):])
        elif line.startswith(priority_pattern):
	    # We have an priority:
	    priorities.append(line[len(priority_pattern):])
	elif line == "}":
	    # We are done parsing WiFi object:
	    break
	else:
	    print("Unrecognized wpa_supplicant.conf line: '{0}'".format(line))
	    error = True

    if error:
	return None

    # Detemine WiFi *comment*:
    comment = "Unknown WiFi"
    if len(comments) > 0:
        comment = comments[0]
    print("comment=''{0}''".format(comment))

    # Determine *ssid*:
    ssid = "none"
    if len(ssids) == 0:
        print("No router name (ssid) specified for '{0}'".format(comment))
    elif len(ssids) > 1:
        print("Too many routiner names (ssid's) specified for '{0}'".
	  format(comment))
    else:
        ssid = ssids[0]

    # Determine *psk*:
    psk = "none"
    if len(psks) == 0:
        print("No password specified for '{0}'".format(comment))
	return None
    elif len(psks) > 1:
        print("Too many passwords specified for '{0}'".format(comment))
	return None
    else:
        psk = psks[0]

    # Determine *proto*:
    proto = "RSN"
    if len(protos) > 1:
        print("Too many proto fields specified for '{0}'".format(comment))
	return None
    elif len(protos) == 1:
        proto = protos[0]

    # Determine *key_mgmt*:
    key_mgmt = "WPA-PSK"
    if len(key_mgmts) > 1:
        print("Too many key_mgmt fields specified for '{0}'".format(comment))
	return None
    elif len(key_mgmt) == 1:
        key_mgmt = key_mgmts[0]

    # Determine *pairwise*:
    pairwise = "CCMP"
    if len(pairwises) > 1:
        print("Too many pairwise fields specified for '{0}'".format(comment))
	return None
    elif len(pairwises) == 1:
        pairwise = pairwises[0]

    # Determine *auth_alg*:
    auth_alg = "OPEN"
    if len(auth_algs) > 1:
        print("Too many auth_alg fields specified for '{0}'".format(comment))
	return None
    elif len(auth_algs) == 1:
        auth_alg = auth_algs[0]

    # Determine *priority*:
    priority = "5"
    if len(priorities) > 1:
        print("Too many priority fields specified for '{0}'".format(comment))
	return None
    elif len(priorities) == 1:
        priority = priorities[0]

    # Construct the *wifi* object:
    wifi = WiFi(comment=comment, ssid=ssid, psk=psk, proto=proto,
      key_mgmt=key_mgmt, pairwise=pairwise, auth_alg=auth_alg,
      priority=priority)

    return wifi

class WiFi:
    def __init__(self,
      comment, ssid, psk, proto, key_mgmt, pairwise, auth_alg, priority):
	# Verify argument types:
	assert isinstance(comment, str)
	assert isinstance(ssid, str)
	assert isinstance(psk, str)
	assert isinstance(proto, str)
	assert isinstance(key_mgmt, str)
	assert isinstance(pairwise, str)
	assert isinstance(auth_alg, str)
	assert isinstance(priority, str)

	# Load up *self*:
	self.comment = comment
	self.ssid = ssid
	self.psk = psk
	self.proto = proto
	self.key_mgmt = key_mgmt
	self.pairwise = pairwise
	self.auth_alg = auth_alg
	self.priority = priority

    def write(self, out_file):
	# Verify argument types:
        assert isinstance(out_file, file)

	# Write out *self*:
	out_file.write("# {0}\n".format(self.comment))
	out_file.write("network={\n")
	out_file.write("\tssid=\"{0}\"\n".format(self.ssid))
	out_file.write("\tpsk=\"{0}\"\n".format(self.psk))
	out_file.write("\tkey_mgmt={0}\n".format(self.key_mgmt))
	out_file.write("\tpairwise={0}\n".format(self.pairwise))
	out_file.write("\tauth_alg={0}\n".format(self.auth_alg))
	out_file.write("\tpriority={0}\n".format(self.priority))
	out_file.write("}\n")
	out_file.write("\n")

if __name__ == "__main__":
    main()

