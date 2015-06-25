#!/usr/bin/env python
# Copyright (c) 2015 by Ubiquity Robotics.  All rights reserved.

import yaml
import sys
import subprocess

def main():
    # Get the .yaml file to parse:
    argv = sys.argv
    if len(argv) < 2:
        print("usage: config.py config.yaml")
        return 1
    yaml_file_name = argv[1]

    # Open *yaml_file_name*:
    try:
        yaml_file = open(yaml_file_name, "ra")
    except:
        print("Unable to open {0}".format(yaml_file_name))
        return 1

    # Parse *file_name*
    try:
        yaml_tree = yaml.safe_load(yaml_file)
    except yaml.parser.ParserError as error:
        # Print out an error message:
        print("error={0}".format(error))
        return 1
    yaml_file.close()

    # For debugging only:
    #print("yaml_tree=", yaml_tree)

    try:
        hostname = yaml_tree["hostname"]
    except:
        print("No 'hostname' specified in file '{0}'".format(file_name))
        return 1

    try:
        wifis = yaml_tree["wifis"]
        assert isinstance(wifis, dict)
    except:
        print("No 'wifis' sprecified in file '{0}'".format(file_name))

    # Write out `/etc/hostname` file:
    try:
        hostname_file = open("/etc/hostname", "wa")
        hostname_file.write("{0}\n".format(hostname))
        hostname_file.close()
    except:
        print("Unable to write /etc/hostname")
        #return 1

    for key in wifis:
        # Grab the *ssid_password_pair*:
        ssid_password_pair = wifis[key]

        # For debugging only:
        #print("wifi=", wifi_key)
        #print("ssid_password_pair=", ssid_password_pair)

        # Extract *ssid* and *password*:
        try:
            ssid = ssid_password_pair["ssid"]
        except:
            print("No 'ssid:' specified for '{0}'".format(key))
            return 1
        try:
            password = ssid_password_pair["password"]
        except:
            print("No 'password:' specified for '{0}'".format(key))
            return 1
        # For debugging only:
        #print("key:={0} ssid={1} password={2}".format(key, ssid, password))
        
        # Execute a command to create a wifi entry:
        command = ["echo", "nmcli", "dev", "wifi",
	  "con", ssid, "password", password, "name", key]

	# For debugging only:
	#print("command={0}".format(command))
        try:
            subprocess.call(command)
        except:
            print("Command `{0}' failed".format(command))
            return 1

main()
