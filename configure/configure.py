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

	wifis = []
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

				for x in NetworkManager.Settings.ListConnections():
					if x.GetSettings()['connection']['type'] == '802-11-wireless':
						wifi = Wifi(x)
						wifis.append(wifi)

				index = 0
				for wifi in wifis:
					index += 1
					print("[{0}]: Edit/delete ssid `{1}'". format(index, wifi.get_ssid()))

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
					wifi = WiFi()
					wifi.set_basic_settings(ssid, psk=psk)
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

		elif command == 3:
			if (old_hostname != hostname):
				print("Need root to save changes to hostname, calling sudo")
				returncode = subprocess.call(["/usr/bin/sudo", "./change_hostname.py", hostname_file_name, hosts_file_name, new_hostname])

			for wifi in wifis:
				wifi.save()

			break

		else:
		    print("Invalid comand")
	return 0

class Wifi:
	def __init__(self, connection=None):
		# Generate default UUID
		self.uuid = str(uuid.uuid4())
		self.connection = connection

		if (connection != None):
			if(connection.GetSettings()['connection']['type'] == '802-11-wireless'):
				self.ssid = connection.GetSettings()['802-11-wireless']['ssid']
				self.uuid = connection.GetSettings()['connection']['uuid']

	def set_basic_settings(self, ssid, psk=None):
		self.ssid = ssid
		self.psk = psk

	def get_ssid(self):
		return self.ssid

	def get_uuid(self):
		return self.uuid

	def mark_for_delete(self):
		self.delete_mark = True

	def delete_if_marked(self):
		self.connection.Delete()

	def save(self):
		if (self.connection != None):
			settings = self.connection.GetSettings()
			secrets = self.connection.GetSecrets()
			#Add secrets to connection settings
			for key in secrets:
				settings[key].update(secrets[key])

			if(settings['connection']['type'] == '802-11-wireless'):
				settings['802-11-wireless']['ssid'] = self.ssid
				if (self.psk != None):
					settings['802-11-wireless-security']['psk'] = self.psk

			self.connection.Update(settings)

		else:
			settings = {
				'connection': {
					'id': self.ssid,
					'type': '802-11-wireless',
					'uuid': self.uuid
				},

				'802-11-wireless': {
					'mode': 'infrastructure',
					'security': '802-11-wireless-security',
					'ssid': self.ssid 
				},

				'802-11-wireless-security': {
					'auth-alg': 'open', 
					'key-mgmt': 'wpa-psk',
					'psk': self.psk 
				},

				'ipv4': {'method': 'auto'},
				'ipv6': {'method': 'auto'}
			}

			NetworkManager.Settings.AddConnection(settings)
			self.connection = NetworkManager.Settings.GetConnectionByUuid(self.uuid)


if __name__ == "__main__":
	main()