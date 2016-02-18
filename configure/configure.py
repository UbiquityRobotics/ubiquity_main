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

try:
	import NetworkManager
except:
	print("Please install NetworkManager and python-networkmanager")
	print("sudo apt-get install network-manager python-networkmanager")
	sys.exit(0)

import uuid

import subprocess

def main():

	# For debugging only:
	#root = ""
	root = "./"

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
	for x in NetworkManager.Settings.ListConnections():
		if x.GetSettings()['connection']['type'] == '802-11-wireless':
			wifi = Wifi(x)
			wifis.append(wifi)

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
					wifi = Wifi()
					wifi.set_basic_settings(ssid, psk=psk)
					wifis.append(wifi)

				elif 0 < command < add_command:
					# Prompt for Wifi command:
					wifis_index = command - 1
					wifi = wifis[wifis_index]
					while True:
						print("")
						print("Edit WiFi '{0}' access point".
							format(wifi.get_ssid()))
						print("[0] Done editing this Wifi access point")
						print("[1] Edit WiFi Name (currently '{0})'".
							format(wifi.get_ssid()))
						print("[2] Change Wifi Password")
						print("[3] Delete entire '{0}' access point)".
							format(wifi.get_ssid()))
						try:
							command = int(raw_input("Command: ").strip())
						except:
							command = 999999

						# Dispatch on command:
						if command == 0:
							# Done editing:
							break
						elif command == 1:
							# Edit name:
							ssid = raw_input("New name (SSID): ").strip()
							wifi.set_basic_settings(ssid)
						elif command == 2:
							# Edit password:
							psk = raw_input("New Password: ").strip()
							wifi.set_basic_settings(wifi.get_ssid(), psk=psk)
						elif command == 3:
							pass
						else:
							print("Invalid command")
				else:
					print("Invalid command")

		elif command == 3:
			if (old_hostname != new_hostname):
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

		self.ssid = ''
		self.psk = None
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

			try:
				secrets = self.connection.GetSecrets()
				#Add secrets to connection settings
				for key in secrets:
					settings[key].update(secrets[key])
			except Exception, e:
				print ("no secrets for: " + settings['connection']['id'])

			if(settings['connection']['type'] == '802-11-wireless'):
				settings['connection']['id'] = self.ssid
				settings['802-11-wireless']['ssid'] = self.ssid
				if (self.psk != None):
					settings['802-11-wireless-security']['psk'] = self.psk

			if(settings['ipv4']['method'] == 'auto'):
				del settings['ipv4']
				settings['ipv4'] = {'method': 'auto'}
			if(settings['ipv6']['method'] == 'auto'):
				del settings['ipv6']
				settings['ipv6'] = {'method': 'auto'}

			try:
				self.connection.Update(settings)
			except:
				print("self.connection.Update(settings) Failed")
				print("Make sure you have the policykit file installed, and that you are in the netdev group")
				print("Outlined here https://github.com/UbiquityRobotics/ubiquity_main/issues/5#issuecomment-184276930")
				sys.exit(0)

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

			try:
				NetworkManager.Settings.AddConnection(settings)
			except:
				print("NetworkManager.Settings.AddConnection(settings) Failed")
				print("Make sure you have the policykit file installed, and that you are in the netdev group")
				print("Outlined here https://github.com/UbiquityRobotics/ubiquity_main/issues/5#issuecomment-184276930")
				sys.exit(0)
			self.connection = NetworkManager.Settings.GetConnectionByUuid(self.uuid)


if __name__ == "__main__":
	main()