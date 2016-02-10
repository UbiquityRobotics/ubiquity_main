#!/usr/bin/env python

# Copyright (c) 2016 by Rohan Agrawal.  All rights reserved.

import os
import os.path
import sys

import NetworkManager

for x in NetworkManager.Settings.ListConnections():
	if x.GetSettings()['connection']['type'] == '802-11-wireless':
		print x.GetSettings()['802-11-wireless']['ssid']

print ""
print ""

# print NetworkManager.NetworkManager.GetDevices()[1].DeviceType
# for x in NetworkManager.NetworkManager.GetDevices()[1].GetAccessPoints():
# 	print x.ssid

for device in NetworkManager.NetworkManager.GetDevices():
	if device.DeviceType != NetworkManager.NM_DEVICE_TYPE_WIFI:
		continue
	print("Visible on %s" % device.Udi[device.Udi.rfind('/')+1:])
	device = device.SpecificDevice()
	active = device.ActiveAccessPoint
	aps = device.GetAccessPoints()
	for ap in aps:
		prefix = '* ' if ap.object_path == active.object_path else '  '
		print("%s %s" % (prefix, ap.Ssid))
# print x.Interface