#!/usr/bin/env python

# Copyright (c) 2016 by Rohan Agrawal.  All rights reserved.

import os
import os.path
import sys

import NetworkManager

import struct
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
		prefix = '* ' if active !="/" and ap.object_path == active.object_path else '  '
		print("%s %s" % (prefix, ap.Ssid))
# print x.Interface

c = NetworkManager.const

for conn in NetworkManager.NetworkManager.ActiveConnections:
    settings = conn.Connection.GetSettings()

    for s in list(settings.keys()):
        if 'data' in settings[s]:
            settings[s + '-data'] = settings[s].pop('data')

    secrets = conn.Connection.GetSecrets()
    for key in secrets:
        settings[key].update(secrets[key])

    devices = ""
    if conn.Devices:
        devices = " (on %s)" % ", ".join([x.Interface for x in conn.Devices])
    print("Active connection: %s%s" % (settings['connection']['id'], devices))
    size = max([max([len(y) for y in x.keys() + ['']]) for x in settings.values()])
    format = "      %%-%ds %%s" % (size + 5)
    for key, val in sorted(settings.items()):
        print("   %s" % key)
        for name, value in val.items():
            print(format % (name, value))
    for dev in conn.Devices:
        print("Device: %s" % dev.Interface)
        print("   Type             %s" % c('device_type', dev.DeviceType))
        # print("   IPv4 address     %s" % socket.inet_ntoa(struct.pack('L', dev.Ip4Address)))
        devicedetail = dev.SpecificDevice()
        if not callable(devicedetail.HwAddress):
            print("   MAC address      %s" % devicedetail.HwAddress)
        print("   IPv4 config")
        print("      Addresses")
        for addr in dev.Ip4Config.Addresses:
            print("         %s/%d -> %s" % tuple(addr))
        print("      Routes")
        for route in dev.Ip4Config.Routes:
            print("         %s/%d -> %s (%d)" % tuple(route))
        print("      Nameservers")
        for ns in dev.Ip4Config.Nameservers:
            print("         %s" % ns)