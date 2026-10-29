#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

"""A simple device status script for cockpit.

This script examines cockpit config. files, then reports
the host and port status for each remote device.
"""

import os
import platform
import re
import socket
import sys

import cockpit.config


# Strings used for IP address and port in config. files.
IP_STR = "ipaddress"  # ConfigParser makes keys lower case
PORT_STR = "port"
# String used to format output.
FORMAT_STR = "{:<20}  {:>16}  {:<8}  {:<6}"
# A list of special device types.
IGNORE_LIST = ["server"]


def ping(host):
    """
    Returns True if host responds to a ping request.
    """
    ping_str = "-n 1" if platform.system().lower() == "windows" else "-c 1"
    return os.system("ping " + ping_str + " " + host) == 0


def test_port(host, port):
    """
    Returns True if a port is open.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((host, int(port)))
    except socket.error:
        result = False
    else:
        result = True
    finally:
        s.close()
    return result


# Mappings of device to hosts and ports.
device_to_host = {}
device_to_port = {}
# Mappings of device to host and port statuses.
hosts_up = {}
devices_up = {}

skipped = []

config = cockpit.config.CockpitConfig(sys.argv).depot_config

# Iterate over config sections.
for s in config.sections():
    # Skip special devices.
    if s.lower() in IGNORE_LIST:
        skipped.append("skipped %s:  in ignore list" % s)
    # Skip devices that don't have remotes.
    if not any(map(lambda x: x in config.options(s), [IP_STR, "uri"])):
        skipped.append("skipped %s:  no host or uri" % s)
        continue
    if "uri" in config.options(s):
        uri = config.get(s, "uri")
        match = re.match(r"(.*@)?(.*):([0-9]+)?", uri)
        if match is None:
            skipped.append("skipped %s:  invalid uri; missing port?" % s)
            continue
        prefix, host, port = match.groups()
    else:
        host = config.get(s, IP_STR)
        try:
            port = config.get(s, PORT_STR)
        except:
            skipped.append("skipped %s:  IP with no port" % s)
            continue
    # Extract remote details from config and store in dicts.
    device_to_host[s] = host
    device_to_port[s] = port


# Iterate over the mappings to query host and port status.
for device, host in device_to_host.items():
    port = device_to_port[device]
    if host not in hosts_up.keys():
        hosts_up[host] = "up" if ping(host) else "down"
    devices_up[device] = "open" if test_port(host, port) else "closed"

# Report.
print("\n\n")
print(FORMAT_STR.format("DEVICE", "HOSTNAME", "STATUS", "PORT"))
print(FORMAT_STR.format("======", "========", "======", "======"))
for device in sorted(device_to_host.keys()):
    host = device_to_host[device]
    port = device_to_port[device]
    print(FORMAT_STR.format(device, host, hosts_up[host], devices_up[device]))
print("\n")
for s in skipped:
    print(s)
