#-
# Copyright (c) 2010-2012 Aleksandr Rybalko <ray@ddteam.net>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# $FreeBSD$
#

#include <sys/bus.h>
#include <machine/bus.h>
#include <dev/switch/switchvar.h>

INTERFACE switch;

CODE {
	static struct switch_capability *
	null_get_caps(device_t dev)
	{
		return (0);
	}

	static int
	null_get_reg(device_t dev, uint32_t reg, uint32_t *value)
	{
		return (0);
	}

	static int
	null_set_reg(device_t dev, uint32_t reg, uint32_t *value)
	{
		return (0);
	}

	static int
	null_find_mac(device_t dev, uint64_t mac)
	{
		return (0);
	}

	static int
	null_mac_write (device_t dev, uint64_t mac, int index,
	    uint32_t port_map, uint8_t age, int *hash_idx)
	{
		return (0);
	}

	static int
	null_set_port_vid (device_t dev, int port, uint16_t pvid)
	{
		return (0);
	}

	static int
	null_get_port_vid (device_t dev, int port, uint16_t *pvid)
	{
		return (0);
	}

	static int
	null_set_port_flags (device_t dev, int port, uint32_t flags)
	{
		return (0);
	}

	static int
	null_get_port_flags (device_t dev, int port, uint32_t *flags)
	{
		return (0);
	}

	static int
	null_set_vid (device_t dev, int index, uint16_t vid)
	{
		return (0);
	}

	static int
	null_get_vid (device_t dev, int index, uint16_t *vid)
	{
		return (0);
	}

	static int
	null_set_vlanports (device_t dev, int index, uint32_t members)
	{
		return (0);
	}

	static int
	null_get_vlanports (device_t dev, int index, uint32_t *members)
	{
		return (0);
	}

	static int
	null_get_portlink (device_t dev, int port)
	{
		return (0);
	}

	static int
	null_get_portspeed (device_t dev, int port)
	{
		return (0);
	}

	static int
	null_force_mode (device_t dev, int port, uint32_t mode)
	{
		return (0);
	}

	static void
	null_tick (device_t dev)
	{

	}

	static int
	null_miibus_writereg(device_t dev, int phy, int reg, int value)
	{
		return (0xffff);
	}

	static int
	null_miibus_readreg(device_t dev, int phy, int reg)
	{
		return (0xffff);
	}

	static int
	null_pbvlan_setports(device_t dev, int port, uint32_t allowed)
	{
		return (0);
	}

	static int
	null_pbvlan_getports(device_t dev, int port, uint32_t *allowed)
	{
		return (0);
	}

	static int
	null_reset_subsys(device_t dev, int subsys)
	{

		return (0);
	}
};

#
# Get number of ports on this switch
#
METHOD struct switch_capability * get_caps {
	device_t dev;
} DEFAULT null_get_caps;

#
# Get switch register value
#
METHOD int get_reg {
	device_t dev;
	uint32_t reg;
	uint32_t *value;
} DEFAULT null_get_reg;

#
# Set switch register value, return old value
#
METHOD int set_reg {
	device_t dev;
	uint32_t reg;
	uint32_t *value;
} DEFAULT null_set_reg;

#
# Get MAC address table index of this MAC
#
METHOD int find_mac {
	device_t dev;
	uint64_t mac;
} DEFAULT null_find_mac;

#
# Write MAC address into table
#
METHOD int mac_write {
	device_t dev;
	uint64_t mac;
	int index;
	uint32_t port_map;
	uint8_t age;
	int *hash_idx;
} DEFAULT null_mac_write;

#
# Set Port VID
#
METHOD int set_pvid {
	device_t dev;
	int port;
	uint16_t pvid;
} DEFAULT null_set_port_vid;

#
# Get Port VID
#
METHOD int get_pvid {
	device_t dev;
	int port;
	uint16_t *pvid;
} DEFAULT null_get_port_vid;

#
# Set Port VLAN flags
#
METHOD int set_pflags {
	device_t dev;
	int port;
	uint32_t flags;
} DEFAULT null_set_port_flags;

#
# Get Port VLAN flags
#
METHOD int get_pflags {
	device_t dev;
	int port;
	uint32_t *flags;
} DEFAULT null_get_port_flags;

#
# Set VID for VLAN with that index
#
METHOD int set_vid {
	device_t dev;
	int index;
	uint16_t vid;
} DEFAULT null_set_vid;

#
# Get VID of VLAN with that index
#
METHOD int get_vid {
	device_t dev;
	int index;
	uint16_t *vid;
} DEFAULT null_get_vid;

#
# Set VLAN member ports
#
METHOD int set_vlanports {
	device_t dev;
	int index;
	uint32_t members;
} DEFAULT null_set_vlanports;

#
# Set VLAN member ports
#
METHOD int get_vlanports {
	device_t dev;
	int index;
	uint32_t *members;
} DEFAULT null_get_vlanports;

#
# Set VLAN untagged member ports
#
METHOD int set_vlanutports {
	device_t dev;
	int index;
	uint32_t members;
} DEFAULT null_set_vlanports;

#
# Get VLAN untagged member ports
#
METHOD int get_vlanutports {
	device_t dev;
	int index;
	uint32_t *members;
} DEFAULT null_get_vlanports;

#
# Write switch PHY register
#
METHOD int miibus_writereg {
	device_t dev;
	int phy;
	int reg;
	int value;
} DEFAULT null_miibus_writereg;

#
# Read switch PHY register
#
METHOD int miibus_readreg {
	device_t dev;
	int phy;
	int reg;
} DEFAULT null_miibus_readreg;

#
# Write switch PHY register
#
METHOD int pbvlan_setports {
	device_t dev;
	int port;
	uint32_t allowed;
} DEFAULT null_pbvlan_setports;

#
# Read switch PHY register
#
METHOD int pbvlan_getports {
	device_t dev;
	int port;
	uint32_t *allowed;
} DEFAULT null_pbvlan_getports;

#
# Get port link status
#
METHOD int get_portlink {
	device_t dev;
	int port;
} DEFAULT null_get_portlink;

#
# Get port speed
#
METHOD int get_portspeed {
	device_t dev;
	int port;
} DEFAULT null_get_portspeed;

#
# Get force required mode 
#
METHOD int force_mode {
	device_t dev;
	int port;
	uint32_t mode;
} DEFAULT null_force_mode;

#
# Give a chance to update something
#
METHOD void tick {
	device_t dev;
} DEFAULT null_tick;

#
# Reset/clear some subsystem
#
METHOD int reset_subsys {
	device_t dev;
	int subsys;
} DEFAULT null_reset_subsys;

