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

INTERFACE switchb;

CODE {
	static uint16_t
	null_reg_get(device_t dev, uint16_t reg)
	{
		return (0);
	}

	static void
	null_reg_set(device_t dev, uint16_t reg, uint16_t val)
	{
		return;
	}

	static uint32_t
	null_read4(device_t parent, uint32_t reg)
	{
		return (0);
	}

	static void
	null_write4(device_t parent, uint32_t reg, uint32_t val)
	{
		return;
	}

	static int
	null_register_isr(device_t dev, driver_filter_t isr, device_t child)
	{
		return (0);
	}

	static void
	null_unregister_isr(device_t dev, device_t child)
	{
		return;
	}

};

#
# Get switch register value
#
METHOD uint16_t reg_get {
	device_t dev;
	uint16_t reg;
} DEFAULT null_reg_get;

#
# Set switch register value
#
METHOD void reg_set {
	device_t dev;
	uint16_t reg;
	uint16_t value;
} DEFAULT null_reg_set;

#
# Read switch memory mapped reg
#
METHOD uint32_t read4 {
	device_t parent;
	uint32_t reg;
} DEFAULT null_read4;

#
# Write switch memory mapped reg
#
METHOD void write4 {
	device_t parent;
	uint32_t reg;
	uint32_t value;
} DEFAULT null_write4;

#
# Register ISR
#
METHOD int register_isr {
	device_t dev;
	driver_filter_t isr;
	device_t child;
} DEFAULT null_register_isr;

#
# UnRegister ISR
#
METHOD void unregister_isr {
	device_t dev;
	device_t child;
} DEFAULT null_unregister_isr;
