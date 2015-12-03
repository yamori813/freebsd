/*-
 * Copyright (c) 2010-2012 Aleksandr Rybalko <ray@ddteam.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 */

#ifndef _SWITCH_IOCTL_H_
#define	_SWITCH_IOCTL_H_

#define	SWITCH_API_VERSION	0
#define	SWITCH_IOCTL_MAIN_BASE	0x00

/* Main switch configuration */
struct switch_config {
	uint32_t phys;		/* PHY IDs that switch use */
	uint8_t version;	/* Version: Now 0. */
	uint8_t cmd;		/* 0 - Get, 1 - Set */
	uint8_t enable;		/* switch state, RW */
	uint8_t status;		/* Command status */
};

/* Basic switch config */
#define	SWITCH_CONFIG		SWITCH_IOCTL_MAIN_BASE+0
#define	IOCTL_SWITCH_CONFIG	_IOWR('E', SWITCH_CONFIG, struct switch_config)

/* Switch capability list */
struct switch_caps {

	uint8_t version;	/* Version: Now 0. */
	uint8_t cmd;		/* 0 - Get, 1 - Set */
	uint8_t status;		/* Command status */
	uint8_t ports;		/* Number of ports */
	uint32_t main;
#define	SWITCH_CAPS_MAIN_PORT_POWER	(1<<0)
#define	SWITCH_CAPS_MAIN_PORT_MIRROR	(1<<1)		/* Can do port mirror */
#define	SWITCH_CAPS_MAIN_PORT_SECURITY	(1<<2)		/* Can limit n MACs */
	uint32_t vlan;
#define	SWITCH_CAPS_VLAN_PORT		(1<<0)		/* Support port based */
#define	SWITCH_CAPS_VLAN_DOT1Q		(1<<1)		/* 802.1q */
#define	SWITCH_CAPS_VLAN_ISL		(1<<2)		/* ISL */
#define	SWITCH_CAPS_VLAN_GLBL_UNTG	(1<<3)		/* Global tag/untag */
#define	SWITCH_CAPS_VLAN_LAN_WAN	(1<<4)		/* LAN/WAN spliting */
#define	SWITCH_CAPS_VLAN_DOUBLE_TAG	(1<<5)		/* Q-in-Q */
#define	SWITCH_CAPS_VLAN_MAX_SHIFT_MASK	0x0000fff0 	/* max VLAN index */
#define	SWITCH_CAPS_VLAN_MAX_SHIFT_SHIFT 4 		/* 4096-full support */
	uint32_t qos;
#define	SWITCH_CAPS_QOS_QUEUES_MASK	0x00000007	/* 2 - 4 queues */
#define	SWITCH_CAPS_QOS_QUEUES_SHIFT	0 		/* 3 - 8 queues */
	uint32_t lacp;
	uint32_t stp;
	uint32_t acl;
	uint32_t stat;
};

/* List of switch capability */
#define	SWITCH_CAP		SWITCH_IOCTL_MAIN_BASE+1
#define	IOCTL_SWITCH_CAP 	_IOWR('E', SWITCH_CAP, struct switch_caps)

struct switch_reg {
	uint32_t addr;
	uint32_t value;
};

#define	SWITCH_GETREG		SWITCH_IOCTL_MAIN_BASE+2
#define	IOCTL_SWITCH_GETREG 	_IOWR('E', SWITCH_GETREG, struct switch_reg)
#define	SWITCH_SETREG		SWITCH_IOCTL_MAIN_BASE+3
#define	IOCTL_SWITCH_SETREG 	_IOWR('E', SWITCH_SETREG, struct switch_reg)

#define	SWITCH_RESETSUB		SWITCH_IOCTL_MAIN_BASE+4
#define	IOCTL_SWITCH_RESETSUB 	_IOWR('E', SWITCH_RESETSUB, uint32_t)
#define	SWITCH_RESETSUB_MASK		0x000000ff
#define	SWITCH_RESETSUB_SWITCH		1
#define	SWITCH_RESETSUB_PORT		2
#define	SWITCH_RESETSUB_VLANS		3
#define	SWITCH_RESETSUB_QOS		4
/* If we doing reset for ports, then store port number at upper location */
#define	SWITCH_RESETSUB_ALLPORTS	0x0000ff00
#define	SWITCH_RESETSUB_PORT_MASK	0x0000ff00
#define	SWITCH_RESETSUB_PORT_SHIFT	8


enum vlan_type_e {
	VLAN_TYPE_NONE = 0,
	VLAN_TYPE_PORT,
	VLAN_TYPE_DOT1Q,
	VLAN_TYPE_ISL
};

struct vlan_config {
	uint8_t version;	/* Version: Now 0. */
	uint8_t cmd;		/* 0 - Get, 1 - Set */
	uint8_t status;		/* Command status */
	/* Current VLAN mode, RW (if switch can) */
	enum vlan_type_e vlan_type;
	union {
		struct {
			enum {
				VLAN_BASE_TYPE_NONE,	/* Device use 12bit */
				VLAN_BASE_TYPE_FREE,	/* VID=base+id */
				VLAN_BASE_TYPE_2MASK,	/* VID=(base&~3)+id */
				VLAN_BASE_TYPE_3MASK,	/* VID=(base&~7)+id */
				VLAN_BASE_TYPE_4MASK,	/* VID=(base&~f)+id */
				VLAN_BASE_TYPE_5MASK,	/* VID=(base&~1f)+id */
			} vlan_base_type;
			/*
			 * VLAN base, for switches that do not
			 * support 12 bit tag
			 */
			uint16_t vlan_base;
		} dot1q;
	} d;
};

struct vlan_vlan_config {
	uint8_t version;	/* Version: Now 0. */
	uint8_t cmd;		/* 0 - Get, 1 - Set */
	uint8_t status;		/* Command status */
	/* Resource index (Port for Port based, VLAN index for 802.1q) */
	uint16_t index;
	enum vlan_type_e vlan_type;
	union {
		/* Port based */
		struct {
			/* Ports allowed for access with port in `index` */
			uint32_t allowed;
		} port;
		struct {
			/* VLAN ID (12 bit) */
			uint16_t vid;
			/* Per port config */
			uint8_t port_config[32];
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_NONE	(0<<0) /* Not a member */
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED	(1<<0) /* Tagged member */
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED	(2<<0) /* Untagged member */
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_FORBIDDEN	(3<<0) /* Filtered if no
							* ingress checking */
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_MASK	(7<<0)
#define	DOT1Q_PORT_VLAN_CONFIG_VLAN_TXTAG	(1<<3) /* Tagging on TX */

		} dot1q;
	} d;
};

struct vlan_port_config {
	uint8_t version;	/* Version: Now 0. */
	uint8_t cmd;		/* 0 - Get, 1 - Set */
	uint8_t status;		/* Command status */
	/* Port index */
	uint16_t index;
	/* 802.1q/ISL */
	enum vlan_type_e vlan_type;
	union {
		/* 802.1q based */
		struct {
			uint32_t flags;
#define	DOT1Q_VLAN_PORT_FLAG_INGRESS	(1<<0)	/* Ingress checking */
#define	DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG	(1<<1)	/* Enable Q-in-Q */
#define	DOT1Q_VLAN_PORT_FLAG_LAN	(1<<2)	/* Marked as LAN port */
#define	DOT1Q_VLAN_PORT_FLAG_WAN	(1<<3)	/* Marked as WAN port */
#define	DOT1Q_VLAN_PORT_FLAG_TAGGED	(1<<4)	/* Port tagged in all VLANs */
#define	DOT1Q_VLAN_PORT_FLAG_UNTAGGED	(1<<5)	/* Port untagged in all VLANs */
#define	DOT1Q_VLAN_PORT_FLAG_FORCE_UNTAGGED (1<<6) /* Remove tag */
#define	DOT1Q_VLAN_PORT_FLAG_FORCE_PVID	(1<<7)	/* Reassign tag to PVID */
#define	DOT1Q_VLAN_PORT_FLAG_DROP_UNTAGGED (1<<8) /* Drop untagged frames */
			uint16_t vid;
		} dot1q;
		/* ISL based */
		struct {
			uint32_t flags;
			uint16_t color;
		} isl;
	} d;
};

#define	SWITCH_IOCTL_VLAN_BASE 0x10
/* get/set switch VLAN basic configuration */
#define	VLAN_CONFIG		SWITCH_IOCTL_VLAN_BASE+0
/*
 * get/set switch VLAN configuration for 
 * one port (Port based) 
 * one VLAN index (!!!not VID!!!) (802.1Q)
 * one ISL index (ISL)
 */
#define	VLAN_VLAN_CONFIG	SWITCH_IOCTL_VLAN_BASE+1
/*
 * get/set switch port PVID for 
 * 802.1q or ISL
 */
#define	VLAN_PORT_CONFIG	SWITCH_IOCTL_VLAN_BASE+2

#define	IOCTL_VLAN_CONFIG	_IOWR('E', VLAN_CONFIG, struct vlan_config)
#define	IOCTL_VLAN_VLAN_CONFIG	_IOWR('E', VLAN_VLAN_CONFIG, \
				    struct vlan_vlan_config)
#define	IOCTL_VLAN_PORT_CONFIG	_IOWR('E', VLAN_PORT_CONFIG, \
				    struct vlan_port_config)


#endif /* _SWITCH_IOCTL_H_ */
