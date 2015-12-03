#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <err.h>

#include <stdlib.h>
#include <limits.h>


#include <dev/switch/switch_ioctl.h>

struct info{
	struct switch_config	*sw;
	struct switch_caps	*cap;
	struct vlan_config	*vlan;
};

int	get_vlan_config(struct info *, int fd, int argc, char **argv);
int	set_vlan_config(struct info *, int fd, int argc, char **argv);
int	get_pbvlan_config(struct info *, int fd, int argc, char **argv);
int	set_pbvlan_config(struct info *, int fd, int argc, char **argv);
int	get_port(struct info *, int fd, int argc, char **argv);
int	set_port(struct info *, int fd, int argc, char **argv);
int	get_reg(struct info *info, int fd, int argc, char **argv);
int	set_reg(struct info *info, int fd, int argc, char **argv);

int
main(int argc, char **argv)
{
	struct switch_config	sw;
	struct switch_caps	cap;
	struct vlan_config	vlan_config;
	struct info		info;
	int			reset = 0;

	info.sw = &sw;
	info.cap = &cap;
	info.vlan = &vlan_config;

	bzero(&sw, sizeof(sw));
	bzero(&cap, sizeof(cap));
	bzero(&vlan_config, sizeof(vlan_config));

	sw.version = 0;
	cap.version = 0;
	vlan_config.version = 0;

	sw.cmd = 0;
	cap.cmd = 0;
	vlan_config.cmd = 0;
	vlan_config.vlan_type = VLAN_TYPE_DOT1Q;

	if (argc < 2) return (1);

	int fd = open(argv[1], O_RDONLY);

	if (ioctl(fd, IOCTL_SWITCH_CONFIG, 	&sw) == -1)
		err(1, "error from IOCTL_SWITCH_CONFIG");
	if (ioctl(fd, IOCTL_SWITCH_CAP, 	&cap) == -1)
		err(1, "error from IOCTL_SWITCH_CAP");
	if (ioctl(fd, IOCTL_VLAN_CONFIG, 	&vlan_config) == -1)
		err(1, "error from IOCTL_VLAN_CONFIG");
#if 0
	printf("IOCTL_SWITCH_CONFIG return:\n"
	    "\tversion = %d\n"
	    "\tcmd = %d\n"
	    "\tenable = %d\n"
	    "\tphys = 0x%08x\n",
	    sw.version,
	    sw.cmd,
	    sw.enable,
	    sw.phys);

	printf("IOCTL_SWITCH_CAP return:\n"
	    "\tversion = %d\n"
	    "\tcmd = %d\n"
	    "   Port capabilities 0x%08x:\n%s%s%s"
	    "   Number of ports = %d\n"
	    "   VLAN capabilities 0x%08x:\n%s%s%s%s"
	    "\tMax VLAN index = %d\n"
	    "\tNum QOS queues = 0x%08x\n"
	    "\tLACP caps = 0x%08x\n"
	    "\tSTP caps = 0x%08x\n"
	    "\tACL caps = 0x%08x\n"
	    "\tStatistics caps = 0x%08x\n"
	    ,
	    cap.version,
	    cap.cmd,
	    cap.main,
	    (cap.main & SWITCH_CAPS_MAIN_PORT_POWER)?
		"\tPort Power control\n":"",
	    (cap.main & SWITCH_CAPS_MAIN_PORT_MIRROR)?
		"\tPort Mirroring\n":"",
	    (cap.main & SWITCH_CAPS_MAIN_PORT_SECURITY)?
		"\tPort Security\n":"",
	    cap.ports,
	    cap.vlan,
	    (cap.vlan & SWITCH_CAPS_VLAN_PORT)?"\tPort based VLAN\n":"",
	    (cap.vlan & SWITCH_CAPS_VLAN_DOT1Q)?"\t802.1q\n":"",
	    (cap.vlan & SWITCH_CAPS_VLAN_ISL)?"\tISL\n":"",
	    (cap.vlan & SWITCH_CAPS_VLAN_GLBL_UNTG)?"\tGlobal Untagged\n":"",
	    (1 << ((cap.vlan & SWITCH_CAPS_VLAN_MAX_SHIFT_MASK) >> 
		SWITCH_CAPS_VLAN_MAX_SHIFT_SHIFT) ) - 1,

	    (cap.qos & SWITCH_CAPS_QOS_QUEUES_MASK) >> 
		SWITCH_CAPS_QOS_QUEUES_SHIFT,
	    cap.lacp,
	    cap.stp,
	    cap.acl,
	    cap.stat
	    );
#endif

#if 0
	sw.enable = sw.enable?0:1;
	sw.cmd = 1; /* Set */
	if (ioctl(fd, IOCTL_SWITCH_CONFIG, 	&sw) == -1)
		err(1, "error from IOCTL_SWITCH_CONFIG");
#endif

	if (argc >= 3 && strcasecmp(argv[2], "get") == 0) {
		if (argc >= 4 && strcasecmp(argv[3], "vlan") == 0) {
			get_vlan_config(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "pbvlan") == 0) {
			get_pbvlan_config(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "port") == 0) {
			get_port(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "reg") == 0) {
			get_reg(&info, fd, argc, argv);
		}
	} else if (argc >= 3 && strcasecmp(argv[2], "set") == 0) {
		if (argc >= 4 && strcasecmp(argv[3], "vlan") == 0) {
			set_vlan_config(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "pbvlan") == 0) {
			set_pbvlan_config(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "port") == 0) {
			set_port(&info, fd, argc, argv);
		}
		if (argc >= 4 && strcasecmp(argv[3], "reg") == 0) {
			set_reg(&info, fd, argc, argv);
		}
	} else if (argc >= 3 && strcasecmp(argv[2], "reset") == 0) {
		if (argc >= 4 && strcasecmp(argv[3], "vlans") == 0) {
			reset = SWITCH_RESETSUB_VLANS;
		}
		if (argc >= 4 && strcasecmp(argv[3], "switch") == 0) {
			reset = SWITCH_RESETSUB_SWITCH;
		}
		if (argc >= 5 && strcasecmp(argv[3], "port") == 0) {
			reset = SWITCH_RESETSUB_PORT |
			    ((strtoul(argv[4], 0, 0) <<
				SWITCH_RESETSUB_PORT_SHIFT) &
				    SWITCH_RESETSUB_PORT_MASK) ;
		}

		if (reset) {
			if (ioctl(fd, IOCTL_SWITCH_RESETSUB, 	&reset) == -1)
				err(1, "error from IOCTL_SWITCH_RESETSUB");
		} else {
			printf("Unknown subsystem \"%s\"\n", argv[3]);
		}
	}

	close(fd);

	return (0);
}

int
get_vlan_config(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_vlan_config vlan;
	int i;

	bzero(&vlan, sizeof(vlan));
	vlan.version = 0;
	vlan.cmd = 0;
	vlan.vlan_type = VLAN_TYPE_DOT1Q;

	if (argc >= 5) {
		int idx = strtoul(argv[4], 0, 0);
		vlan.index = idx;

		if (ioctl(fd, IOCTL_VLAN_VLAN_CONFIG,	&vlan) == -1)
			err(1, "error from IOCTL_VLAN_VLAN_CONFIG");

		printf("Index %d VID%d:\n", idx, vlan.d.dot1q.vid);

		for (i = 0; i < info->cap->ports; i++) {
			printf("\tport%d: %s\n", i,
			    (vlan.d.dot1q.port_config[i] == 
				DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED)?
			    "Tagged":
			    (vlan.d.dot1q.port_config[i] == 
				DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED)?
			    "Untagged":
			    (vlan.d.dot1q.port_config[i] == 
				DOT1Q_PORT_VLAN_CONFIG_VLAN_FORBIDDEN)?
			    "Forbidden":
			    "None"
			    );
		}
	}
	return (1);
}

int
get_port(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_port_config vlan_port;

	bzero(&vlan_port, sizeof(vlan_port));
	vlan_port.version = 0;
	vlan_port.cmd = 0;
	vlan_port.vlan_type = VLAN_TYPE_DOT1Q;

	/* get port 7 */

	if (argc >= 5) {
		int idx = strtoul(argv[4], 0, 0);
		vlan_port.index = idx;

		if (ioctl(fd, IOCTL_VLAN_PORT_CONFIG,	&vlan_port) == -1)
			err(1, "error from IOCTL_VLAN_PORT_CONFIG");
		printf("Port %d PVID = %d, options (%s%s%s%s%s%s)\n",
		    idx, vlan_port.d.dot1q.vid,
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_INGRESS)?
		    "IngressCheck ":"",
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG)?
		    "Q-in-Q ":"",
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_LAN)?
		    "LAN ":"",
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_WAN)?
		    "WAN ":"",
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_TAGGED)?
		    "Tagged ":"",
		    (vlan_port.d.dot1q.flags & DOT1Q_VLAN_PORT_FLAG_UNTAGGED)?
		    "Untagged ":""
		    );

	}
	return (1);
}

int
get_reg(struct info *info, int fd, int argc, char **argv)
{
	struct switch_reg r;

	bzero(&r, sizeof(r));
	/* get reg 0x100 */

	if (argc >= 5) {
		r.addr = strtoul(argv[4], 0, 0);

		if (ioctl(fd, IOCTL_SWITCH_GETREG, &r) == -1)
			err(1, "error from IOCTL_SWITCH_GETREG");
		printf("Reg 0x%08x Value = 0x%08x\n",
		    r.addr, r.value);
	}
	return (1);
}

int
set_reg(struct info *info, int fd, int argc, char **argv)
{
	struct switch_reg r;
	uint32_t value;

	bzero(&r, sizeof(r));
	/* set reg 0x100 0x1230 */

	if (argc >= 6) {
		r.addr = strtoul(argv[4], 0, 0);
		value = strtoul(argv[5], 0, 0);
		r.value = value;

		if (ioctl(fd, IOCTL_SWITCH_SETREG, &r) == -1)
			err(1, "error from IOCTL_SWITCH_SETREG");
		printf("Reg 0x%08x Value = 0x%08x (Old value = 0x%08x)\n",
		    r.addr, value, r.value);
	}
	return (1);
}

/* Shortcuts */
#define	DVPF_DOUBLE_TAG DOT1Q_VLAN_PORT_FLAG_DOUBLE_TAG
#define	DVPF_UNTAGGED DOT1Q_VLAN_PORT_FLAG_UNTAGGED

int
set_port(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_port_config vlan_port;
	uint32_t port_flags;
	int i, idx, port_flags_set;

	port_flags = 0;
	port_flags_set = 0;
	bzero(&vlan_port, sizeof(vlan_port));
	vlan_port.version = 0;
	vlan_port.vlan_type = VLAN_TYPE_DOT1Q;

	/* set port 7 pvid 17 flags WAN Q-in-Q IngressCheck */
	if (argc < 5)
		return (0);

	idx = strtoul(argv[4], 0, 0);
	vlan_port.index = idx;

	vlan_port.cmd = 0;
	/* Get current port config */
	if (ioctl(fd, IOCTL_VLAN_PORT_CONFIG,	&vlan_port) == -1)
	    err(1, "error from IOCTL_VLAN_PORT_CONFIG");

	for (i = 5; i < argc; i++) {
		if (argv[i] && argv[i+1] &&
		    strcasecmp(argv[i], "pvid") == 0 &&
		    strtoul(argv[i+1], 0, 0)) {
			i ++; /* Skip to argument */
			vlan_port.d.dot1q.vid = strtoul(argv[i], 0, 0);
			/* XXX check parsing error */
		} else if (argv[i] && argv[i+1] &&
			   strcasecmp(argv[i], "flags") == 0) {
			i ++; /* Skip to argument */
			while (i < argc && argv[i]) {
				switch (argv[i][0]) {
				case 'I': /* IngressCheck */
					port_flags_set++;
					port_flags |=
					   DOT1Q_VLAN_PORT_FLAG_INGRESS;
					break;
				case 'Q': /* Q-in-Q */
					if (info->cap->vlan &
					    SWITCH_CAPS_VLAN_DOUBLE_TAG) {
						port_flags_set++;
						port_flags |= DVPF_DOUBLE_TAG;
					} else
						printf("This switch don`t "
						    "support Q-in-Q\n");
					break;
				case 'L': /* LAN */
					if (info->cap->vlan &
					    SWITCH_CAPS_VLAN_LAN_WAN) {
						port_flags_set++;
						port_flags |=
						    DOT1Q_VLAN_PORT_FLAG_LAN;
					} else
						printf("This switch can`t split"
						    " ports for WAN/LAN\n");
					break;
				case 'W': /* WAN */
					if (info->cap->vlan &
					    SWITCH_CAPS_VLAN_LAN_WAN) {
						port_flags_set++;
						port_flags |=
						    DOT1Q_VLAN_PORT_FLAG_WAN;
					} else
						printf("This switch can`t "
						    "split ports for "
						    "WAN/LAN\n");
					break;
				case 'T': /* Global Tagged */
					if (info->cap->vlan &
					    SWITCH_CAPS_VLAN_GLBL_UNTG) {
						port_flags_set++;
						port_flags |=
						    DOT1Q_VLAN_PORT_FLAG_TAGGED;
						printf("Set to Tagged\n");
					} else
						printf("This switch can`t do "
						    "global tagging\n");
					break;
				case 'U': /* Global Untagged */
					if (info->cap->vlan &
					    SWITCH_CAPS_VLAN_GLBL_UNTG) {
						port_flags_set++;
						port_flags |= DVPF_UNTAGGED;
						printf("Set to UnTagged\n");
					} else
						printf("This switch can`t do "
						    "global tagging\n");
					break;
				default:
					err(1, "Illegal flag %s\n", argv[i]);
				}
				i++;
			}
		} else {
			err(1, "Unknown field \"%s\"\n", argv[i]);
		}
	}

	vlan_port.cmd = 1;
	if (port_flags_set > 0)
		vlan_port.d.dot1q.flags = port_flags;
	if (ioctl(fd, IOCTL_VLAN_PORT_CONFIG,	&vlan_port) == -1)
		err(1, "error from IOCTL_VLAN_PORT_CONFIG");

	return (0);
}

int
set_vlan_config(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_vlan_config vlan;
	char *check;
	int i, idx;

	bzero(&vlan, sizeof(vlan));
	vlan.version = 0;
	vlan.cmd = 0;
	vlan.vlan_type = VLAN_TYPE_DOT1Q;

	/* vlan 7 add port 3 t/u/f */
	/* vlan 7 del port 3 */

	if (argc < 5)
		return (1);
	idx = strtoul(argv[4], &check, 0);

	if (check != (argv[4] + strlen(argv[4])))
		return (1);

	vlan.index = idx;

	if (ioctl(fd, IOCTL_VLAN_VLAN_CONFIG,	&vlan) == -1) {
		warn("error from IOCTL_VLAN_VLAN_CONFIG");
	}

	if (argc >= 7) {
		if (strncasecmp(argv[5], "ad", 2) == 0) {
			/*  */
			int port = strtoul(argv[6], &check, 0);
			if (check != (argv[6] + strlen(argv[6])))
				return (1);

			if (argc >= 8 && tolower(argv[7][0]) == 'u') {
				vlan.d.dot1q.port_config[port] =
				    DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED;
			} else if (argc >= 8 && tolower(argv[7][0]) == 't') {
				vlan.d.dot1q.port_config[port] =
				    DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED;
			} else if (argc >= 8 && tolower(argv[7][0]) == 'f') {
				vlan.d.dot1q.port_config[port] =
				    DOT1Q_PORT_VLAN_CONFIG_VLAN_FORBIDDEN;
			} else {
				printf("Only tagged/untagged/forbidden "
				    "allowed (wrong '%s')\n", argv[7]);
				return (1);
			}
		} else if (strncasecmp(argv[5], "de", 2) == 0) {
			/*  */
			int port = strtoul(argv[6], &check, 0);
			if (check != (argv[6] + strlen(argv[6])))
				return (1);
			vlan.d.dot1q.port_config[port] =
			    DOT1Q_PORT_VLAN_CONFIG_VLAN_NONE;
		} else if (strncasecmp(argv[5], "vi", 2) == 0) {
			/*  */
			int vid = strtoul(argv[6], &check, 0);
			if (check != (argv[6] + strlen(argv[6])))
				return (1);
			vlan.d.dot1q.vid = vid;
		} else {
			printf("Only add/delete/vid allowed (wrong '%s')\n",
			    argv[5]);
			return (1);
		}
	}

	vlan.index = idx;
	vlan.cmd = 1; /* Set */

	if (ioctl(fd, IOCTL_VLAN_VLAN_CONFIG,	&vlan) == -1)
		err(1, "error from IOCTL_VLAN_VLAN_CONFIG");

	printf("Index %d VID%d:\n", idx, vlan.d.dot1q.vid);

	for (i = 0; i < info->cap->ports; i++)
		printf("%02d ", i);
	printf("\n");
	for (i = 0; i < info->cap->ports; i++) {
		printf(" %s ",
		    (vlan.d.dot1q.port_config[i] ==
			DOT1Q_PORT_VLAN_CONFIG_VLAN_TAGGED) ? "T" :
		    (vlan.d.dot1q.port_config[i] ==
			DOT1Q_PORT_VLAN_CONFIG_VLAN_UNTAGGED) ? "U" :
		    (vlan.d.dot1q.port_config[i] ==
			DOT1Q_PORT_VLAN_CONFIG_VLAN_FORBIDDEN) ? "F" :
		    "N");
	}
	printf("\n");
	return (1);
}

int
get_pbvlan_config(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_vlan_config vlan;
	int idx;

	bzero(&vlan, sizeof(vlan));
	vlan.version = 0;
	vlan.cmd = 0;
	vlan.vlan_type = VLAN_TYPE_PORT;

	if (argc >= 5) {
		idx = strtoul(argv[4], 0, 0);
		vlan.index = idx;

		if (ioctl(fd, IOCTL_VLAN_VLAN_CONFIG,	&vlan) == -1)
			err(1, "error from IOCTL_VLAN_VLAN_CONFIG");

		printf("Port %d allowed to send to ports %#08x\n", idx,
		    vlan.d.port.allowed);
	}
	return (1);
}

int
set_pbvlan_config(struct info *info, int fd, int argc, char **argv)
{
	struct vlan_vlan_config vlan;
	uint32_t allowed;
	int idx;

	bzero(&vlan, sizeof(vlan));
	vlan.version = 0;
	vlan.cmd = 1;
	vlan.vlan_type = VLAN_TYPE_PORT;

	if (argc >= 5) {
		idx = strtoul(argv[4], 0, 0);
		vlan.index = idx;

		if (argc >= 6) {
			allowed = strtoul(argv[5], 0, 0);
			/* TODO: Check strtoul error and value */
			vlan.d.port.allowed = allowed;
		} else {
			err(1, "error, [PBVLAN] no allowed ports specified");
		}
		if (ioctl(fd, IOCTL_VLAN_VLAN_CONFIG,	&vlan) == -1)
			err(1, "error from IOCTL_VLAN_VLAN_CONFIG");
	}
	return (1);
}


