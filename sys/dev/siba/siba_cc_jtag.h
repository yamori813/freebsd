
#ifndef _SIBA_CC_JTAG_H_
#define  _SIBA_CC_JTAG_H_

#define SIBA_CC_JTAG_BASE	0x0030

#define SIBA_CC_JTAG_COMMAND	0x0000
#define SIBA_CC_JTAG_IR		0x0004
#define SIBA_CC_JTAG_DR		0x0008
#define SIBA_CC_JTAG_CTRL	0x000c


/* SIBA_CC_JTAG_COMMAND */
#define JTAG_CMD_START              0x80000000
#define JTAG_CMD_BUSY               0x80000000
#define JTAG_CMD_PAUSE              0x40000000
#define JTAG_CMD0_ACC_MASK          0x0000f000
#define JTAG_CMD0_ACC_IRDR          0x00000000
#define JTAG_CMD0_ACC_DR            0x00001000
#define JTAG_CMD0_ACC_IR            0x00002000
#define JTAG_CMD0_ACC_RESET         0x00003000
#define JTAG_CMD0_ACC_IRPDR         0x00004000
#define JTAG_CMD0_ACC_PDR           0x00005000
#define JTAG_CMD0_IRW_MASK          0x00000f00
#define JTAG_CMD_ACC_MASK           0x000f0000 /* Changes for corerev 11 */
#define JTAG_CMD_ACC_IRDR           0x00000000
#define JTAG_CMD_ACC_DR             0x00010000
#define JTAG_CMD_ACC_IR             0x00020000
#define JTAG_CMD_ACC_RESET          0x00030000
#define JTAG_CMD_ACC_IRPDR          0x00040000
#define JTAG_CMD_ACC_PDR            0x00050000
#define JTAG_CMD_IRW_MASK           0x00001f00
#define JTAG_CMD_IRW_SHIFT          8
#define JTAG_CMD_DRW_MASK           0x0000003f

/* SIBA_CC_JTAG_CTRL */
#define JTAG_CTRL_FORCE_CLK         4          /* Force clock */
#define JTAG_CTRL_EXT_EN            2          /* Enable external targets */
#define JTAG_CTRL_EN                1          /* Enable Jtag master */



#endif  _SIBA_CC_JTAG_H_


