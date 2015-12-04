
#include <dev/siba/siba_cc_jtag.h>

/*
 * Initialize jtag master and return handle for
 * jtag_rwreg. Returns NULL on failure.
 */
void *
siba_jtag_attach(sb_t *sbh, uint clkd, bool exttap)
{
	void *regs;

	chipcregs_t *cc = (chipcregs_t *) regs;
	uint32_t caps;

	if (siba_core_get_rev(sc) < 11)
			return (NULL);


	/* Set clock divider if requested */
	if (clkd != 0) {
		tmp = R_REG(&cc->clkdiv);
		tmp = (tmp & ~CLKD_JTAG) |
			((clkd << CLKD_JTAG_SHIFT) & CLKD_JTAG);
		W_REG(&cc->clkdiv, tmp);
	}

	/* Enable JTAG interface */
	W_REG(&cc->jtagctrl, JCTRL_EN | (exttap ? JCTRL_EXT_EN : 0));

	return (regs);
}

void
sb_jtagm_disable(void *h)
{
	chipcregs_t *cc = (chipcregs_t *)h;

	W_REG(&cc->jtagctrl, R_REG(&cc->jtagctrl) & ~JCTRL_EN);
}

/*
 * Read/write a jtag register. Assumes a target with
 * 8 bit IR and 32 bit DR.
 */
#define	IRWIDTH		8
#define	DRWIDTH		32
uint32
jtag_rwreg(void *h, uint32 ir, uint32 dr)
{
	chipcregs_t *cc = (chipcregs_t *) h;
	uint32 tmp;

	W_REG(&cc->jtagir, ir);
	W_REG(&cc->jtagdr, dr);
	tmp = JCMD_START | JCMD_ACC_IRDR |
		((IRWIDTH - 1) << JCMD_IRW_SHIFT) |
		(DRWIDTH - 1);
	W_REG(&cc->jtagcmd, tmp);
	while (((tmp = R_REG(&cc->jtagcmd)) & JCMD_BUSY) == JCMD_BUSY) {
		/* OSL_DELAY(1); */
	}

	tmp = R_REG(&cc->jtagdr);
	return (tmp);
}

