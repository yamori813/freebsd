

static const struct siba_cc_pmu_res_updown siba_cc_pmu_4325_updown[] =
    SIBA_CC_PMU_4325_RES_UPDOWN;
static const struct siba_cc_pmu_res_depend siba_cc_pmu_4325_depend[] =
    SIBA_CC_PMU_4325_RES_DEPEND;
static const struct siba_cc_pmu_res_updown siba_cc_pmu_4328_updown[] =
    SIBA_CC_PMU_4328_RES_UPDOWN;
static const struct siba_cc_pmu_res_depend siba_cc_pmu_4328_depend[] =
    SIBA_CC_PMU_4328_RES_DEPEND;
static const struct siba_cc_pmu0_plltab siba_cc_pmu0_plltab[] =
    SIBA_CC_PMU0_PLLTAB_ENTRY;
static const struct siba_cc_pmu1_plltab siba_cc_pmu1_plltab[] =
    SIBA_CC_PMU1_PLLTAB_ENTRY;


void
siba_powerup(struct siba_softc *siba, int dynamic)
{
#ifdef SIBA_PCI_CODE
	siba_pci_gpio(siba, SIBA_GPIO_CRYSTAL | SIBA_GPIO_PLL, 1);
#endif
	siba_cc_clock(&siba->siba_cc,
	    (dynamic != 0) ? SIBA_CLOCK_DYNAMIC : SIBA_CLOCK_FAST);
}


static void
siba_cc_pmu_init(struct siba_cc *scc)
{
	const struct siba_cc_pmu_res_updown *updown = NULL;
	const struct siba_cc_pmu_res_depend *depend = NULL;
	struct siba_dev_softc *sd = scc->scc_dev;
	struct siba_softc *siba = sd->sd_bus;
	uint32_t min = 0, max = 0, pmucap;
	unsigned int i, updown_size, depend_size;

	if ((scc->scc_caps & SIBA_CC_CAPS_PMU) == 0)
		return;

	pmucap = SIBA_CC_READ32(scc, SIBA_CC_PMUCAPS);
	scc->scc_pmu.rev = (pmucap & SIBA_CC_PMUCAPS_REV);

	DPRINTF(siba, SIBA_DEBUG_PMU, "PMU(r%u) found (caps %#x)\n",
	    scc->scc_pmu.rev, pmucap);

	if (scc->scc_pmu.rev >= 1) {
		if (siba->siba_chiprev < 2 && siba->siba_chipid == 0x4325)
			SIBA_CC_MASK32(scc, SIBA_CC_PMUCTL,
			    ~SIBA_CC_PMUCTL_NOILP);
		else
			SIBA_CC_SET32(scc, SIBA_CC_PMUCTL,
			    SIBA_CC_PMUCTL_NOILP);
	}

	/* initialize PLL & PMU resources */
	switch (siba->siba_chipid) {
	case 0x4312:
		siba_cc_pmu1_pll0_init(scc, 0 /* use default */);
		/* use the default: min = 0xcbb max = 0x7ffff */
		break;
	case 0x4325:
		siba_cc_pmu1_pll0_init(scc, 0 /* use default */);

		updown = siba_cc_pmu_4325_updown;
		updown_size = N(siba_cc_pmu_4325_updown);
		depend = siba_cc_pmu_4325_depend;
		depend_size = N(siba_cc_pmu_4325_depend);

		min = (1 << SIBA_CC_PMU_4325_BURST) |
		    (1 << SIBA_CC_PMU_4325_LN);
		if (SIBA_CC_READ32(scc, SIBA_CC_CHIPSTAT) &
		    SIBA_CC_CHST_4325_PMUTOP_2B)
			min |= (1 << SIBA_CC_PMU_4325_CLBURST);
		max = 0xfffff;
		break;
	case 0x4328:
		siba_cc_pmu0_pll0_init(scc, 0 /* use default */);

		updown = siba_cc_pmu_4328_updown;
		updown_size = N(siba_cc_pmu_4328_updown);
		depend = siba_cc_pmu_4328_depend;
		depend_size = N(siba_cc_pmu_4328_depend);

		min = (1 << SIBA_CC_PMU_4328_EXT_SWITCH_PWM) |
			  (1 << SIBA_CC_PMU_4328_BB_SWITCH_PWM) |
			  (1 << SIBA_CC_PMU_4328_CRYSTAL_EN);

		max = 0xfffff;
		break;
	case 0x5354:
		siba_cc_pmu0_pll0_init(scc, 0 /* use default */);

		max = 0xfffff;
		break;
	default:
		device_printf(siba->siba_dev,
		    "unknown chipid %#x for PLL & PMU init\n",
		    siba->siba_chipid);
	}

	if (updown) {
		for (i = 0; i < updown_size; i++) {
			SIBA_CC_WRITE32(scc, SIBA_CC_PMU_TABSEL,
			    updown[i].res);
			SIBA_CC_WRITE32(scc, SIBA_CC_PMU_UPDNTM,
			    updown[i].updown);
		}
	}
	if (depend) {
		for (i = 0; i < depend_size; i++) {
			SIBA_CC_WRITE32(scc, SIBA_CC_PMU_TABSEL,
			    depend[i].res);
			switch (depend[i].task) {
			case SIBA_CC_PMU_DEP_SET:
				SIBA_CC_WRITE32(scc, SIBA_CC_PMU_DEPMSK,
				    depend[i].depend);
				break;
			case SIBA_CC_PMU_DEP_ADD:
				SIBA_CC_SET32(scc, SIBA_CC_PMU_DEPMSK,
				    depend[i].depend);
				break;
			case SIBA_CC_PMU_DEP_REMOVE:
				SIBA_CC_MASK32(scc, SIBA_CC_PMU_DEPMSK,
				    ~(depend[i].depend));
				break;
			default:
				KASSERT(0 == 1,
				    ("%s:%d: assertion failed",
					__func__, __LINE__));
			}
		}
	}

	if (min)
		SIBA_CC_WRITE32(scc, SIBA_CC_PMU_MINRES, min);
	if (max)
		SIBA_CC_WRITE32(scc, SIBA_CC_PMU_MAXRES, max);
}

static void
siba_cc_power_init(struct siba_cc *scc)
{
	struct siba_softc *siba = scc->scc_dev->sd_bus;
	int maxfreq;

	if (siba->siba_chipid == 0x4321) {
		if (siba->siba_chiprev == 0)
			SIBA_CC_WRITE32(scc, SIBA_CC_CHIPCTL, 0x3a4);
		else if (siba->siba_chiprev == 1)
			SIBA_CC_WRITE32(scc, SIBA_CC_CHIPCTL, 0xa4);
	}

	if ((scc->scc_caps & SIBA_CC_CAPS_PWCTL) == 0)
		return;

	if (scc->scc_dev->sd_id.sd_rev >= 10)
		SIBA_CC_WRITE32(scc, SIBA_CC_CLKSYSCTL,
		    (SIBA_CC_READ32(scc, SIBA_CC_CLKSYSCTL) &
		    0xffff) | 0x40000);
	else {
		maxfreq = siba_cc_clockfreq(scc, 1);
		SIBA_CC_WRITE32(scc, SIBA_CC_PLLONDELAY,
		    (maxfreq * 150 + 999999) / 1000000);
		SIBA_CC_WRITE32(scc, SIBA_CC_FREFSELDELAY,
		    (maxfreq * 15 + 999999) / 1000000);
	}
}

static void
siba_cc_powerup_delay(struct siba_cc *scc)
{
	struct siba_softc *siba = scc->scc_dev->sd_bus;
	int min;

	if (siba->siba_type != SIBA_TYPE_PCI ||
	    !(scc->scc_caps & SIBA_CC_CAPS_PWCTL))
		return;

	min = siba_cc_clockfreq(scc, 0);
	scc->scc_powerup_delay =
	    (((SIBA_CC_READ32(scc, SIBA_CC_PLLONDELAY) + 2) * 1000000) +
	    (min - 1)) / min;
}


static void
siba_cc_pmu1_pll0_init(struct siba_cc *scc, uint32_t freq)
{
	struct siba_dev_softc *sd = scc->scc_dev;
	struct siba_softc *siba = sd->sd_bus;
	const struct siba_cc_pmu1_plltab *e = NULL;
	uint32_t bufsth = 0, pll, pmu;
	unsigned int i;

	KASSERT(freq == 0, ("%s:%d: assertion vail", __func__, __LINE__));
	if (siba->siba_chipid == 0x4312) {
		scc->scc_pmu.freq = 20000;
		return;
	}

	e = siba_cc_pmu1_plltab_find(SIBA_CC_PMU1_DEFAULT_FREQ);
	KASSERT(e != NULL, ("%s:%d: assertion vail", __func__, __LINE__));
	scc->scc_pmu.freq = e->freq;

	pmu = SIBA_CC_READ32(scc, SIBA_CC_PMUCTL);
	if (SIBA_CC_PMUCTL_XF_VAL(pmu) == e->xf)
		return;

	DPRINTF(siba, SIBA_DEBUG_PLL, "change PLL value to %u.%03u MHz\n",
	    (e->freq / 1000), (e->freq % 1000));

	/* turn PLL off */
	switch (siba->siba_chipid) {
	case 0x4325:
		bufsth = 0x222222;
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MINRES,
		    ~((1 << SIBA_CC_PMU_4325_BBPLL_PWR) |
		      (1 << SIBA_CC_PMU_4325_HT)));
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MAXRES,
		    ~((1 << SIBA_CC_PMU_4325_BBPLL_PWR) |
		      (1 << SIBA_CC_PMU_4325_HT)));
		break;
	default:
		KASSERT(0 == 1,
		    ("%s:%d: assertion failed", __func__, __LINE__));
	}
	for (i = 0; i < 1500; i++) {
		if (!(SIBA_CC_READ32(scc, SIBA_CC_CLKCTLSTATUS) &
		      SIBA_CC_CLKCTLSTATUS_HT))
			break;
		DELAY(10);
	}
	if (SIBA_CC_READ32(scc, SIBA_CC_CLKCTLSTATUS) & SIBA_CC_CLKCTLSTATUS_HT)
		device_printf(siba->siba_dev, "failed to turn PLL off!\n");

	pll = siba_cc_pll_read(scc, SIBA_CC_PMU1_PLL0);
	pll &= ~(SIBA_CC_PMU1_PLL0_P1DIV | SIBA_CC_PMU1_PLL0_P2DIV);
	pll |= ((uint32_t)e->p1div << 20) & SIBA_CC_PMU1_PLL0_P1DIV;
	pll |= ((uint32_t)e->p2div << 24) & SIBA_CC_PMU1_PLL0_P2DIV;
	siba_cc_pll_write(scc, SIBA_CC_PMU1_PLL0, pll);

	pll = siba_cc_pll_read(scc, SIBA_CC_PMU1_PLL2);
	pll &= ~(SIBA_CC_PMU1_PLL2_NDIVINT | SIBA_CC_PMU1_PLL2_NDIVMODE);
	pll |= ((uint32_t)e->ndiv_int << 20) & SIBA_CC_PMU1_PLL2_NDIVINT;
	pll |= (1 << 17) & SIBA_CC_PMU1_PLL2_NDIVMODE;
	siba_cc_pll_write(scc, SIBA_CC_PMU1_PLL2, pll);

	pll = siba_cc_pll_read(scc, SIBA_CC_PMU1_PLL3);
	pll &= ~SIBA_CC_PMU1_PLL3_NDIVFRAC;
	pll |= ((uint32_t)e->ndiv_frac << 0) & SIBA_CC_PMU1_PLL3_NDIVFRAC;
	siba_cc_pll_write(scc, SIBA_CC_PMU1_PLL3, pll);

	if (bufsth) {
		pll = siba_cc_pll_read(scc, SIBA_CC_PMU1_PLL5);
		pll &= ~SIBA_CC_PMU1_PLL5_CLKDRV;
		pll |= (bufsth << 8) & SIBA_CC_PMU1_PLL5_CLKDRV;
		siba_cc_pll_write(scc, SIBA_CC_PMU1_PLL5, pll);
	}

	pmu = SIBA_CC_READ32(scc, SIBA_CC_PMUCTL);
	pmu &= ~(SIBA_CC_PMUCTL_ILP | SIBA_CC_PMUCTL_XF);
	pmu |= ((((uint32_t)e->freq + 127) / 128 - 1) << 16) &
	    SIBA_CC_PMUCTL_ILP;
	pmu |= ((uint32_t)e->xf << 2) & SIBA_CC_PMUCTL_XF;
	SIBA_CC_WRITE32(scc, SIBA_CC_PMUCTL, pmu);
}

static void
siba_cc_pmu0_pll0_init(struct siba_cc *scc, uint32_t xtalfreq)
{
	struct siba_dev_softc *sd = scc->scc_dev;
	struct siba_softc *siba = sd->sd_bus;
	const struct siba_cc_pmu0_plltab *e = NULL;
	uint32_t pmu, tmp, pll;
	unsigned int i;

	if ((siba->siba_chipid == 0x5354) && !xtalfreq)
		xtalfreq = 25000;
	if (xtalfreq)
		e = siba_cc_pmu0_plltab_findentry(xtalfreq);
	if (!e)
		e = siba_cc_pmu0_plltab_findentry(
		    SIBA_CC_PMU0_DEFAULT_XTALFREQ);
	KASSERT(e != NULL, ("%s:%d: fail", __func__, __LINE__));
	xtalfreq = e->freq;
	scc->scc_pmu.freq = e->freq;

	pmu = SIBA_CC_READ32(scc, SIBA_CC_PMUCTL);
	if (((pmu & SIBA_CC_PMUCTL_XF) >> 2) == e->xf)
		return;

	DPRINTF(siba, SIBA_DEBUG_PLL, "change PLL value to %u.%03u mhz\n",
	    (xtalfreq / 1000), (xtalfreq % 1000));

	KASSERT(siba->siba_chipid == 0x4328 || siba->siba_chipid == 0x5354,
	    ("%s:%d: fail", __func__, __LINE__));

	switch (siba->siba_chipid) {
	case 0x4328:
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MINRES,
		    ~(1 << SIBA_CC_PMU_4328_BB_PLL_PU));
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MAXRES,
		    ~(1 << SIBA_CC_PMU_4328_BB_PLL_PU));
		break;
	case 0x5354:
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MINRES,
		    ~(1 << SIBA_CC_PMU_5354_BB_PLL_PU));
		SIBA_CC_MASK32(scc, SIBA_CC_PMU_MAXRES,
		    ~(1 << SIBA_CC_PMU_5354_BB_PLL_PU));
		break;
	}
	for (i = 1500; i; i--) {
		tmp = SIBA_CC_READ32(scc, SIBA_CC_CLKCTLSTATUS);
		if (!(tmp & SIBA_CC_CLKCTLSTATUS_HT))
			break;
		DELAY(10);
	}
	tmp = SIBA_CC_READ32(scc, SIBA_CC_CLKCTLSTATUS);
	if (tmp & SIBA_CC_CLKCTLSTATUS_HT)
		device_printf(siba->siba_dev, "failed to turn PLL off!\n");

	/* set PDIV */
	pll = siba_cc_pll_read(scc, SIBA_CC_PMU0_PLL0);
	if (xtalfreq >= SIBA_CC_PMU0_PLL0_PDIV_FREQ)
		pll |= SIBA_CC_PMU0_PLL0_PDIV_MSK;
	else
		pll &= ~SIBA_CC_PMU0_PLL0_PDIV_MSK;
	siba_cc_pll_write(scc, SIBA_CC_PMU0_PLL0, pll);

	/* set WILD */
	pll = siba_cc_pll_read(scc, SIBA_CC_PMU0_PLL1);
	pll &= ~(SIBA_CC_PMU0_PLL1_STOPMOD | SIBA_CC_PMU0_PLL1_IMSK |
	    SIBA_CC_PMU0_PLL1_FMSK);
	pll |= ((uint32_t)e->wb_int << 28) & SIBA_CC_PMU0_PLL1_IMSK;
	pll |= ((uint32_t)e->wb_frac << 8) & SIBA_CC_PMU0_PLL1_FMSK;
	if (e->wb_frac == 0)
		pll |= SIBA_CC_PMU0_PLL1_STOPMOD;
	siba_cc_pll_write(scc, SIBA_CC_PMU0_PLL1, pll);

	/* set WILD */
	pll = siba_cc_pll_read(scc, SIBA_CC_PMU0_PLL2);
	pll &= ~SIBA_CC_PMU0_PLL2_IMSKHI;
	pll |= (((uint32_t)e->wb_int >> 4) << 0) & SIBA_CC_PMU0_PLL2_IMSKHI;
	siba_cc_pll_write(scc, SIBA_CC_PMU0_PLL2, pll);

	/* set freq and divisor. */
	pmu = SIBA_CC_READ32(scc, SIBA_CC_PMUCTL);
	pmu &= ~SIBA_CC_PMUCTL_ILP;
	pmu |= (((xtalfreq + 127) / 128 - 1) << 16) & SIBA_CC_PMUCTL_ILP;
	pmu &= ~SIBA_CC_PMUCTL_XF;
	pmu |= ((uint32_t)e->xf << 2) & SIBA_CC_PMUCTL_XF;
	SIBA_CC_WRITE32(scc, SIBA_CC_PMUCTL, pmu);
}

static const struct siba_cc_pmu1_plltab *
siba_cc_pmu1_plltab_find(uint32_t crystalfreq)
{
	const struct siba_cc_pmu1_plltab *e;
	unsigned int i;

	for (i = 0; i < N(siba_cc_pmu1_plltab); i++) {
		e = &siba_cc_pmu1_plltab[i];
		if (crystalfreq == e->freq)
			return (e);
	}

	return (NULL);
}

static const struct siba_cc_pmu0_plltab *
siba_cc_pmu0_plltab_findentry(uint32_t crystalfreq)
{
	const struct siba_cc_pmu0_plltab *e;
	unsigned int i;

	for (i = 0; i < N(siba_cc_pmu0_plltab); i++) {
		e = &siba_cc_pmu0_plltab[i];
		if (e->freq == crystalfreq)
			return (e);
	}

	return (NULL);
}

int
siba_powerdown(struct siba_softc *siba)
{
	struct siba_cc *scc;

	if (siba->siba_type == SIBA_TYPE_SSB)
		return (0);

	scc = &siba->siba_cc;
	if (!scc->scc_dev || scc->scc_dev->sd_id.sd_rev < 5)
		return (0);
	siba_cc_clock(scc, SIBA_CLOCK_SLOW);
#ifdef SIBA_PCI_CODE
	siba_pci_gpio(siba, SIBA_GPIO_CRYSTAL | SIBA_GPIO_PLL, 0);
#endif
	return (0);
}

static void
siba_cc_clock(struct siba_cc *scc, enum siba_clock clock)
{
	struct siba_dev_softc *sd = scc->scc_dev;
	struct siba_softc *siba;
	uint32_t tmp;

	if (sd == NULL)
		return;
	siba = sd->sd_bus;
	/*
	 * chipcommon < r6 (no dynamic clock control)
	 * chipcommon >= r10 (unknown)
	 */
	if (sd->sd_id.sd_rev < 6 || sd->sd_id.sd_rev >= 10 ||
	    (scc->scc_caps & SIBA_CC_CAPS_PWCTL) == 0)
		return;

	switch (clock) {
	case SIBA_CLOCK_DYNAMIC:
		tmp = SIBA_CC_READ32(scc, SIBA_CC_CLKSLOW) &
		    ~(SIBA_CC_CLKSLOW_ENXTAL | SIBA_CC_CLKSLOW_FSLOW |
		    SIBA_CC_CLKSLOW_IPLL);
		if ((tmp & SIBA_CC_CLKSLOW_SRC) != SIBA_CC_CLKSLOW_SRC_CRYSTAL)
			tmp |= SIBA_CC_CLKSLOW_ENXTAL;
		SIBA_CC_WRITE32(scc, SIBA_CC_CLKSLOW, tmp);
#ifdef SIBA_PCI_CODE
		if (tmp & SIBA_CC_CLKSLOW_ENXTAL)
			siba_pci_gpio(siba, SIBA_GPIO_CRYSTAL, 0);
#ednif
		break;
	case SIBA_CLOCK_SLOW:
		SIBA_CC_WRITE32(scc, SIBA_CC_CLKSLOW,
		    SIBA_CC_READ32(scc, SIBA_CC_CLKSLOW) |
		    SIBA_CC_CLKSLOW_FSLOW);
		break;
	case SIBA_CLOCK_FAST:
		/* crystal on */
#ifdef SIBA_PCI_CODE
		siba_pci_gpio(siba, SIBA_GPIO_CRYSTAL, 1);
#endif
		SIBA_CC_WRITE32(scc, SIBA_CC_CLKSLOW,
		    (SIBA_CC_READ32(scc, SIBA_CC_CLKSLOW) |
			SIBA_CC_CLKSLOW_IPLL) & ~SIBA_CC_CLKSLOW_FSLOW);
		break;
	default:
		KASSERT(0 == 1,
		    ("%s: unsupported clock %#x", __func__, clock));
	}
}



static int
siba_cc_clockfreq(struct siba_cc *scc, int max)
{
	enum siba_clksrc src;
	int div = 1, limit = 0;

	src = siba_cc_clksrc(scc);
	if (scc->scc_dev->sd_id.sd_rev < 6) {
		div = (src == SIBA_CC_CLKSRC_PCI) ? 64 :
		    (src == SIBA_CC_CLKSRC_CRYSTAL) ? 32 : 1;
		KASSERT(div != 1,
		    ("%s: unknown clock %d", __func__, src));
	} else if (scc->scc_dev->sd_id.sd_rev < 10) {
		switch (src) {
		case SIBA_CC_CLKSRC_CRYSTAL:
		case SIBA_CC_CLKSRC_PCI:
			div = ((SIBA_CC_READ32(scc, SIBA_CC_CLKSLOW) >> 16) +
			    1) * 4;
			break;
		case SIBA_CC_CLKSRC_LOWPW:
			break;
		}
	} else
		div = ((SIBA_CC_READ32(scc, SIBA_CC_CLKSYSCTL) >> 16) + 1) * 4;

	switch (src) {
	case SIBA_CC_CLKSRC_CRYSTAL:
		limit = (max) ? 20200000 : 19800000;
		break;
	case SIBA_CC_CLKSRC_LOWPW:
		limit = (max) ? 43000 : 25000;
		break;
	case SIBA_CC_CLKSRC_PCI:
		limit = (max) ? 34000000 : 25000000;
		break;
	}

	return (limit / div);
}

static enum siba_clksrc
siba_cc_clksrc(struct siba_cc *scc)
{
	struct siba_dev_softc *sd = scc->scc_dev;
	struct siba_softc *siba = sd->sd_bus;

	if (sd->sd_id.sd_rev < 6) {
		if (siba->siba_type == SIBA_TYPE_PCI) {
#ifdef SIBA_PCI_CODE
			if (pci_read_config(siba->siba_dev, SIBA_GPIO_OUT, 4) &
			    0x10)
				return (SIBA_CC_CLKSRC_PCI);
#endif
			return (SIBA_CC_CLKSRC_CRYSTAL);
		}
		if (siba->siba_type == SIBA_TYPE_SSB ||
		    siba->siba_type == SIBA_TYPE_PCMCIA)
			return (SIBA_CC_CLKSRC_CRYSTAL);
	}
	if (sd->sd_id.sd_rev < 10) {
		switch (SIBA_CC_READ32(scc, SIBA_CC_CLKSLOW) & 0x7) {
		case 0:
			return (SIBA_CC_CLKSRC_LOWPW);
		case 1:
			return (SIBA_CC_CLKSRC_CRYSTAL);
		case 2:
			return (SIBA_CC_CLKSRC_PCI);
		default:
			break;
		}
	}

	return (SIBA_CC_CLKSRC_CRYSTAL);
}

static uint32_t
siba_cc_pll_read(struct siba_cc *scc, uint32_t offset)
{

	SIBA_CC_WRITE32(scc, SIBA_CC_PLLCTL_ADDR, offset);
	return (SIBA_CC_READ32(scc, SIBA_CC_PLLCTL_DATA));
}

static void
siba_cc_pll_write(struct siba_cc *scc, uint32_t offset, uint32_t value)
{

	SIBA_CC_WRITE32(scc, SIBA_CC_PLLCTL_ADDR, offset);
	SIBA_CC_WRITE32(scc, SIBA_CC_PLLCTL_DATA, value);
}



