// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cadence Torrent SD0801 PHY driver.
 *
 * Copyright 2018 Cadence Design Systems, Inc.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define DEFAULT_NUM_LANES	2
#define MAX_NUM_LANES		4
#define DEFAULT_MAX_BIT_RATE	8100 /* in Mbps */

#define POLL_TIMEOUT_US		2000
#define LANE_MASK		0x7

/*
 * register offsets from DPTX PHY register block base (i.e MHDP
 * register base + 0x30a00)
 */
#define PHY_AUX_CONFIG			0x00
#define PHY_AUX_CTRL			0x04
#define PHY_RESET			0x20
#define PHY_PMA_XCVR_PLLCLK_EN		0x24
#define PHY_PMA_XCVR_PLLCLK_EN_ACK	0x28
#define PHY_PMA_XCVR_POWER_STATE_REQ	0x2c
#define PHY_POWER_STATE_LN_0	0x0000
#define PHY_POWER_STATE_LN_1	0x0008
#define PHY_POWER_STATE_LN_2	0x0010
#define PHY_POWER_STATE_LN_3	0x0018
#define PHY_PMA_XCVR_POWER_STATE_ACK	0x30
#define PHY_PMA_CMN_READY		0x34
#define PHY_PMA_XCVR_TX_VMARGIN		0x38
#define PHY_PMA_XCVR_TX_DEEMPH		0x3c

/*
 * register offsets from SD0801 PHY register block base (i.e MHDP
 * register base + 0x500000)
 */
#define CMN_SSM_BANDGAP_TMR		(0x00084 / 2)
#define CMN_SSM_BIAS_TMR		(0x00088 / 2)
#define CMN_PLLSM0_PLLPRE_TMR		(0x000a8 / 2)
#define CMN_PLLSM0_PLLLOCK_TMR		(0x000b0 / 2)
#define CMN_PLLSM1_PLLPRE_TMR		(0x000c8 / 2)
#define CMN_PLLSM1_PLLLOCK_TMR		(0x000d0 / 2)
#define CMN_BGCAL_INIT_TMR		(0x00190 / 2)
#define CMN_BGCAL_ITER_TMR		(0x00194 / 2)
#define CMN_IBCAL_INIT_TMR		(0x001d0 / 2)
#define CMN_PLL0_VCOCAL_TCTRL		(0x00208 / 2)
#define CMN_PLL0_VCOCAL_INIT_TMR	(0x00210 / 2)
#define CMN_PLL0_VCOCAL_ITER_TMR	(0x00214 / 2)
#define CMN_PLL0_VCOCAL_REFTIM_START	(0x00218 / 2)
#define CMN_PLL0_VCOCAL_PLLCNT_START	(0x00220 / 2)
#define CMN_PLL0_INTDIV_M0		(0x00240 / 2)
#define CMN_PLL0_FRACDIVL_M0		(0x00244 / 2)
#define CMN_PLL0_FRACDIVH_M0		(0x00248 / 2)
#define CMN_PLL0_HIGH_THR_M0		(0x0024c / 2)
#define CMN_PLL0_DSM_DIAG_M0		(0x00250 / 2)
#define CMN_PLL0_SS_CTRL1_M0		(0x00260 / 2)
#define CMN_PLL0_SS_CTRL2_M0            (0x00264 / 2)
#define CMN_PLL0_SS_CTRL3_M0            (0x00268 / 2)
#define CMN_PLL0_SS_CTRL4_M0            (0x0026C / 2)
#define CMN_PLL0_LOCK_REFCNT_START      (0x00270 / 2)
#define CMN_PLL0_LOCK_PLLCNT_START	(0x00278 / 2)
#define CMN_PLL0_LOCK_PLLCNT_THR        (0x0027C / 2)
#define CMN_PLL1_VCOCAL_TCTRL		(0x00308 / 2)
#define CMN_PLL1_VCOCAL_INIT_TMR	(0x00310 / 2)
#define CMN_PLL1_VCOCAL_ITER_TMR	(0x00314 / 2)
#define CMN_PLL1_VCOCAL_REFTIM_START	(0x00318 / 2)
#define CMN_PLL1_VCOCAL_PLLCNT_START	(0x00320 / 2)
#define CMN_PLL1_INTDIV_M0		(0x00340 / 2)
#define CMN_PLL1_FRACDIVL_M0		(0x00344 / 2)
#define CMN_PLL1_FRACDIVH_M0		(0x00348 / 2)
#define CMN_PLL1_HIGH_THR_M0		(0x0034c / 2)
#define CMN_PLL1_DSM_DIAG_M0		(0x00350 / 2)
#define CMN_PLL1_SS_CTRL1_M0		(0x00360 / 2)
#define CMN_PLL1_SS_CTRL2_M0            (0x00364 / 2)
#define CMN_PLL1_SS_CTRL3_M0            (0x00368 / 2)
#define CMN_PLL1_SS_CTRL4_M0            (0x0036C / 2)
#define CMN_PLL1_LOCK_REFCNT_START      (0x00370 / 2)
#define CMN_PLL1_LOCK_PLLCNT_START	(0x00378 / 2)
#define CMN_PLL1_LOCK_PLLCNT_THR        (0x0037C / 2)
#define CMN_TXPUCAL_INIT_TMR		(0x00410 / 2)
#define CMN_TXPUCAL_ITER_TMR		(0x00414 / 2)
#define CMN_TXPDCAL_INIT_TMR		(0x00430 / 2)
#define CMN_TXPDCAL_ITER_TMR		(0x00434 / 2)
#define CMN_RXCAL_INIT_TMR		(0x00450 / 2)
#define CMN_RXCAL_ITER_TMR		(0x00454 / 2)
#define CMN_SD_CAL_INIT_TMR		(0x00490 / 2)
#define CMN_SD_CAL_ITER_TMR		(0x00494 / 2)
#define CMN_SD_CAL_REFTIM_START		(0x00498 / 2)
#define CMN_SD_CAL_PLLCNT_START		(0x004a0 / 2)
#define CMN_PDIAG_PLL0_CTRL_M0		(0x00680 / 2)
#define CMN_PDIAG_PLL0_CLK_SEL_M0	(0x00684 / 2)
#define CMN_PDIAG_PLL0_CP_PADJ_M0	(0x00690 / 2)
#define CMN_PDIAG_PLL0_CP_IADJ_M0	(0x00694 / 2)
#define CMN_PDIAG_PLL0_FILT_PADJ_M0	(0x00698 / 2)
#define CMN_PDIAG_PLL0_CP_PADJ_M1	(0x006d0 / 2)
#define CMN_PDIAG_PLL0_CP_IADJ_M1	(0x006d4 / 2)
#define CMN_PDIAG_PLL1_CTRL_M0		(0x00700 / 2)
#define CMN_PDIAG_PLL1_CLK_SEL_M0	(0x00704 / 2)
#define CMN_PDIAG_PLL1_CP_PADJ_M0	(0x00710 / 2)
#define CMN_PDIAG_PLL1_CP_IADJ_M0	(0x00714 / 2)
#define CMN_PDIAG_PLL1_FILT_PADJ_M0	(0x00718 / 2)
#define CMN_PDIAG_PLL1_CP_PADJ_M1	(0x00750 / 2)
#define CMN_PDIAG_PLL1_CP_IADJ_M1	(0x00754 / 2)

#define XCVR_DIAG_PLLDRC_CTRL(j)		(0x4000 + 0x01ca + (j) * 0x400)
#define XCVR_DIAG_HSCLK_SEL(j)			(0x4000 + 0x01cc + (j) * 0x400)
#define XCVR_DIAG_HSCLK_DIV(j)			(0x4000 + 0x01ce + (j) * 0x400)
#define XCVR_DIAG_BIDI_CTRL(j)			(0x4000 + 0x01d4 + (j) * 0x400)
#define TX_PSC_A0(j)				(0x4000 + 0x0200 + (j) * 0x400)
#define TX_PSC_A1(j)				(0x4000 + 0x0202 + (j) * 0x400)
#define TX_PSC_A2(j)				(0x4000 + 0x0204 + (j) * 0x400)
#define TX_PSC_A3(j)				(0x4000 + 0x0206 + (j) * 0x400)

#define TX_RCVDET_ST_TMR(j)			(0x4000 + 0x0246 + (j) * 0x400)

#define RX_PSC_A0(j)				(0x8000 + 0x0000 + (j) * 0x400)
#define RX_PSC_A1(j)				(0x8000 + 0x0002 + (j) * 0x400)
#define RX_PSC_A2(j)				(0x8000 + 0x0004 + (j) * 0x400)
#define RX_PSC_A3(j)				(0x8000 + 0x0006 + (j) * 0x400)

#define TX_TXCC_CTRL(j)				(0x4000 + 0x80 + (j) * 0x400)
#define TX_DIAG_ACYA(j)				(0x4000 + 0x3ce + (j) * 0x400)
#define DRV_DIAG_TX_DRV(j)			(0x4000 + 0x18c + (j) * 0x400)
#define TX_TXCC_MGNFS_MULT_000(j)		(0x4000 + 0xa0 + (j) * 0x400)
#define TX_TXCC_CPOST_MULT_00(j)		(0x4000 + 0x98 + (j) * 0x400)

#define PHY_PLL_CFG				(0xc000 + 0x001c)

#define PHY_PMA_CMN_CTRL2			0xe002
#define PHY_PMA_PLL_RAW_CTRL			0xe006

#define TX_DIAG_ACYA_HBDC_MASK			0x0001U

#define RX_PSC_CAL(j)				(0x8000 + 0x000c + (j) * 0x400)

#define RX_REE_GCSM1_CTRL(j)			(0x8000 + 0x0210 + (j) * 0x400)
#define RX_REE_GCSM2_CTRL(j)			(0x8000 + 0x0220 + (j) * 0x400)
#define RX_REE_PERGCSM_CTRL(j)			(0x8000 + 0x0230 + (j) * 0x400)

struct cdns_torrent_phy {
	void __iomem *base;	/* DPTX registers base */
	void __iomem *sd_base; /* SD0801 registers base */
	u32 num_lanes; /* Number of lanes to use */
	u32 max_bit_rate; /* Maximum link bit rate to use (in Mbps) */
	struct device *dev;
};

static int cdns_torrent_dp_init(struct phy *phy);
static void cdns_torrent_dp_run(struct cdns_torrent_phy *cdns_phy);
static
int cdns_torrent_dp_wait_pma_cmn_ready(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cfg(struct cdns_torrent_phy *cdns_phy);
static
void cdns_torrent_dp_pma_cmn_cfg_25mhz(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_lane_cfg(struct cdns_torrent_phy *cdns_phy,
					 unsigned int lane);
static
void cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(struct cdns_torrent_phy *cdns_phy);
static void cdns_torrent_dp_pma_cmn_rate(struct cdns_torrent_phy *cdns_phy);
static void cdns_dp_phy_write_field(struct cdns_torrent_phy *cdns_phy,
				    unsigned int offset,
				    unsigned char start_bit,
				    unsigned char num_bits,
				    unsigned int val);

static const struct phy_ops cdns_torrent_phy_ops = {
	.init		= cdns_torrent_dp_init,
	.owner		= THIS_MODULE,
};

/* PHY mmr access functions */

static void cdns_dp_phy_write_phy(struct cdns_torrent_phy *cdns_phy,
				  u32 offset, u16 val)
{
	writew(val, cdns_phy->sd_base + offset);
}

static u16 cdns_dp_phy_read_phy(struct cdns_torrent_phy *cdns_phy, u32 offset)
{
	return readw(cdns_phy->sd_base + offset);
}

#define cdns_phy_read_poll_timeout(offset, val, cond, delay_us, timeout_us) \
	readw_poll_timeout(cdns_phy->sd_base + (offset), val, \
			   cond, delay_us, timeout_us)

/* DPTX mmr access functions */

static void cdns_dp_phy_write_dp(struct cdns_torrent_phy *cdns_phy,
				 u32 offset, u16 val)
{
	writel(val, cdns_phy->base + offset);
}

static u32 cdns_dp_phy_read_dp(struct cdns_torrent_phy *cdns_phy, u32 offset)
{
	return readl(cdns_phy->base + offset);
}

#define cdns_phy_read_dp_poll_timeout(cdns_phy, offset, val, cond, \
				      delay_us, timeout_us) \
	readl_poll_timeout((cdns_phy)->base + (offset), \
			   val, cond, delay_us, timeout_us)

static int cdns_torrent_dp_init(struct phy *phy)
{
	unsigned char lane_bits;
	int r;

	struct cdns_torrent_phy *cdns_phy = phy_get_drvdata(phy);

	cdns_dp_phy_write_dp(cdns_phy, PHY_AUX_CTRL, 0x0003); /* enable AUX */

	/* PHY PMA registers configuration function */
	cdns_torrent_dp_pma_cfg(cdns_phy);

	/*
	 * Set lines power state to A0
	 * Set lines pll clk enable to 0
	 */

	cdns_dp_phy_write_field(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ,
				PHY_POWER_STATE_LN_0, 6, 0x0000);

	if (cdns_phy->num_lanes >= 2) {
		cdns_dp_phy_write_field(cdns_phy,
					PHY_PMA_XCVR_POWER_STATE_REQ,
					PHY_POWER_STATE_LN_1, 6, 0x0000);

		if (cdns_phy->num_lanes == 4) {
			cdns_dp_phy_write_field(cdns_phy,
						PHY_PMA_XCVR_POWER_STATE_REQ,
						PHY_POWER_STATE_LN_2, 6, 0);
			cdns_dp_phy_write_field(cdns_phy,
						PHY_PMA_XCVR_POWER_STATE_REQ,
						PHY_POWER_STATE_LN_3, 6, 0);
		}
	}

	cdns_dp_phy_write_field(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN,
				0, 1, 0x0000);

	if (cdns_phy->num_lanes >= 2) {
		cdns_dp_phy_write_field(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN,
					1, 1, 0x0000);
		if (cdns_phy->num_lanes == 4) {
			cdns_dp_phy_write_field(cdns_phy,
						PHY_PMA_XCVR_PLLCLK_EN,
						2, 1, 0x0000);
			cdns_dp_phy_write_field(cdns_phy,
						PHY_PMA_XCVR_PLLCLK_EN,
						3, 1, 0x0000);
		}
	}

	/*
	 * release phy_l0*_reset_n and pma_tx_elec_idle_ln_* based on
	 * used lanes
	 */
	lane_bits = (1 << cdns_phy->num_lanes) - 1;
	cdns_dp_phy_write_dp(cdns_phy, PHY_RESET,
			     ((0xF & ~lane_bits) << 4) | (0xF & lane_bits));

	/* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_PLLCLK_EN, 0x0001);

	/* PHY PMA registers configuration functions */
	cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(cdns_phy);
	cdns_torrent_dp_pma_cmn_rate(cdns_phy);

	/* take out of reset */
	cdns_dp_phy_write_field(cdns_phy, PHY_RESET, 8, 1, 1);
	r = cdns_torrent_dp_wait_pma_cmn_ready(cdns_phy);
	if (r)
		return r;
	cdns_torrent_dp_run(cdns_phy);

	return 0;
}

static
int cdns_torrent_dp_wait_pma_cmn_ready(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int reg;
	int ret;

	ret = cdns_phy_read_dp_poll_timeout(cdns_phy, PHY_PMA_CMN_READY, reg,
					    reg & 1, 0, 5000);
	if (ret == -ETIMEDOUT) {
		dev_err(cdns_phy->dev,
				"timeout waiting for PMA common ready\n");
		return -ETIMEDOUT;
	}

	return 0;

}

static void cdns_torrent_dp_pma_cfg(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int i;

	/* PMA common configuration */
	cdns_torrent_dp_pma_cmn_cfg_25mhz(cdns_phy);

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < cdns_phy->num_lanes; i++)
		cdns_torrent_dp_pma_lane_cfg(cdns_phy, i);
}

static
void cdns_torrent_dp_pma_cmn_cfg_25mhz(struct cdns_torrent_phy *cdns_phy)
{
	/* refclock registers - assumes 25 MHz refclock */
	cdns_dp_phy_write_phy(cdns_phy, CMN_SSM_BIAS_TMR, 0x0019);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLPRE_TMR, 0x0032);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM0_PLLLOCK_TMR, 0x00D1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLPRE_TMR, 0x0032);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLLSM1_PLLLOCK_TMR, 0x00D1);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_INIT_TMR, 0x007D);
	cdns_dp_phy_write_phy(cdns_phy, CMN_BGCAL_ITER_TMR, 0x007D);
	cdns_dp_phy_write_phy(cdns_phy, CMN_IBCAL_INIT_TMR, 0x0019);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_INIT_TMR, 0x001E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPUCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_INIT_TMR, 0x001E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_TXPDCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_INIT_TMR, 0x02EE);
	cdns_dp_phy_write_phy(cdns_phy, CMN_RXCAL_ITER_TMR, 0x0006);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_INIT_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_ITER_TMR, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_REFTIM_START, 0x000E);
	cdns_dp_phy_write_phy(cdns_phy, CMN_SD_CAL_PLLCNT_START, 0x012B);

	/* PLL registers */
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_PADJ_M0, 0x0509);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CP_IADJ_M0, 0x0F00);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_FILT_PADJ_M0, 0x0F08);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_DSM_DIAG_M0, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_INIT_TMR, 0x00FA);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_INIT_TMR, 0x00FA);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL1_VCOCAL_ITER_TMR, 0x0004);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_REFTIM_START, 0x0317);

}

static
void cdns_torrent_dp_pma_cmn_vco_cfg_25mhz(struct cdns_torrent_phy *cdns_phy)
{
	/* Assumes 25 MHz refclock */
	switch (cdns_phy->max_bit_rate) {
	/* Setting VCO for 10.8GHz */
	case 2700:
	case 5400:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x01B0);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x0120);
		break;
	/* Setting VCO for 9.72GHz */
	case 2430:
	case 3240:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0184);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0xCCCD);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x0104);
		break;
	/* Setting VCO for 8.64GHz */
	case 2160:
	case 4320:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0159);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x999A);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x00E7);
		break;
	/* Setting VCO for 8.1GHz */
	case 8100:
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_INTDIV_M0, 0x0144);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_HIGH_THR_M0, 0x00D8);
		break;
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
	cdns_dp_phy_write_phy(cdns_phy, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0315);

}

static void cdns_torrent_dp_pma_cmn_rate(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int clk_sel_val = 0;
	unsigned int hsclk_div_val = 0;
	unsigned int i;

	/* 16'h0000 for single DP link configuration */
	cdns_dp_phy_write_phy(cdns_phy, PHY_PLL_CFG, 0x0000);

	switch (cdns_phy->max_bit_rate) {
	case 1620:
		clk_sel_val = 0x0f01;
		hsclk_div_val = 2;
		break;
	case 2160:
	case 2430:
	case 2700:
		clk_sel_val = 0x0701;
		hsclk_div_val = 1;
		break;
	case 3240:
		clk_sel_val = 0x0b00;
		hsclk_div_val = 2;
		break;
	case 4320:
	case 5400:
		clk_sel_val = 0x0301;
		hsclk_div_val = 0;
		break;
	case 8100:
		clk_sel_val = 0x0200;
		hsclk_div_val = 0;
		break;
	}

	cdns_dp_phy_write_phy(cdns_phy, CMN_PDIAG_PLL0_CLK_SEL_M0, clk_sel_val);

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < cdns_phy->num_lanes; i++)
		cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_HSCLK_DIV(i),
				      hsclk_div_val);

}

static void cdns_torrent_dp_pma_lane_cfg(struct cdns_torrent_phy *cdns_phy,
		unsigned int lane)
{
	/* Writing Tx/Rx Power State Controllers registers */
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A0(lane), 0x00FB);
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A2(lane), 0x04AA);
	cdns_dp_phy_write_phy(cdns_phy, TX_PSC_A3(lane), 0x04AA);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A0(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A2(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_A3(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, RX_PSC_CAL(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, RX_REE_GCSM1_CTRL(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_REE_GCSM2_CTRL(lane), 0x0000);
	cdns_dp_phy_write_phy(cdns_phy, RX_REE_PERGCSM_CTRL(lane), 0x0000);

	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_BIDI_CTRL(lane), 0x000F);
	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_PLLDRC_CTRL(lane), 0x0001);
	cdns_dp_phy_write_phy(cdns_phy, XCVR_DIAG_HSCLK_SEL(lane), 0x0000);

}

static void cdns_torrent_dp_run(struct cdns_torrent_phy *cdns_phy)
{
	unsigned int read_val;
	u32 write_val1 = 0;
	u32 write_val2 = 0;
	u32 mask = 0;
	int ret;

	/*
	 * waiting for ACK of pma_xcvr_pllclk_en_ln_*, only for the
	 * master lane
	 */
	ret = cdns_phy_read_dp_poll_timeout(cdns_phy,
					    PHY_PMA_XCVR_PLLCLK_EN_ACK,
					    read_val, read_val & 1, 0,
					    POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT)
		dev_err(cdns_phy->dev,
			"timeout waiting for link PLL clock enable ack\n");

	ndelay(100);

	switch (cdns_phy->num_lanes) {
	case 1:	/* lane 0 */
		write_val1 = 0x00000004;
		write_val2 = 0x00000001;
		mask = 0x0000003f;
		break;
	case 2: /* lane 0-1 */
		write_val1 = 0x00000404;
		write_val2 = 0x00000101;
		mask = 0x00003f3f;
		break;
	case 4: /* lane 0-3 */
		write_val1 = 0x04040404;
		write_val2 = 0x01010101;
		mask = 0x3f3f3f3f;
		break;
	}

	cdns_dp_phy_write_dp(cdns_phy,
			     PHY_PMA_XCVR_POWER_STATE_REQ, write_val1);

	ret = cdns_phy_read_dp_poll_timeout(cdns_phy,
					    PHY_PMA_XCVR_POWER_STATE_ACK,
					    read_val,
					    (read_val & mask) == write_val1, 0,
					    POLL_TIMEOUT_US);

	if (ret == -ETIMEDOUT)
		dev_err(cdns_phy->dev,
			"timeout waiting for link power state ack\n");

	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ, 0);
	ndelay(100);

	cdns_dp_phy_write_dp(cdns_phy,
			     PHY_PMA_XCVR_POWER_STATE_REQ, write_val2);

	ret = cdns_phy_read_dp_poll_timeout(cdns_phy,
					    PHY_PMA_XCVR_POWER_STATE_ACK,
					    read_val,
					    (read_val & mask) == write_val2,
					    0, POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT)
		dev_err(cdns_phy->dev,
			"timeout waiting for link power state ack\n");

	cdns_dp_phy_write_dp(cdns_phy, PHY_PMA_XCVR_POWER_STATE_REQ, 0);
	ndelay(100);
}

static void cdns_dp_phy_write_field(struct cdns_torrent_phy *cdns_phy,
				    unsigned int offset,
				    unsigned char start_bit,
				    unsigned char num_bits,
				    unsigned int val)
{
	unsigned int read_val;

	read_val = cdns_dp_phy_read_dp(cdns_phy, offset);
	cdns_dp_phy_write_dp(cdns_phy, offset,
			     ((val << start_bit) |
			     (read_val & ~(((1 << num_bits) - 1) <<
			     start_bit))));

}

static int cdns_torrent_phy_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct cdns_torrent_phy *cdns_phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct phy *phy;
	int err;

	cdns_phy = devm_kzalloc(dev, sizeof(*cdns_phy), GFP_KERNEL);
	if (!cdns_phy)
		return -ENOMEM;

	cdns_phy->dev = &pdev->dev;

	phy = devm_phy_create(dev, NULL, &cdns_torrent_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create Torrent PHY\n");
		return PTR_ERR(phy);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	cdns_phy->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cdns_phy->base))
		return PTR_ERR(cdns_phy->base);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cdns_phy->sd_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(cdns_phy->sd_base))
		return PTR_ERR(cdns_phy->sd_base);

	err = device_property_read_u32(dev, "num_lanes",
				       &cdns_phy->num_lanes);
	if (err)
		cdns_phy->num_lanes = DEFAULT_NUM_LANES;

	switch (cdns_phy->num_lanes) {
	case 1:
	case 2:
	case 4:
		/* valid number of lanes */
		break;
	default:
		dev_err(dev, "unsupported number of lanes: %d\n",
			cdns_phy->num_lanes);
		return -EINVAL;
	}

	err = device_property_read_u32(dev, "max_bit_rate",
				       &cdns_phy->max_bit_rate);
	if (err)
		cdns_phy->max_bit_rate = DEFAULT_MAX_BIT_RATE;

	switch (cdns_phy->max_bit_rate) {
	case 1620:
	case 2160:
	case 2430:
	case 2700:
	case 3240:
	case 4320:
	case 5400:
	case 8100:
		/* valid bit rate */
		break;
	default:
		dev_err(dev, "unsupported max bit rate: %dMbps\n",
			cdns_phy->max_bit_rate);
		return -EINVAL;
	}

	phy_set_drvdata(phy, cdns_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	dev_info(dev, "%d lanes, max bit rate %d.%03d Gbps\n",
		 cdns_phy->num_lanes,
		 cdns_phy->max_bit_rate / 1000,
		 cdns_phy->max_bit_rate % 1000);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id cdns_torrent_phy_of_match[] = {
	{
		.compatible = "cdns,torrent-phy"
	},
	{}
};
MODULE_DEVICE_TABLE(of, cdns_torrent_phy_of_match);

static struct platform_driver cdns_torrent_phy_driver = {
	.probe	= cdns_torrent_phy_probe,
	.driver = {
		.name	= "cdns-torrent-phy",
		.of_match_table	= cdns_torrent_phy_of_match,
	}
};
module_platform_driver(cdns_torrent_phy_driver);

MODULE_AUTHOR("Cadence Design Systems, Inc.");
MODULE_DESCRIPTION("Cadence Torrent PHY driver");
MODULE_LICENSE("GPL v2");
