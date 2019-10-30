// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP DisplayPort Torrent PHY driver.
 *
 * Copyright (c) 2019 Cadence Design Systems
 * Author: Anil Varughese <aniljoy@cadence.com>
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
#include <linux/reset.h>
#include <linux/regmap.h>
#include <phy-cadence-torrent.h>

#define POLL_TIMEOUT_US		2000
#define LANE_MASK		0x7

/*
 * register offsets from DPTX PHY register block base (i.e MHDP
 * register base + 0x30a00)
 */
#define PHY_AUX_CONFIG			0x00
#define PHY_AUX_CTRL			0x04
#define PHY_RESET			0x20
#define PMA_TX_ELEC_IDLE_MASK		0xF0U
#define PMA_TX_ELEC_IDLE_SHIFT		4
#define PHY_L00_RESET_N_MASK		0x01U
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
#define CMN_SSM_BANDGAP_TMR		(0x00084/2)
#define CMN_SSM_BIAS_TMR		(0x00088/2)
#define CMN_PLLSM0_PLLPRE_TMR		(0x000a8/2)
#define CMN_PLLSM0_PLLLOCK_TMR		(0x000b0/2)
#define CMN_PLLSM1_PLLPRE_TMR		(0x000c8/2)
#define CMN_PLLSM1_PLLLOCK_TMR		(0x000d0/2)
#define CMN_BGCAL_INIT_TMR		(0x00190/2)
#define CMN_BGCAL_ITER_TMR		(0x00194/2)
#define CMN_IBCAL_INIT_TMR		(0x001d0/2)
#define CMN_PLL0_VCOCAL_TCTRL		(0x00208/2)
#define CMN_PLL0_VCOCAL_INIT_TMR	(0x00210/2)
#define CMN_PLL0_VCOCAL_ITER_TMR	(0x00214/2)
#define CMN_PLL0_VCOCAL_REFTIM_START	(0x00218/2)
#define CMN_PLL0_VCOCAL_PLLCNT_START	(0x00220/2)
#define CMN_PLL0_INTDIV_M0		(0x00240/2)
#define CMN_PLL0_FRACDIVL_M0		(0x00244/2)
#define CMN_PLL0_FRACDIVH_M0		(0x00248/2)
#define CMN_PLL0_HIGH_THR_M0		(0x0024c/2)
#define CMN_PLL0_DSM_DIAG_M0		(0x00250/2)
#define CMN_PLL0_SS_CTRL1_M0		(0x00260/2)
#define CMN_PLL0_SS_CTRL2_M0            (0x00264/2)
#define CMN_PLL0_SS_CTRL3_M0            (0x00268/2)
#define CMN_PLL0_SS_CTRL4_M0            (0x0026C/2)
#define CMN_PLL0_LOCK_REFCNT_START      (0x00270/2)
#define CMN_PLL0_LOCK_PLLCNT_START	(0x00278/2)
#define CMN_PLL0_LOCK_PLLCNT_THR        (0x0027C/2)
#define CMN_PLL1_VCOCAL_TCTRL		(0x00308/2)
#define CMN_PLL1_VCOCAL_INIT_TMR	(0x00310/2)
#define CMN_PLL1_VCOCAL_ITER_TMR	(0x00314/2)
#define CMN_PLL1_VCOCAL_REFTIM_START	(0x00318/2)
#define CMN_PLL1_VCOCAL_PLLCNT_START	(0x00320/2)
#define CMN_PLL1_INTDIV_M0		(0x00340/2)
#define CMN_PLL1_FRACDIVL_M0		(0x00344/2)
#define CMN_PLL1_FRACDIVH_M0		(0x00348/2)
#define CMN_PLL1_HIGH_THR_M0		(0x0034c/2)
#define CMN_PLL1_DSM_DIAG_M0		(0x00350/2)
#define CMN_PLL1_SS_CTRL1_M0		(0x00360/2)
#define CMN_PLL1_SS_CTRL2_M0            (0x00364/2)
#define CMN_PLL1_SS_CTRL3_M0            (0x00368/2)
#define CMN_PLL1_SS_CTRL4_M0            (0x0036C/2)
#define CMN_PLL1_LOCK_REFCNT_START      (0x00370/2)
#define CMN_PLL1_LOCK_PLLCNT_START	(0x00378/2)
#define CMN_PLL1_LOCK_PLLCNT_THR        (0x0037C/2)
#define CMN_TXPUCAL_INIT_TMR		(0x00410/2)
#define CMN_TXPUCAL_ITER_TMR		(0x00414/2)
#define CMN_TXPDCAL_INIT_TMR		(0x00430/2)
#define CMN_TXPDCAL_ITER_TMR		(0x00434/2)
#define CMN_RXCAL_INIT_TMR		(0x00450/2)
#define CMN_RXCAL_ITER_TMR		(0x00454/2)
#define CMN_SD_CAL_INIT_TMR		(0x00490/2)
#define CMN_SD_CAL_ITER_TMR		(0x00494/2)
#define CMN_SD_CAL_REFTIM_START		(0x00498/2)
#define CMN_SD_CAL_PLLCNT_START		(0x004a0/2)
#define CMN_PDIAG_PLL0_CTRL_M0		(0x00680/2)
#define CMN_PDIAG_PLL0_CLK_SEL_M0	(0x00684/2)
#define CMN_PDIAG_PLL0_CP_PADJ_M0	(0x00690/2)
#define CMN_PDIAG_PLL0_CP_IADJ_M0	(0x00694/2)
#define CMN_PDIAG_PLL0_FILT_PADJ_M0	(0x00698/2)
#define CMN_PDIAG_PLL0_CP_PADJ_M1	(0x006d0/2)
#define CMN_PDIAG_PLL0_CP_IADJ_M1	(0x006d4/2)
#define CMN_PDIAG_PLL1_CTRL_M0		(0x00700/2)
#define CMN_PDIAG_PLL1_CLK_SEL_M0	(0x00704/2)
#define CMN_PDIAG_PLL1_CP_PADJ_M0	(0x00710/2)
#define CMN_PDIAG_PLL1_CP_IADJ_M0	(0x00714/2)
#define CMN_PDIAG_PLL1_FILT_PADJ_M0	(0x00718/2)
#define CMN_PDIAG_PLL1_CP_PADJ_M1	(0x00750/2)
#define CMN_PDIAG_PLL1_CP_IADJ_M1	(0x00754/2)

#define XCVR_DIAG_PLLDRC_CTRL(j)	(0x4000 + 0x01ca + (j) * 0x400)
#define XCVR_DIAG_HSCLK_SEL(j)		(0x4000 + 0x01cc + (j) * 0x400)
#define XCVR_DIAG_HSCLK_DIV(j)		(0x4000 + 0x01ce + (j) * 0x400)
#define XCVR_DIAG_BIDI_CTRL(j)		(0x4000 + 0x01d4 + (j) * 0x400)
#define TX_PSC_A0(j)			(0x4000 + 0x0200 + (j) * 0x400)
#define TX_PSC_A1(j)			(0x4000 + 0x0202 + (j) * 0x400)
#define TX_PSC_A2(j)			(0x4000 + 0x0204 + (j) * 0x400)
#define TX_PSC_A3(j)			(0x4000 + 0x0206 + (j) * 0x400)

#define TX_RCVDET_ST_TMR(j)		(0x4000 + 0x0246 + (j) * 0x400)

#define RX_PSC_A0(j)			(0x8000 + 0x0000 + (j) * 0x400)
#define RX_PSC_A1(j)			(0x8000 + 0x0002 + (j) * 0x400)
#define RX_PSC_A2(j)			(0x8000 + 0x0004 + (j) * 0x400)
#define RX_PSC_A3(j)			(0x8000 + 0x0006 + (j) * 0x400)

#define TX_TXCC_CTRL(j)			(0x4000 + 0x80 + (j) * 0x400)
#define TX_DIAG_ACYA(j)			(0x4000 + 0x3ce + (j) * 0x400)
#define DRV_DIAG_TX_DRV(j)		(0x4000 + 0x18c + (j) * 0x400)
#define TX_TXCC_MGNFS_MULT_000(j)	(0x4000 + 0xa0 + (j) * 0x400)
#define TX_TXCC_CPOST_MULT_00(j)	(0x4000 + 0x98 + (j) * 0x400)

#define PHY_PLL_CFG			(0xc000 + 0x001c)

#define PHY_PMA_CMN_CTRL2		(0xe002)
#define PHY_PMA_PLL_RAW_CTRL		(0xe006)

#define TX_DIAG_ACYA_HBDC_MASK		(0x0001U)

#define RX_PSC_CAL(j)			(0x8000 + 0x000c + (j) * 0x400)
#define RX_REE_GCSM1_CTRL(j)		(0x8000 + 0x0210 + (j) * 0x400)
#define RX_REE_GCSM2_CTRL(j)		(0x8000 + 0x0220 + (j) * 0x400)
#define RX_REE_PERGCSM_CTRL(j)		(0x8000 + 0x0230 + (j) * 0x400)


/*
 * Structure used to store values of PHY registers for voltage-related
 * coefficients, for particular voltage swing and re-emphasis level. Values
 * are shared across all physical lanes.
 */
struct coefficients {
	/** Value of DRV_DIAG_TX_DRV register to use */
	uint16_t diag_tx_drv;
	/** Value of TX_TXCC_MGNFS_MULT_000 register to use */
	uint16_t mgnfs_mult;
	/** Value of TX_TXCC_CPOST_MULT_00 register to use */
	uint16_t cpost_mult;
};

/*
 * Array consists of values of voltage-related registers for sd0801 PHY. A value
 * of 0xFFFF is a placeholder for invalid combination, and will never be used.
 */
static const struct coefficients voltage_coeffs[4][4] = {
	/* voltage swing 0, pre-emphasis 0->3 */
	{
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x002A, .cpost_mult = 0x0000},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x001F, .cpost_mult = 0x0014},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0012, .cpost_mult = 0x0020},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x002A}
	},

	/* voltage swing 1, pre-emphasis 0->3 */
	{
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x001F, .cpost_mult = 0x0000},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0013, .cpost_mult = 0x0012},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x001F},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	},

	/* voltage swing 2, pre-emphasis 0->3 */
	{
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0013, .cpost_mult = 0x0000},
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x0013},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	},

	/* voltage swing 3, pre-emphasis 0->3 */
	{
	  {.diag_tx_drv = 0x0003, .mgnfs_mult = 0x0000, .cpost_mult = 0x0000},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF},
	  {.diag_tx_drv = 0xFFFF, .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF}
	}
};

enum phy_powerstate {
	POWERSTATE_A0 = 0,
	/* Powerstate A1 is unused */
	POWERSTATE_A2 = 2,
	POWERSTATE_A3 = 3,
};

static void cdns_tor_dp_pma_cfg(struct cdns_tor_phy *tor_phy,
				struct cdns_tor_inst *tor_ins);
static void cdns_tor_dp_pma_lane_cfg(struct cdns_tor_phy *tor_phy,
				    struct cdns_tor_inst *tor_ins, int lane);
static void cdns_tor_dp_pma_cmn_cfg_19_2mhz(struct cdns_tor_phy *tor_phy);
static void cdns_tor_dp_pma_cmn_cfg_25mhz(struct cdns_tor_phy *tor_phy);
static void cdns_tor_dp_pma_cmn_rate(struct cdns_tor_phy *tor_phy,
				    u32 rate, u32 lanes);
static int cdns_tor_dp_wait_pma_cmn_ready(struct cdns_tor_phy *tor_phy);
static void cdns_tor_dp_run(struct cdns_tor_phy *tor_phy,
			    struct cdns_tor_inst *tor_ins);
static void cdns_write_field_aux(struct cdns_tor_phy *tor_phy,
					unsigned int offset,
					unsigned char start_bit,
					unsigned char num_bits,
					unsigned int val);
static void cdns_tor_dp_enable_ssc_19_2mhz(struct cdns_tor_phy *tor_phy,
					    u32 ctrl2_val, u32 ctrl3_val);
static void cdns_tor_dp_pma_cmn_vco_cfg_19_2mhz(struct cdns_tor_phy *tor_phy,
					    u32 rate, bool ssc);
static void cdns_tor_dp_pma_cmn_vco_cfg_25mhz(struct cdns_tor_phy *tor_phy,
					      u32 rate, bool ssc);
static void cdns_tor_dp_enable_ssc_25mhz(struct cdns_tor_phy *tor_phy,
					 u32 ctrl2_val);


int cdns_tor_dp_init(struct phy *gphy)
{
	struct cdns_tor_inst *tor_ins = phy_get_drvdata(gphy);
	struct cdns_tor_phy *tor_phy = dev_get_drvdata(gphy->dev.parent);
	unsigned char lane_bits;
	int r;

	dev_info(tor_phy->dev, "Initializing DP PHY regmap_dp %08x regmap_aux %08x tor_base %08x tor_aux_base %08x num_lanes %d max_bit_rate %d ref_clk_rate %u\n",
	      tor_phy->rmap_dp_cmn, tor_phy->rmap_dp_aux,
	      tor_phy->tor_dp_base, tor_phy->tor_dp_aux_base,
	      tor_ins->num_lanes, tor_ins->max_bit_rate,
	      tor_ins->ref_clk_rate);

	cdns_tor_write(tor_phy->rmap_dp_aux, PHY_AUX_CTRL, 0x0003);

	/* PHY PMA registers configuration function */
	cdns_tor_dp_pma_cfg(tor_phy, tor_ins);

	/*
	 * Set lines power state to A0
	 * Set lines pll clk enable to 0
	 */
	cdns_write_field_aux(tor_phy, PHY_PMA_XCVR_POWER_STATE_REQ,
				PHY_POWER_STATE_LN_0, 6, 0x0000);

	if (tor_ins->num_lanes >= 2) {
		cdns_write_field_aux(tor_phy,
					PHY_PMA_XCVR_POWER_STATE_REQ,
					PHY_POWER_STATE_LN_1, 6, 0x0000);

		if (tor_ins->num_lanes == 4) {
			cdns_write_field_aux(tor_phy,
						PHY_PMA_XCVR_POWER_STATE_REQ,
						PHY_POWER_STATE_LN_2, 6, 0);
			cdns_write_field_aux(tor_phy,
						PHY_PMA_XCVR_POWER_STATE_REQ,
						PHY_POWER_STATE_LN_3, 6, 0);
		}
	}

	cdns_write_field_aux(tor_phy, PHY_PMA_XCVR_PLLCLK_EN,
				0, 1, 0x0000);

	if (tor_ins->num_lanes >= 2) {
		cdns_write_field_aux(tor_phy, PHY_PMA_XCVR_PLLCLK_EN,
					1, 1, 0x0000);
		if (tor_ins->num_lanes == 4) {
			cdns_write_field_aux(tor_phy,
						PHY_PMA_XCVR_PLLCLK_EN,
						2, 1, 0x0000);
			cdns_write_field_aux(tor_phy,
						PHY_PMA_XCVR_PLLCLK_EN,
						3, 1, 0x0000);
		}
	}


	/*
	 * release phy_l0*_reset_n and pma_tx_elec_idle_ln_* based on
	 * used lanes
	 */
	lane_bits = (1 << tor_ins->num_lanes) - 1;
	cdns_tor_write(tor_phy->rmap_dp_aux, PHY_RESET,
		      ((0xF & ~lane_bits) << 4) | (0xF & lane_bits));

	/* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
	cdns_tor_write(tor_phy->rmap_dp_aux,
		      PHY_PMA_XCVR_PLLCLK_EN, 0x0001);

	/* PHY PMA registers configuration functions */
	/* Initialize PHY with max supported link rate, without SSC. */
	if (tor_ins->ref_clk_rate == REF_CLK_19_2MHZ) {
		cdns_tor_dp_pma_cmn_vco_cfg_19_2mhz(tor_phy,
					tor_ins->max_bit_rate, false);
	} else if (tor_ins->ref_clk_rate == REF_CLK_25MHZ) {
		cdns_tor_dp_pma_cmn_vco_cfg_25mhz(tor_phy,
					tor_ins->max_bit_rate, false);
	} else {
		dev_err(tor_phy->dev,
			"ref_clk_rate of %u not supported by dp\n",
			tor_ins->ref_clk_rate);
		r = -EINVAL;
		return r;
	}

	cdns_tor_dp_pma_cmn_rate(tor_phy, tor_ins->max_bit_rate,
				tor_ins->num_lanes);

	/* take out of reset */
	cdns_write_field_aux(tor_phy, PHY_RESET, 8, 1, 1);

	r = cdns_tor_dp_wait_pma_cmn_ready(tor_phy);
	if (r)
		return r;

	cdns_tor_dp_run(tor_phy, tor_ins);

	return 0;
}

static void cdns_write_field_aux(struct cdns_tor_phy *tor_phy,
					unsigned int offset,
					unsigned char start_bit,
					unsigned char num_bits,
					unsigned int val)
{

	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	unsigned int read_val;

	read_val = cdns_tor_read_aux(tor_phy, offset);
	cdns_tor_write(rmap_dp_aux, offset,
		      ((val << start_bit) |
		      (read_val & ~(((1 << num_bits) - 1) << start_bit))));
}

static void cdns_tor_dp_pma_cfg(struct cdns_tor_phy *tor_phy,
				struct cdns_tor_inst *tor_ins)
{
	unsigned int i;

	if (tor_ins->ref_clk_rate == REF_CLK_19_2MHZ) {
		/* PMA common configuration 19.2MHz */
		cdns_tor_dp_pma_cmn_cfg_19_2mhz(tor_phy);
	} else if (tor_ins->ref_clk_rate == REF_CLK_25MHZ) {
		/* PMA common configuration 25MHz */
		cdns_tor_dp_pma_cmn_cfg_25mhz(tor_phy);
	}

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < tor_ins->num_lanes; i++)
		cdns_tor_dp_pma_lane_cfg(tor_phy, tor_ins, i);
}

static void cdns_tor_dp_pma_cmn_cfg_19_2mhz(struct cdns_tor_phy *tor_phy)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	/* refclock registers - assumes 19.2 MHz refclock */
	cdns_tor_write(rmap_cmn, CMN_SSM_BIAS_TMR, 0x0014);
	cdns_tor_write(rmap_cmn, CMN_PLLSM0_PLLPRE_TMR, 0x0027);
	cdns_tor_write(rmap_cmn, CMN_PLLSM0_PLLLOCK_TMR, 0x00A1);
	cdns_tor_write(rmap_cmn, CMN_PLLSM1_PLLPRE_TMR, 0x0027);
	cdns_tor_write(rmap_cmn, CMN_PLLSM1_PLLLOCK_TMR, 0x00A1);
	cdns_tor_write(rmap_cmn, CMN_BGCAL_INIT_TMR, 0x0060);
	cdns_tor_write(rmap_cmn, CMN_BGCAL_ITER_TMR, 0x0060);
	cdns_tor_write(rmap_cmn, CMN_IBCAL_INIT_TMR, 0x0014);
	cdns_tor_write(rmap_cmn, CMN_TXPUCAL_INIT_TMR, 0x0018);
	cdns_tor_write(rmap_cmn, CMN_TXPUCAL_ITER_TMR, 0x0005);
	cdns_tor_write(rmap_cmn, CMN_TXPDCAL_INIT_TMR, 0x0018);
	cdns_tor_write(rmap_cmn, CMN_TXPDCAL_ITER_TMR, 0x0005);
	cdns_tor_write(rmap_cmn, CMN_RXCAL_INIT_TMR, 0x0240);
	cdns_tor_write(rmap_cmn, CMN_RXCAL_ITER_TMR, 0x0005);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_INIT_TMR, 0x0002);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_ITER_TMR, 0x0002);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_REFTIM_START, 0x000B);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_PLLCNT_START, 0x0137);

	/* PLL registers */
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CP_PADJ_M0, 0x0509);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CP_IADJ_M0, 0x0F00);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_FILT_PADJ_M0, 0x0F08);
	cdns_tor_write(rmap_cmn, CMN_PLL0_DSM_DIAG_M0, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CP_PADJ_M0, 0x0509);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CP_IADJ_M0, 0x0F00);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_FILT_PADJ_M0, 0x0F08);
	cdns_tor_write(rmap_cmn, CMN_PLL1_DSM_DIAG_M0, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_INIT_TMR, 0x00C0);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_ITER_TMR, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_INIT_TMR, 0x00C0);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_ITER_TMR, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_REFTIM_START, 0x0260);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_TCTRL, 0x0003);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_REFTIM_START, 0x0260);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_TCTRL, 0x0003);
}

/*
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void cdns_tor_dp_enable_ssc_19_2mhz(struct cdns_tor_phy *tor_phy,
						u32 ctrl2_val, u32 ctrl3_val)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, 0x0001);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, ctrl2_val);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, ctrl3_val);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL4_M0, 0x0003);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, 0x0001);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, ctrl2_val);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, ctrl3_val);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL4_M0, 0x0003);
}

static void cdns_tor_dp_pma_cmn_vco_cfg_19_2mhz(struct cdns_tor_phy *tor_phy,
						u32 rate, bool ssc)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	/* Assumes 19.2 MHz refclock */
	switch (rate) {
	/* Setting VCO for 10.8GHz */
	case 2700:
	case 5400:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x0119);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x4000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x00BC);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CTRL_M0, 0x0012);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x0119);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x4000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x00BC);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CTRL_M0, 0x0012);
		if (ssc)
			cdns_tor_dp_enable_ssc_19_2mhz(tor_phy, 0x033A, 0x006A);
		break;
	/* Setting VCO for 9.72GHz */
	case 1620:
	case 2430:
	case 3240:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x01FA);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x4000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x0152);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x01FA);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x4000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x0152);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_tor_dp_enable_ssc_19_2mhz(tor_phy, 0x05DD, 0x0069);
		break;
	/* Setting VCO for 8.64GHz */
	case 2160:
	case 4320:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x01C2);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x012C);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x01C2);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x012C);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_tor_dp_enable_ssc_19_2mhz(tor_phy, 0x0536, 0x0069);
		break;
	/* Setting VCO for 8.1GHz */
	case 8100:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x01A5);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0xE000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x011A);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x01A5);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0xE000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x011A);
		cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
		if (ssc)
			cdns_tor_dp_enable_ssc_19_2mhz(tor_phy, 0x04D7, 0x006A);
		break;
	}

	if (ssc) {
		cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_PLLCNT_START, 0x025E);
		cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
		cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_PLLCNT_START, 0x025E);
		cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_THR, 0x0005);
	} else {
		cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0260);
		cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0260);
		/* Set reset register values to disable SSC */
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL2_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL3_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL4_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL2_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL3_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL4_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_THR, 0x0003);
	}

	cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_REFCNT_START, 0x0099);
	cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_START, 0x0099);
	cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_REFCNT_START, 0x0099);
	cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_START, 0x0099);
}

static void cdns_tor_dp_pma_cmn_cfg_25mhz(struct cdns_tor_phy *tor_phy)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	/* refclock registers - assumes 25 MHz refclock */
	cdns_tor_write(rmap_cmn, CMN_SSM_BIAS_TMR, 0x0019);
	cdns_tor_write(rmap_cmn, CMN_PLLSM0_PLLPRE_TMR, 0x0032);
	cdns_tor_write(rmap_cmn, CMN_PLLSM0_PLLLOCK_TMR, 0x00D1);
	cdns_tor_write(rmap_cmn, CMN_PLLSM1_PLLPRE_TMR, 0x0032);
	cdns_tor_write(rmap_cmn, CMN_PLLSM1_PLLLOCK_TMR, 0x00D1);
	cdns_tor_write(rmap_cmn, CMN_BGCAL_INIT_TMR, 0x007D);
	cdns_tor_write(rmap_cmn, CMN_BGCAL_ITER_TMR, 0x007D);
	cdns_tor_write(rmap_cmn, CMN_IBCAL_INIT_TMR, 0x0019);
	cdns_tor_write(rmap_cmn, CMN_TXPUCAL_INIT_TMR, 0x001E);
	cdns_tor_write(rmap_cmn, CMN_TXPUCAL_ITER_TMR, 0x0006);
	cdns_tor_write(rmap_cmn, CMN_TXPDCAL_INIT_TMR, 0x001E);
	cdns_tor_write(rmap_cmn, CMN_TXPDCAL_ITER_TMR, 0x0006);
	cdns_tor_write(rmap_cmn, CMN_RXCAL_INIT_TMR, 0x02EE);
	cdns_tor_write(rmap_cmn, CMN_RXCAL_ITER_TMR, 0x0006);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_INIT_TMR, 0x0002);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_ITER_TMR, 0x0002);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_REFTIM_START, 0x000E);
	cdns_tor_write(rmap_cmn, CMN_SD_CAL_PLLCNT_START, 0x012B);
	/* PLL registers */
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CP_PADJ_M0, 0x0509);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CP_IADJ_M0, 0x0F00);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_FILT_PADJ_M0, 0x0F08);
	cdns_tor_write(rmap_cmn, CMN_PLL0_DSM_DIAG_M0, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CP_PADJ_M0, 0x0509);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CP_IADJ_M0, 0x0F00);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_FILT_PADJ_M0, 0x0F08);
	cdns_tor_write(rmap_cmn, CMN_PLL1_DSM_DIAG_M0, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_INIT_TMR, 0x00FA);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_ITER_TMR, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_INIT_TMR, 0x00FA);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_ITER_TMR, 0x0004);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_REFTIM_START, 0x0317);
	cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_TCTRL, 0x0003);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_REFTIM_START, 0x0317);
	cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_TCTRL, 0x0003);
}

/*
 * Set registers responsible for enabling and configuring SSC, with second
 * register value provided by a parameter.
 */
static void cdns_tor_dp_enable_ssc_25mhz(struct cdns_tor_phy *tor_phy,
					u32 ctrl2_val)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, 0x0001);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, ctrl2_val);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, 0x007F);
	cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL4_M0, 0x0003);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, 0x0001);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, ctrl2_val);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, 0x007F);
	cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL4_M0, 0x0003);
}

static void cdns_tor_dp_pma_cmn_vco_cfg_25mhz(struct cdns_tor_phy *tor_phy,
					      u32 rate, bool ssc)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	/* Assumes 25 MHz refclock */
	switch (rate) {
	/* Setting VCO for 10.8GHz */
	case 2700:
	case 5400:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x01B0);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x0120);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x01B0);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x0120);
		if (ssc)
			cdns_tor_dp_enable_ssc_25mhz(tor_phy, 0x0423);
		break;
	/* Setting VCO for 9.72GHz */
	case 1620:
	case 2430:
	case 3240:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x0184);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0xCCCD);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x0104);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x0184);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0xCCCD);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x0104);
		if (ssc)
			cdns_tor_dp_enable_ssc_25mhz(tor_phy, 0x03B9);
		break;
	/* Setting VCO for 8.64GHz */
	case 2160:
	case 4320:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x0159);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x999A);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x00E7);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x0159);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x999A);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x00E7);
		if (ssc)
			cdns_tor_dp_enable_ssc_25mhz(tor_phy, 0x034F);
		break;
	/* Setting VCO for 8.1GHz */
	case 8100:
		cdns_tor_write(rmap_cmn, CMN_PLL0_INTDIV_M0, 0x0144);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_HIGH_THR_M0, 0x00D8);
		cdns_tor_write(rmap_cmn, CMN_PLL1_INTDIV_M0, 0x0144);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVL_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_FRACDIVH_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_HIGH_THR_M0, 0x00D8);
		if (ssc)
			cdns_tor_dp_enable_ssc_25mhz(tor_phy, 0x031A);
		break;
	}

	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);

	if (ssc) {
		cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0315);
		cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
		cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0315);
		cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_THR, 0x0005);
	} else {
		cdns_tor_write(rmap_cmn, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0317);
		cdns_tor_write(rmap_cmn, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0317);
		/* Set reset register values to disable SSC */
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL1_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL2_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL3_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_SS_CTRL4_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL1_M0, 0x0002);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL2_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL3_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_SS_CTRL4_M0, 0x0000);
		cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_THR, 0x0003);
	}

	cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_REFCNT_START, 0x00C7);
	cdns_tor_write(rmap_cmn, CMN_PLL0_LOCK_PLLCNT_START, 0x00C7);
	cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_REFCNT_START, 0x00C7);
	cdns_tor_write(rmap_cmn, CMN_PLL1_LOCK_PLLCNT_START, 0x00C7);
}

static void cdns_tor_dp_pma_lane_cfg(struct cdns_tor_phy *tor_phy,
				    struct cdns_tor_inst *tor_ins, int lane)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;

	/* Per lane, refclock-dependent receiver detection setting */
	if (tor_ins->ref_clk_rate == REF_CLK_19_2MHZ)
		cdns_tor_write(rmap_cmn, TX_RCVDET_ST_TMR(lane), 0x0780);
	else if (tor_ins->ref_clk_rate == REF_CLK_25MHZ)
		cdns_tor_write(rmap_cmn, TX_RCVDET_ST_TMR(lane), 0x09C4);

	/* Writing Tx/Rx Power State Controllers registers */
	cdns_tor_write(rmap_cmn, TX_PSC_A0(lane), 0x00FB);
	cdns_tor_write(rmap_cmn, TX_PSC_A2(lane), 0x04AA);
	cdns_tor_write(rmap_cmn, TX_PSC_A3(lane), 0x04AA);
	cdns_tor_write(rmap_cmn, RX_PSC_A0(lane), 0x0000);
	cdns_tor_write(rmap_cmn, RX_PSC_A2(lane), 0x0000);
	cdns_tor_write(rmap_cmn, RX_PSC_A3(lane), 0x0000);

	cdns_tor_write(rmap_cmn, RX_PSC_CAL(lane), 0x0000);
	cdns_tor_write(rmap_cmn, RX_REE_GCSM1_CTRL(lane), 0x0000);
	cdns_tor_write(rmap_cmn, RX_REE_GCSM2_CTRL(lane), 0x0000);
	cdns_tor_write(rmap_cmn, RX_REE_PERGCSM_CTRL(lane), 0x0000);

	cdns_tor_write(rmap_cmn, XCVR_DIAG_BIDI_CTRL(lane), 0x000F);
	cdns_tor_write(rmap_cmn, XCVR_DIAG_PLLDRC_CTRL(lane), 0x0001);
	cdns_tor_write(rmap_cmn, XCVR_DIAG_HSCLK_SEL(lane), 0x0000);

}

static void cdns_tor_dp_run(struct cdns_tor_phy *tor_phy,
			    struct cdns_tor_inst *tor_ins)
{
	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	unsigned int read_val;
	u32 write_val1 = 0;
	u32 write_val2 = 0;
	u32 mask = 0;
	int ret;

	/*
	 * waiting for ACK of pma_xcvr_pllclk_en_ln_*, only for the
	 * master lane
	 */
	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
					PHY_PMA_XCVR_PLLCLK_EN_ACK,
					read_val,
					read_val & 1, 0, POLL_TIMEOUT_US);

	if (ret == -ETIMEDOUT)
		dev_err(tor_phy->dev,
			"timeout waiting for link PLL clock enable ack\n");

	ndelay(100);

	switch (tor_ins->num_lanes) {

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

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, write_val1);

	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
					PHY_PMA_XCVR_POWER_STATE_ACK,
					read_val,
					(read_val & mask) == write_val1, 0,
					POLL_TIMEOUT_US);

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, 0);
	ndelay(100);

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, write_val2);

	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
					    PHY_PMA_XCVR_POWER_STATE_ACK,
					    read_val,
					    (read_val & mask) == write_val2, 0,
					    POLL_TIMEOUT_US);
	if (ret == -ETIMEDOUT)
		dev_err(tor_phy->dev,
			"timeout waiting for link power state ack\n");

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, 0);

	ndelay(100);
}

static void cdns_tor_dp_pma_cmn_rate(struct cdns_tor_phy *tor_phy,
				    u32 rate, u32 lanes)
{
	struct regmap *rmap_cmn = tor_phy->rmap_dp_cmn;
	unsigned int clk_sel_val = 0;
	unsigned int hsclk_div_val = 0;
	unsigned int i;

	/* 16'h0000 for single DP link configuration */
	cdns_tor_write(rmap_cmn, PHY_PLL_CFG, 0x0000);

	switch (rate) {
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

	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL0_CLK_SEL_M0, clk_sel_val);
	cdns_tor_write(rmap_cmn, CMN_PDIAG_PLL1_CLK_SEL_M0, clk_sel_val);

	/* PMA lane configuration to deal with multi-link operation */
	for (i = 0; i < lanes; i++)
		cdns_tor_write(rmap_cmn, XCVR_DIAG_HSCLK_DIV(i),
			      hsclk_div_val);
}

static int cdns_tor_dp_wait_pma_cmn_ready(struct cdns_tor_phy *tor_phy)
{
	unsigned int reg;
	int ret;

	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy, PHY_PMA_CMN_READY, reg,
					    reg & 1, 0, 5000);
	if (ret == -ETIMEDOUT) {
		dev_err(tor_phy->dev,
			"timeout waiting for PMA common ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int cdns_tor_dp_set_power_state(struct cdns_tor_phy *tor_phy,
				       struct phy_configure_opts_dp *dp,
				       enum phy_powerstate powerstate)
{
	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	/* Register value for power state for a single byte. */
	u32 value_part;
	u32 value;
	u32 mask;
	u32 read_val;
	u32 ret;

	switch (powerstate) {
	case (POWERSTATE_A0):
		value_part = 0x01U;
		break;
	case (POWERSTATE_A2):
		value_part = 0x04U;
		break;
	default:
		/* Powerstate A3 */
		value_part = 0x08U;
		break;
	}

	/*
	 * Select values of registers and mask,
	 * depending on enabled lane count.
	 */
	switch (dp->lanes) {
	/* lane 0 */
	case (1):
		value = value_part;
		mask = 0x0000003FU;
		break;
	/* lanes 0-1 */
	case (2):
		value = (value_part
			 | (value_part << 8));
		mask = 0x00003F3FU;
		break;
	/* lanes 0-3, all */
	default:
		value = (value_part
			 | (value_part << 8)
			 | (value_part << 16)
			 | (value_part << 24));
		mask = 0x3F3F3F3FU;
		break;
	}

	/* Set power state A<n>. */
	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, value);

	/* Wait, until PHY acknowledges power state completion. */
	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
						PHY_PMA_XCVR_POWER_STATE_ACK,
						read_val,
						(read_val & mask) == value, 0,
						POLL_TIMEOUT_US);

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, 0x00000000);

	ndelay(100);

	return ret;
}

/*
 * Enable or disable PLL for selected lanes.
 */
static int cdns_tor_phy_set_pll_en(struct cdns_tor_phy *tor_phy,
				  struct phy_configure_opts_dp *dp,
				  bool enable)
{
	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	u32 read_val;
	u32 ret;

	/*
	 * Used to determine, which bits to check for or enable in
	 * PHY_PMA_XCVR_PLLCLK_EN register.
	 */
	u32 pll_reg_bits;
	/* used to enable or disable lanes. */
	u32 pll_reg_write_val;

	/* Select values of registers,mask depending on enabled lane count. */
	switch (dp->lanes) {
	/* lane 0 */
	case (1):
		pll_reg_bits = 0x00000001;
		break;
	/* lanes 0-1 */
	case (2):
		pll_reg_bits = 0x00000003;
		break;
	/* lanes 0-3, all */
	default:
		pll_reg_bits = 0x0000000F;
		break;
	}

	if (enable)
		pll_reg_write_val = pll_reg_bits;
	else
		pll_reg_write_val = 0x00000000;

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_PLLCLK_EN, pll_reg_write_val);

	/* Wait for acknowledgment from PHY. */
	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
				PHY_PMA_XCVR_PLLCLK_EN_ACK,
				read_val,
				(read_val & pll_reg_bits) == pll_reg_write_val,
				0, POLL_TIMEOUT_US);
	ndelay(100);
	return ret;
}

/*
 * Perform register operations related to setting link rate, once powerstate is
 * set and PLL disable request was processed.
 */
static int cdns_tor_phy_configure_rate(struct cdns_tor_phy *tor_phy,
				    struct cdns_tor_inst *tor_ins,
				    struct phy_configure_opts_dp *config_opts)
{
	u32 ret;
	u32 read_val;

	/* Disable the cmn_pll0_en before re-programming the new data rate. */
	cdns_tor_write(tor_phy->rmap_dp_cmn, PHY_PMA_PLL_RAW_CTRL, 0);

	/* Wait for PLL ready de-assertion. */
	/* For PLL0 - PHY_PMA_CMN_CTRL2[2] == 1 */
	ret = cdns_tor_read_poll_timeout(tor_phy, PHY_PMA_CMN_CTRL2,
					 read_val,
					 ((read_val >> 2) & 0x01) != 0,
					 0, POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ndelay(200);

	/* DP Rate Change - VCO Output settings. */
	if (tor_ins->ref_clk_rate == REF_CLK_19_2MHZ) {
		/* PMA common configuration 19.2MHz */
		cdns_tor_dp_pma_cmn_vco_cfg_19_2mhz(tor_phy,
						    config_opts->link_rate,
						    config_opts->ssc);
		cdns_tor_dp_pma_cmn_cfg_19_2mhz(tor_phy);
	} else if (tor_ins->ref_clk_rate == REF_CLK_25MHZ) {
		/* PMA common configuration 25MHz */
		cdns_tor_dp_pma_cmn_vco_cfg_25mhz(tor_phy,
						  config_opts->link_rate,
						  config_opts->ssc);
		cdns_tor_dp_pma_cmn_cfg_25mhz(tor_phy);
	}

	cdns_tor_dp_pma_cmn_rate(tor_phy,
				config_opts->link_rate,
				config_opts->lanes);

	/* Enable the cmn_pll0_en. */
	cdns_tor_write(tor_phy->rmap_dp_cmn, PHY_PMA_PLL_RAW_CTRL, 0x3);

	/* Wait for PLL ready assertion. */
	/* For PLL0 - PHY_PMA_CMN_CTRL2[0] == 1 */
	ret = cdns_tor_read_poll_timeout(tor_phy, PHY_PMA_CMN_CTRL2,
					 read_val,
					 (read_val & 0x01) != 0,
					 0, POLL_TIMEOUT_US);

	return ret;
}

/*
 * Verify, that parameters to configure PHY with are correct.
 */
static int cdns_tor_dp_verify_config(struct cdns_tor_inst *tor_ins,
				     struct phy_configure_opts_dp *dp)
{
	u8 i;

	/* If changing link rate was required, verify it's supported. */
	if (dp->set_rate) {
		switch (dp->link_rate) {
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
			return -EINVAL;
		}
	}

	/* Verify lane count. */
	switch (dp->lanes) {
	case 1:
	case 2:
	case 4:
		/* valid lane count. */
		break;
	default:
		return -EINVAL;
	}

	/* Check against actual number of PHY's lanes. */
	if (dp->lanes > tor_ins->num_lanes)
		return -EINVAL;

	/*
	 * If changing voltages is required, check swing and
	 * pre-emphasis levels, per-lane.
	 */
	if (dp->set_voltages) {
		/* Lane count verified previously. */
		for (i = 0; i < dp->lanes; i++) {
			if ((dp->voltage[i] > 3) || (dp->pre[i] > 3))
				return -EINVAL;

			/*
			 * Sum of voltage swing and pre-emphasis
			 * levels cannot exceed 3.
			 */
			if (dp->voltage[i] + dp->pre[i] > 3)
				return -EINVAL;
		}
	}

	return 0;
}

/* Set power state A0 and PLL clock enable to 0 on enabled lanes. */
static void cdns_tor_dp_set_a0_pll(struct cdns_tor_phy *tor_phy,
				   struct phy_configure_opts_dp *dp)
{
	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	u32 pwr_state = cdns_tor_read_aux(tor_phy,
					  PHY_PMA_XCVR_POWER_STATE_REQ);
	u32 pll_clk_en = cdns_tor_read_aux(tor_phy, PHY_PMA_XCVR_PLLCLK_EN);

	/* Lane 0 is always enabled. */
	pwr_state &= ~0x1FU;
	pll_clk_en &= ~0x01U;

	if (dp->lanes > 1) {
		pwr_state &= ~(0x1FU << 8);
		pll_clk_en &= ~(0x01U << 1);
	}

	if (dp->lanes > 2) {
		pwr_state &= ~(0x1FU << 16);
		pwr_state &= ~(0x1FU << 24);
		pll_clk_en &= ~(0x01U << 2);
		pll_clk_en &= ~(0x01U << 3);
	}

	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_POWER_STATE_REQ, pwr_state);
	cdns_tor_write(rmap_dp_aux, PHY_PMA_XCVR_PLLCLK_EN, pll_clk_en);
}

/* Configure lane count as required. */
static int cdns_tor_dp_set_lanes(struct cdns_tor_phy *tor_phy,
				 struct phy_configure_opts_dp *dp)
{
	struct regmap *rmap_dp_aux = tor_phy->rmap_dp_aux;
	u32 value;
	u32 ret;
	u8 lane_mask = (1 << dp->lanes) - 1;

	value = cdns_tor_read_aux(tor_phy, PHY_RESET);
	/* clear pma_tx_elec_idle_ln_* bits. */
	value &= ~PMA_TX_ELEC_IDLE_MASK;
	/* Assert pma_tx_elec_idle_ln_* for disabled lanes. */
	value |= ((~lane_mask) <<
			PMA_TX_ELEC_IDLE_SHIFT) & PMA_TX_ELEC_IDLE_MASK;
	cdns_tor_write(rmap_dp_aux, PHY_RESET, value);

	/* reset the link by asserting phy_l00_reset_n low */
	cdns_tor_write(rmap_dp_aux,
		      PHY_RESET, value & (~PHY_L00_RESET_N_MASK));

	/*
	 * Assert lane reset on unused lanes and lane 0 so they remain in reset
	 * and powered down when re-enabling the link
	 */
	value = (value & 0x0000FFF0) | (0x0000000E & lane_mask);
	cdns_tor_write(rmap_dp_aux, PHY_RESET, value);

	cdns_tor_dp_set_a0_pll(tor_phy, dp);

	/* release phy_l0*_reset_n based on used laneCount */
	value = (value & 0x0000FFF0) | (0x0000000F & lane_mask);
	cdns_tor_write(rmap_dp_aux, PHY_RESET, value);

	/* Wait, until PHY gets ready after releasing PHY reset signal. */
	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
						PHY_PMA_CMN_READY, value,
						(value & 0x01) != 0, 0,
						POLL_TIMEOUT_US);

	if (ret)
		return ret;

	ndelay(100);

	/* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
	cdns_tor_write(tor_phy->rmap_dp_aux, PHY_PMA_XCVR_PLLCLK_EN, 0x0001);

	/*
	 * waiting for ACK of pma_xcvr_pllclk_en_ln_*,
	 * only for the master lane
	 */
	ret = cdns_tor_dp_aux_read_poll_timeout(tor_phy,
						PHY_PMA_XCVR_PLLCLK_EN_ACK,
						value,
						(value & 0x01) != 0, 0,
						POLL_TIMEOUT_US);

	if (ret)
		return ret;

	ndelay(100);

	ret = cdns_tor_dp_set_power_state(tor_phy, dp, POWERSTATE_A2);
	if (ret)
		return ret;
	ret = cdns_tor_dp_set_power_state(tor_phy, dp, POWERSTATE_A0);

	return ret;
}

/* Configure link rate as required. */
static int cdns_tor_dp_set_rate(struct cdns_tor_phy *tor_phy,
				struct cdns_tor_inst *tor_ins,
				struct phy_configure_opts_dp *dp)
{
	u32 ret;

	ret = cdns_tor_dp_set_power_state(tor_phy, dp, POWERSTATE_A3);
	if (ret)
		return ret;
	ret = cdns_tor_phy_set_pll_en(tor_phy, dp, false);
	if (ret)
		return ret;
	ndelay(200);

	ret = cdns_tor_phy_configure_rate(tor_phy, tor_ins, dp);
	if (ret)
		return ret;
	ndelay(200);

	ret = cdns_tor_phy_set_pll_en(tor_phy, dp, true);
	if (ret)
		return ret;
	ret = cdns_tor_dp_set_power_state(tor_phy, dp, POWERSTATE_A2);
	if (ret)
		return ret;
	ret = cdns_tor_dp_set_power_state(tor_phy, dp, POWERSTATE_A0);
	if (ret)
		return ret;
	ndelay(900);

	return ret;
}

/* Configure voltage swing and pre-emphasis for all enabled lanes. */
static void cdns_tor_dp_set_voltages(struct cdns_tor_phy *tor_phy,
				     struct phy_configure_opts_dp *dp)
{
	struct regmap *rmap_dp_cmn = tor_phy->rmap_dp_cmn;
	u8 lane;
	u16 phy_reg;

	for (lane = 0; lane < dp->lanes; lane++) {
		phy_reg = cdns_tor_read(tor_phy, TX_DIAG_ACYA(lane));
		/*
		 * Write 1 to register bit TX_DIAG_ACYA[0] to freeze the
		 * current state of the analog TX driver.
		 */
		phy_reg |= TX_DIAG_ACYA_HBDC_MASK;
		cdns_tor_write(rmap_dp_cmn, TX_DIAG_ACYA(lane), phy_reg);

		cdns_tor_write(rmap_dp_cmn, TX_TXCC_CTRL(lane), 0x08A4);
		phy_reg =
		voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].diag_tx_drv;
		cdns_tor_write(rmap_dp_cmn, DRV_DIAG_TX_DRV(lane), phy_reg);

		phy_reg =
		voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].mgnfs_mult;
		cdns_tor_write(rmap_dp_cmn,
			      TX_TXCC_MGNFS_MULT_000(lane), phy_reg);

		phy_reg =
		voltage_coeffs[dp->voltage[lane]][dp->pre[lane]].cpost_mult;
		cdns_tor_write(rmap_dp_cmn,
			      TX_TXCC_CPOST_MULT_00(lane), phy_reg);

		phy_reg = cdns_tor_read(tor_phy, TX_DIAG_ACYA(lane));
		/*
		 * Write 0 to register bit TX_DIAG_ACYA[0] to allow the state of
		 * analog TX driver to reflect the new programmed one.
		 */
		phy_reg &= ~TX_DIAG_ACYA_HBDC_MASK;
		cdns_tor_write(rmap_dp_cmn, TX_DIAG_ACYA(lane), phy_reg);
	}
};

int cdns_tor_dp_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct cdns_tor_inst *tor_ins = phy_get_drvdata(phy);
	struct cdns_tor_phy *tor_phy = phy_get_drvdata(phy);
	int ret;

	dev_dbg(&phy->dev,
		"cdns_tor_phy_configure rate %d, lanes %d(%d), vs %d,%d,%d,%d, pe %d,%d,%d,%d, ssc %d\n",
		opts->dp.set_rate	? opts->dp.link_rate : -1,
		opts->dp.lanes, opts->dp.set_lanes,
		(opts->dp.set_voltages && opts->dp.lanes > 0)
		? opts->dp.voltage[0]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 1)
		? opts->dp.voltage[1]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 2)
		? opts->dp.voltage[2]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 3)
		? opts->dp.voltage[3]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 0)
		? opts->dp.pre[0]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 1)
		? opts->dp.pre[1]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 2)
		? opts->dp.pre[2]	: -1,
		(opts->dp.set_voltages && opts->dp.lanes > 3)
		? opts->dp.pre[3]	: -1,
		opts->dp.ssc);

	ret = cdns_tor_dp_verify_config(tor_ins, &opts->dp);
	if (ret) {
		dev_err(&phy->dev, "invalid params for phy configure\n");
		return ret;
	}

	if (opts->dp.set_lanes) {
		ret = cdns_tor_dp_set_lanes(tor_phy, &opts->dp);
		if (ret) {
			dev_err(&phy->dev, "cdns_tor_phy_set_lanes failed\n");
			return ret;
		}
	}

	if (opts->dp.set_rate) {
		ret = cdns_tor_dp_set_rate(tor_phy, tor_ins, &opts->dp);
		if (ret) {
			dev_err(&phy->dev, "cdns_tor_phy_set_rate failed\n");
			return ret;
		}
	}

	if (opts->dp.set_voltages)
		cdns_tor_dp_set_voltages(tor_phy, &opts->dp);

	return ret;
}
