/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence Torrent Multiprotocol PHY driver header file.
 *
 * Copyright (c) 2019 Cadence Design Systems
 * Author: Anil Varughese <aniljoy@cadence.com>
 *
 */

#ifndef _PHY_CADENCE_TORRENT_
#define _PHY_CADENCE_TORRENT_

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>

#define CDNS_TOR_MAX_LANES 8
#define REF_CLK_19_2MHZ 19200000
#define REF_CLK_25MHZ 25000000

struct cdns_reg_pairs {
	u32 off;
	u16 val;
};

struct cdns_tor_inst {
	struct phy *phy;
	u32 phy_type;
	u32 num_lanes;
	u32 mlane;
	u32 max_bit_rate;
	u32 ref_clk_rate;
	struct reset_control *lnk_rst;
};

struct cdns_regmap_cdb_context {
	struct device *dev;
	void __iomem *base;
	u8 reg_offset_shift;
	u32 block_offset;
	u8 block_offset_shift;
};

struct cdns_tor_gen_data {
	u8 block_offset_shift;
	u8 reg_offset_shift;
	u32 pcie_cmn_regs;
	u32 pcie_ln_regs;
	u32 usb_cmn_regs;
	u32 usb_ln_regs;
	struct cdns_reg_pairs *pcie_cmn_vals;
	struct cdns_reg_pairs *pcie_ln_vals;
	struct cdns_reg_pairs  *usb_cmn_vals;
	struct cdns_reg_pairs  *usb_ln_vals;
};

struct cdns_tor_phy {
	struct device *dev;
	struct cdns_tor_gen_data *init_data;
	struct cdns_tor_inst phys_inst[CDNS_TOR_MAX_LANES];
	struct regmap *rmap_gen_lane_cdb[CDNS_TOR_MAX_LANES];
	struct regmap *rmap_gen_cmn_cdb;
	struct regmap *rmap_gen_phy_config_ctrl;
	struct regmap *rmap_dp_cmn;
	struct regmap *rmap_dp_aux;
	struct regmap_field *phy_pll_cfg_1;
	void __iomem *tor_dp_base;
	void __iomem *tor_dp_aux_base;
	struct reset_control *phy_rst;
	struct reset_control *apb_rst;
	struct clk *clk;
	int nsubnodes;
};

/* #define TOR_DEBUG */

#ifdef TOR_DEBUG

#define cdns_tor_read_poll_timeout(tor_phy, offset, val, \
				  cond, delay_us, timeout_us) \
({ \
	int ret = 0; \
	dev_info(tor_phy->dev, \
		"- TORRENT READ POLL TIMEOUT %s %08x [%s %d]\n", \
		#offset, offset, __func__, __LINE__); \
	/*ret = */\
		/*readw_poll_timeout(tor_phy->tor_dp_base + offset,*/ \
				  /*val, cond, 0, POLL_TIMEOUT_US);*/ \
	ret == 0 ? 0 : -ETIMEDOUT; \
})

#define cdns_tor_dp_aux_read_poll_timeout(tor_phy, offset, val, \
			      cond, delay_us, timeout_us) \
({ \
	int ret = 0; \
	dev_info(tor_phy->dev, \
		"- AUX READ POLL TIMEOUT %s %08x [%s %d]\n", \
		#offset, offset, __func__, __LINE__); \
	/*ret = */\
		/*readl_poll_timeout(tor_phy->tor_dp_aux_base + offset,*/ \
				  /*val, cond, 0, POLL_TIMEOUT_US);*/ \
	ret == 0 ? 0 : -ETIMEDOUT; \
})

#define cdns_tor_read(tor_phy, offset)  \
({ \
	int ret = 0; \
	dev_info(tor_phy->dev, \
		"- TORRENT READ %s (0x%08x) [%s:%d]\n", \
		#offset, offset,  __func__, __LINE__); \
	/*ret = readw(tor_phy->tor_dp_base + offset); */ \
	ret == 0 ? 0 : ret; \
})

#define cdns_tor_read_aux(tor_phy, offset)  \
({ \
	int ret = 0; \
	dev_info(tor_phy->dev, "- AUX READ %s (0x%08x) [%s:%d]\n", \
		#offset, offset,  __func__, __LINE__); \
	/*ret = readl(tor_phy->tor_dp_aux_base + offset); */ \
	ret == 0 ? 0 : ret; \
})

#define cdns_tor_write(regmap_handle, offset, value)  \
({ \
	pr_info("WRITE %s = %08x (0x%08x) [%s:%d]\n", \
		#offset, offset, value, __func__, __LINE__); \
	/*regmap_write(regmap_handle, offset, value);*/ \
})

#define cdns_tor_field_write(regmap_handle, value)  \
({ \
	pr_info("FIELD WRITE regmap_handle %08x = %d [%s:%d]\n", \
		regmap_handle, value, __func__, __LINE__); \
	/*regmap_field_write(regmap_handle, value);*/ \
})

#else /* !TOR_DEBUG */

#define cdns_tor_read_poll_timeout(tor_phy, offset, val, \
				  cond, delay_us, timeout_us) \
({ \
	int ret = 0; \
	ret = readw_poll_timeout(tor_phy->tor_dp_base + offset, \
					    val, cond, 0, \
					    POLL_TIMEOUT_US); \
	ret == 0 ? 0 : -ETIMEDOUT; \
})

#define cdns_tor_dp_aux_read_poll_timeout(tor_phy, offset, val,\
					cond, delay_us, timeout_us) \
({ \
	int ret = 0; \
	ret = readl_poll_timeout(tor_phy->tor_dp_aux_base + offset, \
					    val, cond, 0, \
					    POLL_TIMEOUT_US); \
	ret == 0 ? 0 : -ETIMEDOUT; \
})

#define cdns_tor_read(tor_phy, offset)  \
({ \
	int ret = 0; \
	ret = readw(tor_phy->tor_dp_base + offset); \
	ret == 0 ? 0 : ret; \
})

#define cdns_tor_read_aux(tor_phy, offset)  \
({ \
	int ret = 0; \
	ret = readl(tor_phy->tor_dp_aux_base + offset); \
	ret == 0 ? 0 : ret; \
})

#define cdns_tor_write(regmap_handle, offset, value)  \
({ \
	regmap_write(regmap_handle, offset, value); \
})

#define cdns_tor_field_write(regmap_handle, value)  \
({ \
	regmap_field_write(regmap_handle, value); \
})

#endif /* !TOR_DEBUG */

int cdns_tor_dp_init(struct phy *phy);
int cdns_tor_dp_configure(struct phy *phy, union phy_configure_opts *opts);

#endif /* _PHY_CADENCE_TORRENT_H_ */
