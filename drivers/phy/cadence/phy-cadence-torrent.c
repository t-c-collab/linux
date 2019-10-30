// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence Torrent Multiprotocol PHY driver.
 *
 * Copyright (c) 2019 Cadence Design Systems
 * Author: Anil Varughese <aniljoy@cadence.com>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <dt-bindings/phy/phy.h>
#include <phy-cadence-torrent.h>

/* Torrent defines */
#define TOR_PHY_PIPE_USB3_GEN2_PRE_CFG0_ADDR       (0x0000C020U)
#define TOR_PHY_PIPE_USB3_GEN2_POST_CFG0_ADDR      (0x0000C022U)
#define TOR_PHY_PIPE_USB3_GEN2_POST_CFG1_ADDR      (0x0000C023U)
#define TOR_CMN_PDIAG_PLL0_CLK_SEL_M0_ADDR         (0x000001A1U)
#define TOR_CMN_CDIAG_CDB_PWRI_OVRD_ADDR           (0x00000041U)
#define TOR_CMN_DIAG_BIAS_OVRD1_ADDR               (0x000001E1U)

#define TOR_XCVR_DIAG_HSCLK_SEL_ADDR               (0x000040E6U)
#define TOR_XCVR_DIAG_HSCLK_DIV_ADDR               (0x000040E7U)
#define TOR_XCVR_DIAG_PLLDRC_CTRL_ADDR             (0x000040E5U)
#define TOR_TX_PSC_A0_ADDR                         (0x00004100U)
#define TOR_TX_PSC_A1_ADDR                         (0x00004101U)
#define TOR_TX_PSC_A2_ADDR                         (0x00004102U)
#define TOR_TX_PSC_A3_ADDR                         (0x00004103U)
#define TOR_RX_PSC_A0_ADDR                         (0x00008000U)
#define TOR_RX_PSC_A1_ADDR                         (0x00008001U)
#define TOR_RX_PSC_A2_ADDR                         (0x00008002U)
#define TOR_RX_PSC_A3_ADDR                         (0x00008003U)
#define TOR_TX_TXCC_CTRL_ADDR                      (0x00004040U)
#define TOR_TX_TXCC_CPOST_MULT_01_ADDR             (0x0000404DU)
#define TOR_RX_SIGDET_HL_FILT_TMR_ADDR             (0x00008090U)
#define TOR_RX_REE_GCSM1_CTRL_ADDR                 (0x00008108U)
#define TOR_RX_REE_ATTEN_THR_ADDR                  (0x00008149U)
#define TOR_RX_REE_SMGM_CTRL1_ADDR                 (0x00008177U)
#define TOR_RX_REE_SMGM_CTRL2_ADDR                 (0x00008178U)
#define TOR_XCVR_DIAG_PSC_OVRD_ADDR                (0x000040EBU)
#define TOR_RX_REE_TAP1_CLIP_ADDR                  (0x00008171U)
#define TOR_RX_REE_TAP2TON_CLIP_ADDR               (0x00008172U)
#define TOR_RX_DIAG_SIGDET_TUNE_ADDR               (0x000081E8U)
#define TOR_RX_DIAG_NQST_CTRL_ADDR                 (0x000081E5U)
#define TOR_RX_DIAG_DFE_AMP_TUNE_2_ADDR            (0x000081E2U)
#define TOR_RX_DIAG_DFE_AMP_TUNE_3_ADDR            (0x000081E3U)
#define TOR_RX_DIAG_PI_CAP_ADDR                    (0x000081F5U)
#define TOR_RX_DIAG_PI_RATE_ADDR                   (0x000081F4U)
#define TOR_RX_DIAG_ACYA_ADDR                      (0x000081FFU)
#define TOR_RX_CDRLF_CNFG_ADDR                     (0x00008080U)
#define TOR_RX_CDRLF_CNFG3_ADDR                    (0x00008082U)

#define TOR_CMN_SD_CAL_INIT_TMR_ADDR               (0x00000124U)
#define TOR_CMN_PDIAG_PLL0_CLK_SEL_M0_ADDR         (0x000001A1U)
#define TOR_CMN_PDIAG_PLL0_CLK_SEL_M1_ADDR         (0x000001B1U)
#define TOR_CMN_PDIAG_PLL1_CLK_SEL_M0_ADDR         (0x000001C1U)
#define TOR_CMN_PLL0_VCOCAL_REFTIM_START_ADDR      (0x00000086U)
#define TOR_CMN_PLL1_VCOCAL_REFTIM_START_ADDR      (0x000000C6U)
#define TOR_CMN_PLL0_VCOCAL_TCTRL_ADDR             (0x00000082U)
#define TOR_CMN_PLL1_VCOCAL_TCTRL_ADDR             (0x000000C2U)
#define TOR_CMN_PDIAG_PLL0_CP_PADJ_M0_ADDR         (0x000001A4U)
#define TOR_CMN_PDIAG_PLL0_CP_PADJ_M1_ADDR         (0x000001B4U)
#define TOR_CMN_PDIAG_PLL1_CP_PADJ_M0_ADDR         (0x000001C4U)
#define TOR_CMN_PLL0_VCOCAL_PLLCNT_START_ADDR      (0x00000088U)
#define TOR_CMN_PLL1_VCOCAL_PLLCNT_START_ADDR      (0x000000C8U)
#define TOR_CMN_PLL0_LOCK_REFCNT_START_ADDR        (0x0000009CU)
#define TOR_CMN_PLL1_LOCK_REFCNT_START_ADDR        (0x000000DCU)
#define TOR_CMN_PLL0_LOCK_PLLCNT_START_ADDR        (0x0000009EU)
#define TOR_CMN_PLL1_LOCK_PLLCNT_START_ADDR        (0x000000DEU)
#define TOR_CMN_PLL0_LOCK_PLLCNT_THR_ADDR          (0x0000009FU)
#define TOR_CMN_PLL1_LOCK_PLLCNT_THR_ADDR          (0x000000DFU)

#define TOR_XCVR_DIAG_HSCLK_SEL_ADDR               (0x000040E6U)
#define TOR_XCVR_DIAG_HSCLK_DIV_ADDR               (0x000040E7U)
#define TOR_XCVR_DIAG_PLLDRC_CTRL_ADDR             (0x000040E5U)
#define TOR_RX_REE_TAP1_CLIP_ADDR                  (0x00008171U)
#define TOR_RX_REE_TAP2TON_CLIP_ADDR               (0x00008172U)
#define TOR_RX_DIAG_ACYA_ADDR                      (0x000081FFU)

#define TOR_PHY_PLL_CFG_ADDR     (0x0000C00EU)
#define TOR_GEN_COMMON_CDB_OFFSET	0x0
#define TOR_GEN_LANE_CDB_OFFSET(ln)	((ln) * 0x200U)

static const struct reg_field phy_pll_cfg_1 =
				REG_FIELD(TOR_PHY_PLL_CFG_ADDR, 0, 1);

static int cdns_regmap_writew(void *context, unsigned int reg, unsigned int val)
{
	struct cdns_regmap_cdb_context *ctx = context;
	u32 offset = reg << ctx->reg_offset_shift;

	writew(val, ctx->base + offset);

	return 0;
}

static int cdns_regmap_readw(void *context, unsigned int reg, unsigned int *val)
{
	struct cdns_regmap_cdb_context *ctx = context;
	u32 offset = reg << ctx->reg_offset_shift;

	*val = readw(ctx->base + offset);
	return 0;
}

static int cdns_regmap_writel(void *context, unsigned int reg, unsigned int val)
{
	struct cdns_regmap_cdb_context *ctx = context;
	u32 offset = reg << ctx->reg_offset_shift;

	writel(val, ctx->base + offset);
	return 0;
}

static int cdns_regmap_readl(void *context, unsigned int reg, unsigned int *val)
{
	struct cdns_regmap_cdb_context *ctx = context;
	u32 offset = reg << ctx->reg_offset_shift;

	*val = readl(ctx->base + offset);
	return 0;
}

#define CDNS_TOR_LANE_CDB_REGMAP_CONF(n) \
{ \
	.name = "torrent_lane"n"_cdb", \
	.reg_stride = 1, \
	.fast_io = true, \
	.reg_write = cdns_regmap_writew, \
	.reg_read = cdns_regmap_readw, \
}

/* The number of entries below should match the CDNS_TOR_MAX_LANES */
static struct regmap_config cdns_tor_gen_lane_cdb_config[] = {
	CDNS_TOR_LANE_CDB_REGMAP_CONF("0"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("1"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("2"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("3"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("4"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("5"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("6"),
	CDNS_TOR_LANE_CDB_REGMAP_CONF("7")
};

static struct regmap_config cdns_tor_gen_common_cdb_config = {
	.name = "torrent_gen_common_cdb",
	.reg_stride = 1,
	.fast_io = true,
	.reg_write = cdns_regmap_writew,
	.reg_read = cdns_regmap_readw,
};

static struct regmap_config cdns_tor_gen_phy_config_ctrl_config = {
	.name = "torrent_gen_phy_config_ctrl",
	.reg_stride = 1,
	.fast_io = true,
	.reg_write = cdns_regmap_writew,
	.reg_read = cdns_regmap_readw,
};

static struct regmap_config cdns_tor_dp_config = {
	.name = "torrent_dp",
	.reg_stride = 1,
	.fast_io = true,
	.reg_write = cdns_regmap_writew,
	.reg_read = cdns_regmap_readw,
};

static struct regmap_config cdns_tor_dp_aux_config = {
	.name = "torrent_dp_aux",
	.reg_stride = 1,
	.fast_io = true,
	.reg_write = cdns_regmap_writel,
	.reg_read = cdns_regmap_readl,
};

static void cdns_tor_phy_init(struct phy *gphy)
{
	struct cdns_tor_inst *ins = phy_get_drvdata(gphy);
	struct cdns_tor_phy *tor_phy = dev_get_drvdata(gphy->dev.parent);
	struct regmap *regmap;
	int i, j;
	struct cdns_reg_pairs *cmn_vals, *ln_vals;
	u32 num_cmn_regs, num_ln_regs;

	if (ins->phy_type == PHY_TYPE_PCIE) {
		dev_info(tor_phy->dev, "Applying PHY_TYPE_PCIE configuration");
		num_cmn_regs = tor_phy->init_data->pcie_cmn_regs;
		num_ln_regs = tor_phy->init_data->pcie_ln_regs;
		cmn_vals = tor_phy->init_data->pcie_cmn_vals;
		ln_vals = tor_phy->init_data->pcie_ln_vals;
	} else if (ins->phy_type == PHY_TYPE_USB3) {
		dev_info(tor_phy->dev, "Applying PHY_TYPE_USB3 configuration");
		num_cmn_regs = tor_phy->init_data->usb_cmn_regs;
		num_ln_regs = tor_phy->init_data->usb_ln_regs;
		cmn_vals = tor_phy->init_data->usb_cmn_vals;
		ln_vals = tor_phy->init_data->usb_ln_vals;
	} else {
		dev_info(tor_phy->dev,
			"Unsupported PHY (phy_type=%d) configuration\n",
			ins->phy_type);
		return;
	}

	for (j = 0; j < num_cmn_regs ; j++) {
		regmap = tor_phy->rmap_gen_cmn_cdb;
		cdns_tor_write(regmap, cmn_vals[j].off, cmn_vals[j].val);
	}

	for (i = 0; i < ins->num_lanes; i++) {
		for (j = 0; j < num_ln_regs ; j++) {
			regmap = tor_phy->rmap_gen_lane_cdb[i + ins->mlane];
			cdns_tor_write(regmap, ln_vals[j].off, ln_vals[j].val);
		}
	}
}

static int cdns_tor_phy_on(struct phy *gphy)
{
	struct cdns_tor_inst *ins = phy_get_drvdata(gphy);

	/* Take the PHY lane group out of reset */
	return reset_control_deassert(ins->lnk_rst);
}

static int cdns_tor_phy_off(struct phy *gphy)
{
	struct cdns_tor_inst *ins = phy_get_drvdata(gphy);

	return reset_control_assert(ins->lnk_rst);
}

static const struct phy_ops torrent_dp_ops = {
	.init		= cdns_tor_dp_init,
	.configure	= cdns_tor_dp_configure,
	.power_on	= cdns_tor_phy_on,
	.power_off	= cdns_tor_phy_off,
	.owner		= THIS_MODULE,
};

static const struct phy_ops torrent_gen_ops = {
	.power_on	= cdns_tor_phy_on,
	.power_off	= cdns_tor_phy_off,
	.owner		= THIS_MODULE,
};

void debug_print_dtbindings(struct device *dev,
			    struct cdns_tor_inst *inst)
{
	dev_info(dev, "--\n");

	if (inst->phy_type == PHY_TYPE_USB3)
		dev_info(dev, "cdns,phy-type = PHY_TYPE_USB3\n");
	else if (inst->phy_type == PHY_TYPE_PCIE)
		dev_info(dev, "cdns,phy-type = PHY_TYPE_PCIE\n");
	else if (inst->phy_type == PHY_TYPE_DP)
		dev_info(dev, "cdns,phy-type = PHY_TYPE_DP\n");

	dev_info(dev, "cdns,mlane = %d\n", inst->mlane);
	dev_info(dev, "cdns,num-lanes= %d\n", inst->num_lanes);

	if (inst->phy_type == PHY_TYPE_DP) {
		dev_info(dev, "cdns,max-bit-rate = %d\n",
			inst->max_bit_rate);
	}
	dev_info(dev, "--\n");
}

void debug_print_regmap(struct device *dev,
			struct cdns_regmap_cdb_context *ctx, const char *name)
{
	dev_info(dev, "%s: base %08x reg_offset_shift %d",
		name, ctx->base, ctx->reg_offset_shift);
	dev_info(dev, " block_offset %08x block_offset_shift %d\n",
		ctx->block_offset, ctx->block_offset_shift);
}

static int cdns_tor_get_optional(struct device *dev, struct cdns_tor_inst *inst,
				struct device_node *child)
{
	if (of_property_read_u32(child, "cdns,phy-type", &inst->phy_type))
		return -EINVAL;

	if (inst->phy_type == PHY_TYPE_DP) {
		if (of_property_read_u32(child, "cdns,max-bit-rate",
		   &inst->max_bit_rate))
			return -EINVAL;
	}

	if (of_property_read_u32(child, "cdns,mlane", &inst->mlane))
		return -EINVAL;

	if (of_property_read_u32(child, "cdns,num-lanes", &inst->num_lanes))
		return -EINVAL;

	debug_print_dtbindings(dev, inst);

	return 0;
}

static const struct of_device_id cdns_tor_id_table[];

static struct regmap *cdns_regmap_init(struct device *dev, void __iomem *base,
				       u32 block_offset, u8 block_offset_shift,
				       u8 reg_offset_shift,
				       const struct regmap_config *config,
				       const char *name)
{
	struct cdns_regmap_cdb_context *ctx;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->dev = dev;
	ctx->block_offset = block_offset;
	ctx->block_offset_shift = block_offset_shift;
	ctx->reg_offset_shift = reg_offset_shift;
	ctx->base = base + (block_offset << block_offset_shift);

	debug_print_regmap(dev, ctx, name);

	return devm_regmap_init(dev, NULL, ctx, config);
}

static int cdns_regfield_gen_init(struct cdns_tor_phy *tor_phy)
{
	struct device *dev = tor_phy->dev;
	struct regmap_field *field;
	struct regmap *regmap;

	regmap = tor_phy->rmap_gen_phy_config_ctrl;
	field = devm_regmap_field_alloc(dev, regmap, phy_pll_cfg_1);
	if (IS_ERR(field)) {
		dev_err(dev, "PHY_PLL_CFG_1 reg field init failed\n");
		return PTR_ERR(field);
	}
	tor_phy->phy_pll_cfg_1 = field;

	return 0;
}

static int cdns_regmap_init_dp_blocks(struct cdns_tor_phy *tor_phy,
				      void __iomem *base,
				      void __iomem *aux_base,
				      u8 block_offset_shift,
				      u8 reg_offset_shift)
{
	struct device *dev = tor_phy->dev;
	struct regmap *regmap;

	regmap = cdns_regmap_init(dev, base, 0, 0, 0,
				  &cdns_tor_dp_config, "TORRENT DP");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to init dp regmap\n");
		return PTR_ERR(regmap);
	}

	tor_phy->rmap_dp_cmn = regmap;

	regmap = cdns_regmap_init(dev, aux_base, 0, 0, 0,
				  &cdns_tor_dp_aux_config, "TORRENT DP AUX");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to init dp regmap\n");
		return PTR_ERR(regmap);
	}

	tor_phy->rmap_dp_aux = regmap;

	return 0;
}

static int cdns_regmap_init_gen_blocks(struct cdns_tor_phy *tor_phy,
				      void __iomem *base,
				      u8 block_offset_shift,
				      u8 reg_offset_shift)
{
	struct device *dev = tor_phy->dev;
	struct regmap *regmap;
	u32 block_offset;
	int i;

	for (i = 0; i < CDNS_TOR_MAX_LANES; i++) {
		char str[10];

		sprintf(str, "LANE%d", i);

		block_offset = TOR_GEN_LANE_CDB_OFFSET(i);
		regmap = cdns_regmap_init(dev, base, block_offset,
				      block_offset_shift, reg_offset_shift,
				      &cdns_tor_gen_lane_cdb_config[i], str);
		if (IS_ERR(regmap)) {
			dev_err(dev, "Failed to init lane CDB regmap\n");
			return PTR_ERR(regmap);
		}
		tor_phy->rmap_gen_lane_cdb[i] = regmap;
	}

	regmap = cdns_regmap_init(dev, base, TOR_GEN_COMMON_CDB_OFFSET,
				  block_offset_shift, reg_offset_shift,
				  &cdns_tor_gen_common_cdb_config,
				  "COMMON CDB");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to init common CDB regmap\n");
		return PTR_ERR(regmap);
	}
	tor_phy->rmap_gen_cmn_cdb = regmap;

	regmap = cdns_regmap_init(dev, base, TOR_PHY_PLL_CFG_ADDR,
				  block_offset_shift, reg_offset_shift,
				  &cdns_tor_gen_phy_config_ctrl_config,
				  "PHY_PLL");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to init PHY config and control regmap\n");
		return PTR_ERR(regmap);
	}
	tor_phy->rmap_gen_phy_config_ctrl = regmap;

	return 0;
}

static int cdns_tor_phy_probe(struct platform_device *pdev)
{
	struct cdns_tor_phy *tor_phy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct cdns_tor_gen_data *data;
	struct resource *res;
	int i, ret, node = 0, gen_usb_pcie = 0, tor_dp_disabled = 0;
	void __iomem *base, *base_aux;
	struct device_node *dn = dev->of_node, *child;
	u32 ref_clk_rate;

	if (of_get_child_count(dn) == 0)
		return -ENODEV;

	/* Get init data for this PHY */
	match = of_match_device(cdns_tor_id_table, dev);
	if (!match)
		return -EINVAL;

	if (sizeof(cdns_tor_gen_lane_cdb_config)
		 / sizeof(struct regmap_config) < CDNS_TOR_MAX_LANES) {
		dev_err(dev, "lane_cdb_config has less lanes\n");
		return -EINVAL;
	}

	data = (struct cdns_tor_gen_data *)match->data;

	tor_phy = devm_kzalloc(dev, sizeof(*tor_phy), GFP_KERNEL);
	if (!tor_phy)
		return -ENOMEM;
	dev_set_drvdata(dev, tor_phy);
	tor_phy->dev = dev;
	tor_phy->init_data = data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "missing torrent base \"reg\"\n");
		return PTR_ERR(base);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	base_aux = devm_ioremap_resource(dev, res);
	if (IS_ERR(base_aux)) {
		dev_warn(dev, "missing torrent base aux \"reg\"\n");
		tor_dp_disabled = 1;
	}

	/* Initialize regmap for DP */
	ret = cdns_regmap_init_dp_blocks(tor_phy, base, base_aux,
					data->block_offset_shift,
					data->reg_offset_shift);
	if (ret) {
		dev_err(dev, "failed to initialize torrent dp regmap\n");
		return ret;
	}

	/* Initialize regmap for other protocols like PCIe, USB */
	ret = cdns_regmap_init_gen_blocks(tor_phy, base,
					 data->block_offset_shift,
					 data->reg_offset_shift);
	if (ret) {
		dev_err(dev, "failed to initialize torrent gen regmap\n");
		return ret;
	}

	/* Initialize regmap field for PCIe, USB */
	ret = cdns_regfield_gen_init(tor_phy);

	if (ret)
		return ret;

	platform_set_drvdata(pdev, tor_phy);

	tor_phy->clk = devm_clk_get_optional(dev, "phy_clk");
	if (IS_ERR(tor_phy->clk)) {
		dev_err(dev, "failed to get clock phy_clk\n");
		return PTR_ERR(tor_phy->clk);
	}

	tor_phy->phy_rst =
		    devm_reset_control_get_optional(dev, "torrent_reset");
	if (IS_ERR(tor_phy->phy_rst)) {
		dev_err(dev, "failed to get reset\n");
		return PTR_ERR(tor_phy->phy_rst);
	}

	tor_phy->apb_rst = devm_reset_control_get_optional(dev, "torrent_apb");
	if (IS_ERR(tor_phy->apb_rst)) {
		dev_err(dev, "failed to get apb reset\n");
		return PTR_ERR(tor_phy->apb_rst);
	}

	ret = clk_prepare_enable(tor_phy->clk);
	if (ret)
		return ret;

	ref_clk_rate = clk_get_rate(tor_phy->clk);
	if (!ref_clk_rate) {
		dev_err(tor_phy->dev, "Failed to get ref clock rate\n");
		clk_disable_unprepare(tor_phy->clk);
		return -EINVAL;
	}

	/* Enable APB */
	reset_control_deassert(tor_phy->apb_rst);

	for_each_available_child_of_node(dn, child) {
		struct phy *gphy;
		u32 phy_type;

		tor_phy->phys_inst[node].lnk_rst =
			of_reset_control_array_get_optional_exclusive(child);

		if (IS_ERR(tor_phy->phys_inst[node].lnk_rst)) {
			dev_err(dev, "failed to get reset %s\n",
				child->full_name);
			ret = PTR_ERR(tor_phy->phys_inst[node].lnk_rst);
			goto put_child2;
		}

		ret = cdns_tor_get_optional(dev,
					   &tor_phy->phys_inst[node], child);

		if (ret) {
			dev_err(dev, "missing property in node %s\n",
					child->name);
			goto put_child;
		}

		phy_type = tor_phy->phys_inst[node].phy_type;

		/* Create phy for DP protocol */
		if (phy_type == PHY_TYPE_DP) {
			if (tor_dp_disabled) {
				dev_info(dev, "MHDP Torrent disabled\n");
				gphy = NULL;
				continue;
			}

			gphy = devm_phy_create(dev, child, &torrent_dp_ops);

			if (IS_ERR(gphy)) {
				ret = PTR_ERR(gphy);
				goto put_child;
			}

			tor_phy->tor_dp_base = base;
			tor_phy->tor_dp_aux_base = base_aux;

			dev_info(dev,
			 "Creating PHY for DP dp_base %08x dp_aux_base %08x\n",
			  tor_phy->tor_dp_base, tor_phy->tor_dp_aux_base);

			tor_phy->phys_inst[node].phy = gphy;
			tor_phy->phys_inst[node].ref_clk_rate = ref_clk_rate;
			phy_set_drvdata(gphy, &tor_phy->phys_inst[node]);
		} else {
			/* Create phy for all other protocols PCIe, USB etc */
			gphy = devm_phy_create(dev, child, &torrent_gen_ops);
			if (IS_ERR(gphy)) {
				ret = PTR_ERR(gphy);
				goto put_child;
			}

			dev_info(dev,
				"Creating PHY for PCIe/USB base %08x\n",
				 base);

			if (phy_type == PHY_TYPE_PCIE ||
				phy_type == PHY_TYPE_USB3) {
				gen_usb_pcie = 1;
			}

			tor_phy->phys_inst[node].phy = gphy;
			phy_set_drvdata(gphy, &tor_phy->phys_inst[node]);
		}

		if (phy_type == PHY_TYPE_DP && !tor_dp_disabled) {
			/* Initialise DP registers */
			cdns_tor_dp_init(gphy);
		} else {
			/* Initialise PCIe, USB registers */
			cdns_tor_phy_init(gphy);
		}

		node++;
	}
	tor_phy->nsubnodes = node;

	/* configure pll_cfg for USB/PCIe protocols */
	if (gen_usb_pcie)
		cdns_tor_field_write(tor_phy->phy_pll_cfg_1, 0x3);

	pm_runtime_enable(dev);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	reset_control_deassert(tor_phy->phy_rst);

	return PTR_ERR_OR_ZERO(phy_provider);

put_child:
	node++;
put_child2:
	for (i = 0; i < node; i++)
		reset_control_put(tor_phy->phys_inst[i].lnk_rst);

	of_node_put(child);

	clk_disable_unprepare(tor_phy->clk);
	reset_control_assert(tor_phy->apb_rst);
	return ret;
}

static int cdns_tor_phy_remove(struct platform_device *pdev)
{
	struct cdns_tor_phy *phy = dev_get_drvdata(pdev->dev.parent);
	int i;

	reset_control_assert(phy->phy_rst);
	reset_control_assert(phy->apb_rst);

	pm_runtime_disable(&pdev->dev);

	/*
	 * The device level resets will be put automatically.
	 * Need to put the subnode resets here though.
	 */
	for (i = 0; i < phy->nsubnodes; i++) {
		reset_control_assert(phy->phys_inst[i].lnk_rst);
		reset_control_put(phy->phys_inst[i].lnk_rst);
	}
	return 0;
}

/* Configuration 0 name:  PCIe Gen 1,2,3 Single Link */
/* Configuration 0 mnemonic:  PCIe_SL */
/* pcie common configuration */
static struct cdns_reg_pairs cdns_pcie_cmn_regs_ext_ssc[] = {
	{TOR_CMN_SD_CAL_INIT_TMR_ADDR,		0x0002},
	{TOR_CMN_PDIAG_PLL0_CLK_SEL_M0_ADDR,	0x0601},
	{TOR_CMN_PDIAG_PLL0_CLK_SEL_M1_ADDR,	0x0400},
	{TOR_CMN_PDIAG_PLL1_CLK_SEL_M0_ADDR,	0x8600},
	{TOR_CMN_PLL0_VCOCAL_REFTIM_START_ADDR,	0x0C5E},
	{TOR_CMN_PLL1_VCOCAL_REFTIM_START_ADDR,	0x0C5E},
	{TOR_CMN_PLL0_VCOCAL_TCTRL_ADDR,	0x0003},
	{TOR_CMN_PLL1_VCOCAL_TCTRL_ADDR,	0x0003},
	{TOR_CMN_PDIAG_PLL0_CP_PADJ_M0_ADDR,	0x000F},
	{TOR_CMN_PDIAG_PLL0_CP_PADJ_M1_ADDR,	0x000F},
	{TOR_CMN_PDIAG_PLL1_CP_PADJ_M0_ADDR,	0x000F},
	{TOR_CMN_PLL0_VCOCAL_PLLCNT_START_ADDR,	0x0C5E},
	{TOR_CMN_PLL1_VCOCAL_PLLCNT_START_ADDR,	0x0C5E},
	{TOR_CMN_PLL0_LOCK_REFCNT_START_ADDR,	0x00C7},
	{TOR_CMN_PLL1_LOCK_REFCNT_START_ADDR,	0x00C7},
	{TOR_CMN_PLL0_LOCK_PLLCNT_START_ADDR,	0x00C7},
	{TOR_CMN_PLL1_LOCK_PLLCNT_START_ADDR,	0x00C7},
	{TOR_CMN_PLL0_LOCK_PLLCNT_THR_ADDR,	0x0004},
	{TOR_CMN_PLL1_LOCK_PLLCNT_THR_ADDR,	0x0004},
};

/* pcie lane configuration */
static struct cdns_reg_pairs cdns_pcie_ln_regs_ext_ssc[] = {
	{TOR_XCVR_DIAG_HSCLK_SEL_ADDR,		0x0000},
	{TOR_XCVR_DIAG_HSCLK_DIV_ADDR,		0x0001},
	{TOR_XCVR_DIAG_PLLDRC_CTRL_ADDR,	0x0012},
	{TOR_RX_REE_TAP1_CLIP_ADDR,		0x0019},
	{TOR_RX_REE_TAP2TON_CLIP_ADDR,		0x0019},
	{TOR_RX_DIAG_ACYA_ADDR,			0x0001},
};

/* Configuration 1 name:  USB 3.0 / 3.1 Unique SSC */
/* Configuration 1 mnemonic:  USB_US */
/*  usb common configuration */
static struct cdns_reg_pairs cdns_usb_cmn_regs_ext_ssc[] = {
	{TOR_PHY_PIPE_USB3_GEN2_PRE_CFG0_ADDR,	0x0A0A},
	{TOR_PHY_PIPE_USB3_GEN2_POST_CFG0_ADDR,	0x1000},
	{TOR_PHY_PIPE_USB3_GEN2_POST_CFG1_ADDR,	0x0010},
	{TOR_CMN_PDIAG_PLL0_CLK_SEL_M0_ADDR,	0x8600},
	{TOR_CMN_CDIAG_CDB_PWRI_OVRD_ADDR,	0x8200},
	{TOR_CMN_DIAG_BIAS_OVRD1_ADDR,		0x3400},
};

/*  usb lane configuration */
static struct cdns_reg_pairs cdns_usb_ln_regs_ext_ssc[] = {
	{TOR_XCVR_DIAG_HSCLK_SEL_ADDR,		0x0011},
	{TOR_XCVR_DIAG_HSCLK_DIV_ADDR,		0x0001},
	{TOR_XCVR_DIAG_PLLDRC_CTRL_ADDR,	0x00C9},
	{TOR_TX_PSC_A0_ADDR,			0x02FF},
	{TOR_TX_PSC_A1_ADDR,			0x06AF},
	{TOR_TX_PSC_A2_ADDR,			0x06AE},
	{TOR_TX_PSC_A3_ADDR,			0x06AE},
	{TOR_RX_PSC_A0_ADDR,			0x0D1D},
	{TOR_RX_PSC_A1_ADDR,			0x0D1D},
	{TOR_RX_PSC_A2_ADDR,			0x0D00},
	{TOR_RX_PSC_A3_ADDR,			0x0500},
	{TOR_TX_TXCC_CTRL_ADDR,			0x2A82},
	{TOR_TX_TXCC_CPOST_MULT_01_ADDR,	0x0014},
	{TOR_RX_SIGDET_HL_FILT_TMR_ADDR,	0x0013},
	{TOR_RX_REE_GCSM1_CTRL_ADDR,		0x0000},
	{TOR_RX_REE_ATTEN_THR_ADDR,		0x0C02},
	{TOR_RX_REE_SMGM_CTRL1_ADDR,		0x0330},
	{TOR_RX_REE_SMGM_CTRL2_ADDR,		0x0300},
	{TOR_XCVR_DIAG_PSC_OVRD_ADDR,		0x0003},
	{TOR_RX_REE_TAP1_CLIP_ADDR,		0x0019},
	{TOR_RX_REE_TAP2TON_CLIP_ADDR,		0x0019},
	{TOR_RX_DIAG_SIGDET_TUNE_ADDR,		0x1005},
	{TOR_RX_DIAG_NQST_CTRL_ADDR,		0x00F9},
	{TOR_RX_DIAG_DFE_AMP_TUNE_2_ADDR,	0x0C01},
	{TOR_RX_DIAG_DFE_AMP_TUNE_3_ADDR,	0x0002},
	{TOR_RX_DIAG_PI_CAP_ADDR,		0x0000},
	{TOR_RX_DIAG_PI_RATE_ADDR,		0x0031},
	{TOR_RX_DIAG_ACYA_ADDR,			0x0001},
	{TOR_RX_CDRLF_CNFG_ADDR,		0x018C},
	{TOR_RX_CDRLF_CNFG3_ADDR,		0x0003},
};

static const struct cdns_tor_gen_data cdns_map_gen_torrent = {
	0x2,
	0x2,
	ARRAY_SIZE(cdns_pcie_cmn_regs_ext_ssc),
	ARRAY_SIZE(cdns_pcie_ln_regs_ext_ssc),
	ARRAY_SIZE(cdns_usb_cmn_regs_ext_ssc),
	ARRAY_SIZE(cdns_usb_ln_regs_ext_ssc),
	cdns_pcie_cmn_regs_ext_ssc,
	cdns_pcie_ln_regs_ext_ssc,
	cdns_usb_cmn_regs_ext_ssc,
	cdns_usb_ln_regs_ext_ssc
};

static const struct of_device_id cdns_tor_id_table[] = {
	{
		.compatible = "cdns,torrent-phy-t0",
		.data = &cdns_map_gen_torrent,
	},
	{}
};
MODULE_DEVICE_TABLE(of, cdns_tor_id_table);

static struct platform_driver cdns_tor_driver = {
	.probe		= cdns_tor_phy_probe,
	.remove		= cdns_tor_phy_remove,
	.driver		= {
		.name	= "cdns-torrent-phy",
		.of_match_table = cdns_tor_id_table,
	},
};
module_platform_driver(cdns_tor_driver);

MODULE_ALIAS("platform:cdns_torrent");
MODULE_AUTHOR("Cadence Design Systems");
MODULE_DESCRIPTION("CDNS torrent phy driver");
MODULE_LICENSE("GPL v2");
