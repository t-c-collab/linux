/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TI j721e Cadence MHDP DP wrapper
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Jyri Sarha <jsarha@ti.com
 */

#ifndef CDNS_MHDP_J721E_H
#define CDNS_MHDP_J721E_H

#include <linux/platform_device.h>
#include "cdns-mhdp-core.h"

struct mhdp_platform_ops;

extern const struct mhdp_platform_ops mhdp_ti_j721e_ops;

#endif /* !CDNS_MHDP_J721E_H */
