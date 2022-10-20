/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence MHDP8546 DP MST bridge driver.
 *
 * Copyright (C) 2022 Cadence Design Systems, Inc.
 *
 * Author: Piotr Sroka <piotrs@cadence.com>
 *         Swapnil Jakhade <sjakhade@cadence.com>
 */

#ifndef CDNS_MHDP8546_MST_H
#define CDNS_MHDP8546_MST_H

#include "cdns-mhdp8546-core.h"

int cdns_mhdp_check_mst_status(struct cdns_mhdp_device *mhdp);
void cdns_mhdp_mst_atomic_disable(struct drm_bridge *bridge,
				  struct drm_bridge_state *old_bridge_state);
void cdns_mhdp_mst_probe(struct cdns_mhdp_device *mhdp, const u8 dpcd[DP_RECEIVER_CAP_SIZE]);
int cdns_mhdp_mst_init(struct cdns_mhdp_device *mhdp);
void cdns_mhdp_mst_deinit(struct cdns_mhdp_device *mhdp);

#endif
