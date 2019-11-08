/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_DRV_H__
#define __TIDSS_DRV_H__

#include <linux/spinlock.h>

typedef u32 dispc_irq_t;

struct tidss_device {
	struct device *dev;		/* Underlying DSS device */
	struct drm_device *ddev;	/* DRM device for DSS */

	struct drm_fbdev_cma *fbdev;

	const struct dispc_features *feat;
	struct dispc_device *dispc;

	unsigned int num_crtcs;
	struct drm_crtc *crtcs[8];

	unsigned int num_planes;
	struct drm_plane *planes[8];

	spinlock_t wait_lock;	/* protects the irq masks */
	dispc_irq_t irq_mask;	/* enabled irqs in addition to wait_list */
	dispc_irq_t irq_uf_mask; /* underflow irq bits for all planes */

	struct drm_atomic_state *saved_state;
};

int tidss_runtime_get(struct tidss_device *tidss);
void tidss_runtime_put(struct tidss_device *tidss);

#endif
