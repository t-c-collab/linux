// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP8546 DP MST bridge driver.
 *
 * Copyright (C) 2022 Cadence Design Systems, Inc.
 *
 * Authors: Piotr Sroka <piotrs@cadence.com>
 *          Swapnil Jakhade <sjakhade@cadence.com>
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fixed.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <linux/iopoll.h>
#include <linux/media-bus-format.h>

#include "cdns-mhdp8546-mst.h"

int cdns_mhdp_check_mst_status(struct cdns_mhdp_device *mhdp)
{
	const int max_process_count = 30;
	int process_count = 0;

	for (;;) {
		u8 esi[8] = {};
		bool handled;
		int dret, retry;

		if (process_count == max_process_count) {
			DRM_DEBUG_KMS("Loop exceeded max iterations\n");
			return -EIO;
		}

		process_count++;

		dret = drm_dp_dpcd_read(&mhdp->aux, DP_SINK_COUNT_ESI, esi, 8);
		if (dret != 8) {
			DRM_DEBUG_KMS("failed to get ESI - device may have failed\n");
			return -EIO;
		}

		DRM_DEBUG_KMS("got esi %3ph\n", esi);

		drm_dp_mst_hpd_irq(&mhdp->mst_mgr, esi, &handled);
		if (!handled)
			break;

		for (retry = 0; retry < 3; retry++) {
			int wret;

			wret = drm_dp_dpcd_write(&mhdp->aux, DP_SINK_COUNT_ESI+1, &esi[1], 3);
			if (wret == 3)
				break;
		}
	}

	return 0;
}

static void cdns_mhdp_mst_stream_enable(struct cdns_mhdp_bridge *mhdp_bridge, const bool enable)
{
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	const u8 stream_id = mhdp_bridge->stream_id;
	u32 reg;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_STREAM_CONFIG(stream_id), &reg);

	if (enable) {
		reg |= CDNS_DP_MST_STRM_CFG_STREAM_EN;
		reg &= ~CDNS_DP_MST_STRM_CFG_NO_VIDEO;
	} else {
		reg &= ~CDNS_DP_MST_STRM_CFG_STREAM_EN;
		reg |= CDNS_DP_MST_STRM_CFG_NO_VIDEO;
	}

	cdns_mhdp_reg_write(mhdp, CDNS_DP_STREAM_CONFIG(stream_id), reg);
}

static inline s64 cdns_mhdp_calc_fixed_avg_slots(const u32 pbn, const u32 pbn_div)
{
	s64 fixed_pbn, fixed_pbn_div, fixed_targ_avg_slots;

	fixed_pbn = drm_int2fixp(pbn);
	fixed_pbn_div = drm_int2fixp(pbn_div);
	fixed_targ_avg_slots = drm_fixp_div(fixed_pbn, fixed_pbn_div);

	return fixed_targ_avg_slots;
}

static void cdns_mhdp_set_rate_governing(struct cdns_mhdp_bridge *mhdp_bridge, const bool enable)
{
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	const u8 stream_id = mhdp_bridge->stream_id;
	s64 fixed_targ_avg_slots, fixed_y;
	u32 x, y;

	if (!enable) {
		cdns_mhdp_reg_write(mhdp, CDNS_DP_RATE_GOVERNING_CTRL(stream_id), 0);
		return;
	}

	fixed_targ_avg_slots = cdns_mhdp_calc_fixed_avg_slots(mhdp_bridge->pbn,
							      mhdp->mst_mgr.pbn_div);
	x = mhdp_bridge->pbn / mhdp->mst_mgr.pbn_div;

	fixed_y = fixed_targ_avg_slots - drm_int2fixp(x);
	fixed_y *= 16;

	y = drm_fixp2int_ceil(fixed_y);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_RATE_GOVERNING_CTRL(stream_id),
			    CDNS_DP_RG_TARG_AV_SLOTS_Y(y) |
			    CDNS_DP_RG_TARG_AV_SLOTS_X(x) |
			    CDNS_DP_RG_ENABLE);
}

static struct drm_dp_payload *cdns_mhdp_get_payload(struct cdns_mhdp_bridge *mhdp_bridge)
{
	const int vcpi = mhdp_bridge->connector->port->vcpi.vcpi;
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	int i;

	for (i = 0; i < mhdp->mst_mgr.max_payloads; i++) {
		struct drm_dp_payload *payload = &mhdp->mst_mgr.payloads[i];

		if (payload->vcpi == vcpi)
			return payload;
	}

	return NULL;
}

static int cdns_mhdp_set_act_enable(struct cdns_mhdp_device *mhdp)
{
	u32 mtph_ctrl, mtph_status;
	int ret;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_MTPH_CONTROL, &mtph_ctrl);
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MTPH_CONTROL, mtph_ctrl | CDNS_DP_MTPH_ACT_EN);

	do {
		ret = cdns_mhdp_reg_read(mhdp, CDNS_DP_MTPH_STATUS, &mtph_status);
		if (ret < 0) {
			dev_err(mhdp->dev, "Failed to read CDNS_DP_MTPH_STATUS: %d\n", ret);
			return ret;
		}
	} while ((mtph_status & CDNS_DP_MTPH_ACT_STATUS) != 0);

/*
	//Swap: TODO Add timeout and return error
	dev_err(mhdp->dev, "ACT sequence cannot complete in 30us\n");
	return -EIO;
*/

	return drm_dp_check_act_status(&mhdp->mst_mgr);
}

static int cdns_mhdp_apply_slot_allocation(struct cdns_mhdp_bridge *mhdp_bridge)
{
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	u8 stream_id = mhdp_bridge->stream_id;
	struct drm_dp_payload *payload;

	payload = cdns_mhdp_get_payload(mhdp_bridge);

	if (!payload) {
		DRM_ERROR("payload is not found\n");
		return -EIO;
	}

	cdns_mhdp_reg_write(mhdp, CDNS_DP_MST_SLOT_ALLOCATE(stream_id),
			    CDNS_DP_S_ALLOC_START_SLOT(payload->start_slot) |
			    CDNS_DP_S_ALLOC_END_SLOT(payload->start_slot +
						     payload->num_slots - 1));

	return 0;
}

static void cdns_mhdp_update_slot_allocation(struct cdns_mhdp_bridge *mhdp_bridge)
{
	struct drm_device *dev = mhdp_bridge->base.dev;
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		struct cdns_mhdp_connector *mhdp_connector;

		if (!connector->encoder)
			continue;

		mhdp_connector = to_mhdp_connector(connector);
		if (!mhdp_connector->is_mst_connector)
			continue;

		if (mhdp_connector->bridge->stream_id != -1)
			cdns_mhdp_apply_slot_allocation(mhdp_connector->bridge);
	}
	drm_connector_list_iter_end(&conn_iter);
}

static void cdns_mhdp_mst_connector_destroy(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector;
	struct cdns_mhdp_bridge *mhdp_bridge;

	mhdp_connector = to_mhdp_connector(connector);
	mhdp_bridge = mhdp_connector->bridge;

	drm_connector_cleanup(&mhdp_connector->base);
	drm_bridge_remove(&mhdp_bridge->base);

	drm_dp_mst_put_port_malloc(mhdp_connector->port);

	kfree(mhdp_connector);
	kfree(mhdp_bridge);
}

static int cdns_mhdp_mst_connector_late_register(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(connector);
	int ret;

	ret = drm_dp_mst_connector_late_register(connector,
						 mhdp_connector->port);

	return ret;
}

static void cdns_mhdp_mst_connector_early_unregister(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(connector);

	drm_dp_mst_connector_early_unregister(connector,
					      mhdp_connector->port);
}

static const struct drm_connector_funcs cdns_mhdp_mst_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.reset = drm_atomic_helper_connector_reset,
	.destroy = cdns_mhdp_mst_connector_destroy,
	.late_register = cdns_mhdp_mst_connector_late_register,
	.early_unregister = cdns_mhdp_mst_connector_early_unregister,
};

static int cdns_mhdp_mst_get_modes(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(connector);
	struct cdns_mhdp_device *mhdp = connector_to_mhdp(connector);
	struct edid *edid;
	int num_modes = 0;

	edid = drm_dp_mst_get_edid(connector, &mhdp->mst_mgr, mhdp_connector->port);
	if (edid) {
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else {
		drm_connector_update_edid_property(connector, NULL);
	}

	return num_modes;
}

static struct drm_encoder *cdns_mhdp_mst_atomic_best_encoder(struct drm_connector *connector,
							     struct drm_atomic_state *state)
{
	struct cdns_mhdp_connector *mhdp_connector;

	mhdp_connector = to_mhdp_connector(connector);

	return mhdp_connector->bridge->base.encoder;
}

static int cdns_mhdp_mst_atomic_check(struct drm_connector *connector,
				      struct drm_atomic_state *state)
{
	struct drm_connector_state *new_conn_state =
			drm_atomic_get_new_connector_state(state, connector);
	struct drm_connector_state *old_conn_state =
			drm_atomic_get_old_connector_state(state, connector);
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(connector);
	struct cdns_mhdp_device *mhdp = connector_to_mhdp(connector);
	struct drm_dp_mst_topology_mgr *mst_mgr = &mhdp->mst_mgr;
	struct drm_dp_mst_port *mst_port = mhdp_connector->port;
	struct drm_crtc_state *new_crtc_state;

	if (!old_conn_state->crtc)
		return 0;

	if (new_conn_state->crtc) {
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_conn_state->crtc);
		if (!new_crtc_state ||
		    !drm_atomic_crtc_needs_modeset(new_crtc_state) ||
		    new_crtc_state->enable)
			return 0;
		}

	return drm_dp_atomic_release_vcpi_slots(state, mst_mgr, mst_port);
}

static int
cdns_mhdp_mst_connector_detect(struct drm_connector *connector,
			       struct drm_modeset_acquire_ctx *ctx, bool force)
{
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(connector);
	struct cdns_mhdp_device *mhdp = connector_to_mhdp(connector);

	if (drm_connector_is_unregistered(connector))
		return connector_status_disconnected;

	return drm_dp_mst_detect_port(connector, ctx, &mhdp->mst_mgr,
				      mhdp_connector->port);
}

static const struct drm_connector_helper_funcs cdns_mhdp_mst_connector_helper_funcs = {
	.get_modes = cdns_mhdp_mst_get_modes,
	.detect_ctx = cdns_mhdp_mst_connector_detect,
	.atomic_best_encoder = cdns_mhdp_mst_atomic_best_encoder,
	.atomic_check = cdns_mhdp_mst_atomic_check,
/*
	// Swap: TODO mode_valid can implement calling drm_dp_calc_pbn_mode() here??
	.mode_valid_ctx = cdns_mhdp_mst_mode_valid,
*/
};

static void cdns_mhdp_mst_enable(struct cdns_mhdp_device *mhdp, struct drm_bridge *bridge,
				 const struct drm_display_mode *mode)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_connector *mhdp_connector;
	int ret, slots, stream_id;
	u32 bpp;

	bpp = cdns_mhdp_get_bpp(&mhdp->display_fmt);

	mhdp_connector = mhdp_bridge->connector;
	if (mhdp_bridge->stream_id > -1) {
		DRM_ERROR("stream id is attached before bridge is enabled\n");
		return;
	}

	stream_id = bridge->encoder->crtc->index;

	// Swap: TODO To be done in atomic_check??
	mhdp_bridge->pbn = drm_dp_calc_pbn_mode(mode->clock, bpp, mhdp->is_dsc);

	slots = drm_dp_find_vcpi_slots(&mhdp->mst_mgr, mhdp_bridge->pbn);

	ret = drm_dp_mst_allocate_vcpi(&mhdp->mst_mgr, mhdp_connector->port,
				       mhdp_bridge->pbn, slots);
	if (ret == false) {
		DRM_ERROR("failed to allocate vcpi\n");
		return;
	}

	ret = drm_dp_update_payload_part1(&mhdp->mst_mgr, 1);
	if (ret < 0)
		DRM_ERROR("failed update_payload_part1\n");

	mhdp_bridge->stream_id = stream_id;

	cdns_mhdp_mst_stream_enable(mhdp_bridge, true);

	cdns_mhdp_configure_video(mhdp, bridge, mode);

	ret = cdns_mhdp_apply_slot_allocation(mhdp_bridge);
	if (ret < 0) {
		cdns_mhdp_mst_stream_enable(mhdp_bridge, false);
		mhdp_bridge->stream_id = -1;
		return;
	}

	ret = cdns_mhdp_set_act_enable(mhdp);
	if (ret)
		DRM_ERROR("failed ACT sequence\n");

	cdns_mhdp_set_rate_governing(mhdp_bridge, true);

	drm_dp_update_payload_part2(&mhdp->mst_mgr);
}

static void cdns_mhdp_mst_atomic_enable(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state)
{
	struct cdns_mhdp_device *mhdp = bridge_to_mhdp(bridge);
	struct drm_atomic_state *state = bridge_state->base.state;
	struct drm_crtc_state *crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	const struct drm_display_mode *mode;
	u32 resp;
	int ret;

	dev_dbg(mhdp->dev, "MST bridge enable\n");

	mutex_lock(&mhdp->link_mutex);

	if (mhdp->plugged && !mhdp->link_up) {
		ret = cdns_mhdp_link_up(mhdp);
		if (ret < 0)
			goto out;
	}

	if (!mhdp->is_mst)
		return;

	if (mhdp->info && mhdp->info->ops && mhdp->info->ops->enable)
		mhdp->info->ops->enable(mhdp);

	////// Swap: TODO For stream "n"
	/* Enable VIF clock for stream 0 */
	ret = cdns_mhdp_reg_read(mhdp, CDNS_DPTX_CAR, &resp);
	if (ret < 0) {
		dev_err(mhdp->dev, "Failed to read CDNS_DPTX_CAR %d\n", ret);
		goto out;
	}

	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_CAR,
			    resp | CDNS_VIF_CLK_EN | CDNS_VIF_CLK_RSTN);

	///////

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (WARN_ON(!connector))
		goto out;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		goto out;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (WARN_ON(!crtc_state))
		goto out;

	mode = &crtc_state->adjusted_mode;

	cdns_mhdp_mst_enable(mhdp, bridge, mode);

out:
	mutex_unlock(&mhdp->link_mutex);
	if (ret < 0)
		schedule_work(&mhdp->modeset_retry_work);
}

void cdns_mhdp_mst_atomic_disable(struct drm_bridge *bridge,
				  struct drm_bridge_state *old_bridge_state)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	struct cdns_mhdp_connector *connector = mhdp_bridge->connector;

	/* Swap: TODO Disable VIF clock for stream "n" */

	drm_dp_mst_reset_vcpi_slots(&mhdp->mst_mgr, connector->port);

	drm_dp_update_payload_part1(&mhdp->mst_mgr, 1);

	cdns_mhdp_update_slot_allocation(mhdp_bridge);

	drm_dp_check_act_status(&mhdp->mst_mgr);

	drm_dp_update_payload_part2(&mhdp->mst_mgr);

	drm_dp_mst_deallocate_vcpi(&mhdp->mst_mgr, connector->port);

	cdns_mhdp_set_rate_governing(mhdp_bridge, false);

	cdns_mhdp_mst_stream_enable(mhdp_bridge, false);

	mhdp_bridge->stream_id = -1;
}

static const struct drm_bridge_funcs cdns_mhdp_mst_bridge_funcs = {
	.atomic_enable = cdns_mhdp_mst_atomic_enable,
	.atomic_disable = cdns_mhdp_mst_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static struct cdns_mhdp_bridge *
cdns_mhdp_create_fake_mst_bridge(struct cdns_mhdp_device *mhdp,
				 struct cdns_mhdp_connector *mhdp_connector)
{
	struct cdns_mhdp_mst_cbs *cbs = &mhdp->cbs;
	struct cdns_mhdp_bridge *mhdp_bridge;
	struct drm_encoder *encoder = NULL;

	mhdp_bridge = kzalloc(sizeof(*mhdp_bridge), GFP_KERNEL);
	if (!mhdp_bridge)
		return NULL;

	mhdp_bridge->mhdp = mhdp;
	mhdp_bridge->stream_id = -1;
	mhdp_bridge->connector = mhdp_connector;

	mhdp_bridge->base.funcs = &cdns_mhdp_mst_bridge_funcs;
	if (mhdp->info)
		mhdp_bridge->base.timings = mhdp->info->timings;

	drm_bridge_add(&mhdp_bridge->base);

	if (cbs->funcs.create_mst_encoder)
		encoder = cbs->funcs.create_mst_encoder(cbs->priv_data,
							&mhdp_bridge->base);
	if (encoder) {
		int ret;

		/* Use the same drm device as is in the first encoder */
		encoder->dev = mhdp->bridge.base.encoder->dev;
		encoder->possible_crtcs &= ((1 << CDNS_MHDP_MAX_STREAMS) - 1);

		ret = drm_bridge_attach(encoder, &mhdp_bridge->base, NULL, 0);
		if (ret) {
			dev_err(mhdp->dev, "bridge attaching error %d\n", ret);
			goto err;
		}

		ret = drm_connector_attach_encoder(&mhdp_connector->base, encoder);
		if (ret) {
			dev_err(mhdp->dev, "failed to attach connector to encoder\n");
			goto err;
		}
	}

	return mhdp_bridge;
err:
	kfree(mhdp_bridge);
	return NULL;
}

static struct drm_connector *cdns_mhdp_add_mst_connector(struct drm_dp_mst_topology_mgr *mgr,
							 struct drm_dp_mst_port *port,
							 const char *pathprop)
{
	struct cdns_mhdp_device *mhdp = container_of(mgr, struct cdns_mhdp_device, mst_mgr);
	struct drm_device *dev = mhdp->bridge.base.dev;
	u32 bus_format = MEDIA_BUS_FMT_RGB121212_1X36;
	struct cdns_mhdp_connector *mhdp_connector;
	struct drm_connector_state *conn_state;
	struct drm_connector *connector;
	int ret;

	mhdp_connector = kzalloc(sizeof(*mhdp_connector), GFP_KERNEL);
	if (!mhdp_connector)
		return NULL;

	mhdp_connector->is_mst_connector = true;
	connector = &mhdp_connector->base;
	mhdp_connector->port = port;

	conn_state = kzalloc(sizeof(*conn_state), GFP_KERNEL);
	if (!conn_state)
		goto err;

	__drm_atomic_helper_connector_reset(connector, conn_state);

	ret = drm_connector_init(dev, connector, &cdns_mhdp_mst_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret)
		goto err;

	drm_connector_helper_add(connector, &cdns_mhdp_mst_connector_helper_funcs);

	ret = drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);
	if (ret)
		goto err;

	mhdp_connector->bridge = cdns_mhdp_create_fake_mst_bridge(mhdp, mhdp_connector);
	if (!mhdp_connector->bridge)
		goto err;

	drm_object_attach_property(&connector->base, dev->mode_config.path_property, 0);
	drm_object_attach_property(&connector->base, dev->mode_config.tile_property, 0);

	ret = drm_connector_set_path_property(connector, pathprop);

	if (ret)
		DRM_ERROR("set path property failed\n");

	drm_dp_mst_get_port_malloc(port);

	return connector;

err:
	kfree(mhdp_connector);
	return NULL;
}

static const struct drm_dp_mst_topology_cbs mst_cbs = {
	.add_connector = cdns_mhdp_add_mst_connector,
};

static void cdns_mhdp_set_mst_enable(struct cdns_mhdp_device *mhdp, bool enable)
{
	u32 reg_val;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, &reg_val);

	if (enable)
		reg_val |= CDNS_DP_MST_EN;
	else
		reg_val &= ~CDNS_DP_MST_EN;

	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, reg_val);
}

void cdns_mhdp_mst_probe(struct cdns_mhdp_device *mhdp, const u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	bool is_mst;

	if (!mhdp->can_mst)
		return;

	is_mst = drm_dp_read_mst_cap(&mhdp->aux, dpcd);
	if (is_mst != mhdp->is_mst) {
		mhdp->is_mst = is_mst;
		cdns_mhdp_set_mst_enable(mhdp, mhdp->is_mst);
		drm_dp_mst_topology_mgr_set_mst(&mhdp->mst_mgr, mhdp->is_mst);
	}
}

int cdns_mhdp_mst_init(struct cdns_mhdp_device *mhdp)
{
	struct cdns_mhdp_bridge *mhdp_bridge = &mhdp->bridge;
	struct drm_device *dev = mhdp_bridge->base.dev;
	struct cdns_mhdp_connector *connector = mhdp_bridge->connector;
	int ret;

	mhdp->mst_mgr.cbs = &mst_cbs;
	ret = drm_dp_mst_topology_mgr_init(&mhdp->mst_mgr, dev,
					   &mhdp->aux, 16, CDNS_MHDP_MAX_STREAMS,
					   mhdp->host.lanes_cnt,
					   mhdp->host.link_rate,
					   connector->base.base.id);
	mhdp->can_mst = ret ? false : true;
	mhdp->is_mst = false;
	mhdp_bridge->stream_id = -1;

	return ret;
}

void cdns_mhdp_mst_deinit(struct cdns_mhdp_device *mhdp)
{
	mhdp->is_mst = false;

	if (!mhdp->can_mst)
		return;

	drm_dp_mst_topology_mgr_destroy(&mhdp->mst_mgr);
}
