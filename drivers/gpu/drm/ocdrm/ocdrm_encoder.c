/*
 * Open cores VGA/LCD 2.0 core DRM driver
 * Copyright (c) 2016 Istituto Italiano di Tecnologia
 * Electronic Design Lab.
 *
 * Author: Andrea Merello <andrea.merello@gmail.com>
 *
 * Based on the following drivers:
 *   - Analog Devices AXI HDMI DRM driver, which is
 *     Copyright 2012 Analog Devices Inc.
 *
 *   - ARC PGU DRM driver.
 *     Copyright (C) 2016 Synopsys, Inc. (www.synopsys.com)
 *
 *   - ARM HDLCD Driver
 *     Copyright (C) 2013-2015 ARM Limited
 *
 *   - Atmel atmel-hlcdc driver, which is
 *     Copyright (C) 2014 Traphandler
 *     Copyright (C) 2014 Free Electrons
 *
 *   - OpenCores VGA/LCD 2.0 core frame buffer driver
 *     Copyright (C) 2013 Stefan Kristiansson, stefan.kristiansson@saunalahti.fi
 *
 *   - R-Car Display Unit DRM driver
 *     Copyright (C) 2013-2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include "ocdrm_encoder.h"

static void ocdrm_encoder_nop(struct drm_encoder *encoder)
{
}

static const struct drm_encoder_funcs ocdrm_encoder_funcs = {
	.destroy = drm_encoder_cleanup
};

static struct drm_encoder_helper_funcs ocdrm_encoder_helper_funcs = {
	.commit = ocdrm_encoder_nop,
	.enable  = ocdrm_encoder_nop,
	.disable = ocdrm_encoder_nop
};

int ocdrm_encoder_create(struct ocdrm_priv *priv)
{
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct device_node *ep, *bridge_node;
	int ret;

	encoder = &priv->encoder;
	encoder->possible_crtcs = 1 << drm_crtc_index(&priv->crtc);

	drm_encoder_helper_add(encoder, &ocdrm_encoder_helper_funcs);
	ret = drm_encoder_init(priv->drm_dev, encoder, &ocdrm_encoder_funcs,
			DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	ep = of_graph_get_next_endpoint(priv->drm_dev->dev->of_node, NULL);
	if (!ep)
		return -ENODEV;

	bridge_node = of_graph_get_remote_port_parent(ep);
	if (!bridge_node)
		return -ENODEV;

	bridge = of_drm_find_bridge(bridge_node);
	if (!bridge)
		return -EPROBE_DEFER;

	bridge->encoder = encoder;
	encoder->bridge = bridge;
	drm_bridge_attach(priv->drm_dev, bridge);
	return 0;
}
