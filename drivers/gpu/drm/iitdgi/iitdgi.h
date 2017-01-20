/*
 * IIT DGI video core DRM driver
 * Copyright (c) 2017 Istituto Italiano di Tecnologia
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

#ifndef _IITDGI_DRV_H_
#define _IITDGI_DRV_H_

#include <drm/drm.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>

/* DGI register defines */
#define DGI_ID		0x000
#define DGI_CTRL	0x004
#define DGI_HTIM	0x014
#define DGI_VTIM	0x018
#define DGI_HVLEN	0x01C

#define DGI_ID_MAGIC	0x44474910
#define DGI_CTRL_VEN	0x00000001 /* Video enable */
#define DGI_CTRL_HSL	0x00004000 /* HSync active low */
#define DGI_CTRL_VSL	0x00008000 /* VSync active low */
#define DGI_CTRL_CD16	0x00000000 /* Color Depth 16 */
#define DGI_CTRL_CD24	0x00010000 /* Color Depth 16 */
#define DGI_CTRL_CD32	0x00020000 /* Color Depth 32 */
#define DGI_CTRL_SYNC	0x80000000 /* Sync frame wrt AXI TLAST */

struct iitdgi_priv {
	struct drm_device *drm_dev;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc crtc;
	struct drm_plane plane;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_simple_display_pipe pipe;
	struct dma_chan *dma_chan;
	dma_addr_t dma_addr;
	struct clk *pixel_clock;
	bool clk_enabled;
	void __iomem *regs;
	bool little_endian;
	u32 max_clock;
	u32 max_height;
	u32 max_width;
	bool dma_enabled;
	int dma_size;
	u32 *test_vaddr;
};

extern void iitdgi_writereg(struct iitdgi_priv *priv, loff_t offset, u32 data);
extern u32 iitdgi_readreg(struct iitdgi_priv *priv, loff_t offset);

#endif
