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

#ifndef _OCDRM_DRV_H_
#define _OCDRM_DRV_H_

#include <drm/drm.h>
#include <drm/drm_fb_cma_helper.h>
#include <linux/of.h>
#include <linux/clk.h>

/* OCFB register defines */
#define OCFB_CTRL	0x000
#define OCFB_STAT	0x004
#define OCFB_HTIM	0x008
#define OCFB_VTIM	0x00c
#define OCFB_HVLEN	0x010
#define OCFB_VBARA	0x014
#define OCFB_PALETTE	0x800

#define OCFB_CTRL_VEN	0x00000001 /* Video Enable */
#define OCFB_CTRL_HIE	0x00000002 /* HSync Interrupt Enable */
#define OCFB_CTRL_PC	0x00000800 /* 8-bit Pseudo Color Enable*/
#define OCFB_CTRL_CD8	0x00000000 /* Color Depth 8 */
#define OCFB_CTRL_CD16	0x00000200 /* Color Depth 16 */
#define OCFB_CTRL_CD24	0x00000400 /* Color Depth 24 */
#define OCFB_CTRL_CD32	0x00000600 /* Color Depth 32 */
#define OCFB_CTRL_VBL1	0x00000000 /* Burst Length 1 */
#define OCFB_CTRL_VBL2	0x00000080 /* Burst Length 2 */
#define OCFB_CTRL_VBL4	0x00000100 /* Burst Length 4 */
#define OCFB_CTRL_VBL8	0x00000180 /* Burst Length 8 */
#define OCFB_CTRL_HSL	0x00001000 /* HSync active low */
#define OCFB_CTRL_VSL	0x00002000 /* VSync active low */

#define PALETTE_SIZE	256

struct ocdrm_priv {
	struct drm_device *drm_dev;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc crtc;
	struct drm_plane plane;
	struct drm_encoder encoder;
	struct clk *pixel_clock;
	bool clk_enabled;
	void __iomem *regs;
	bool little_endian;
};

extern void ocdrm_writereg(struct ocdrm_priv *priv, loff_t offset, u32 data);
extern u32 ocdrm_readreg(struct ocdrm_priv *priv, loff_t offset);

#endif
