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

#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "ocdrm_crtc.h"


static inline struct ocdrm_priv *crtc_to_ocdrm(struct drm_crtc *crtc)
{
	return container_of(crtc, struct ocdrm_priv, crtc);
}

static inline struct ocdrm_priv *plane_to_ocdrm(struct drm_plane *plane)
{
	return container_of(plane, struct ocdrm_priv, plane);
}

static void ocdrm_plane_atomic_update(struct drm_plane *plane,
				struct drm_plane_state *old_state)
{
	struct drm_gem_cma_object *obj;
	u32 val;
	uint32_t pixel_format;
	struct ocdrm_priv *priv = plane_to_ocdrm(plane);

	if (!plane->state->crtc || !plane->state->fb)
		return;

	pixel_format = plane->state->fb->pixel_format;

	val = ocdrm_readreg(priv, OCFB_CTRL);
	ocdrm_writereg(priv, OCFB_CTRL, val & ~OCFB_CTRL_VEN);

	if (!drm_atomic_plane_disabling(plane, plane->state)) {
		obj = drm_fb_cma_get_gem_obj(plane->state->fb, 0);
		ocdrm_writereg(priv, OCFB_VBARA, obj->paddr);

		val &= ~(OCFB_CTRL_CD8 | OCFB_CTRL_CD16 | OCFB_CTRL_CD24 | OCFB_CTRL_CD32);
		switch (pixel_format) {
		case DRM_FORMAT_RGB332:
			val |= OCFB_CTRL_CD8;
			//	if (!var->grayscale)
#warning TODO
			val |= OCFB_CTRL_PC;  /* enable palette */
			break;

		case DRM_FORMAT_RGB565:
			dev_dbg(priv->drm_dev->dev, "16 bpp\n");
			val |= OCFB_CTRL_CD16;
			break;

		case DRM_FORMAT_RGB888:
			dev_dbg(priv->drm_dev->dev, "24 bpp\n");
			val |= OCFB_CTRL_CD24;
			break;

		case DRM_FORMAT_XRGB8888:
			dev_dbg(priv->drm_dev->dev, "32 bpp\n");
			val |= OCFB_CTRL_CD32;
			break;

		default:
			dev_err(priv->drm_dev->dev, "Invalid pixelformat specified\n");
			return;
		}

		ocdrm_writereg(priv, OCFB_CTRL, val | OCFB_CTRL_VEN);
	}
}

static void ocdrm_crtc_enable(struct drm_crtc *crtc)
{

	struct ocdrm_priv *priv = crtc_to_ocdrm(crtc);
	if (!priv->clk_enabled)
		clk_prepare_enable(priv->pixel_clock);
	priv->clk_enabled = true;
}

static void ocdrm_crtc_disable(struct drm_crtc *crtc)
{
	struct ocdrm_priv *priv = crtc_to_ocdrm(crtc);

	/* why the plane has been not disabled ? .. we get here from destroy  */
	ocdrm_writereg(priv, OCFB_CTRL, 0);

	if (priv->clk_enabled)
		clk_disable_unprepare(priv->pixel_clock);
	priv->clk_enabled = false;
}


static void ocdrm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	u32 ctrl;
	int ret;
	struct ocdrm_priv *priv = crtc_to_ocdrm(crtc);
	struct drm_display_mode *m = &crtc->state->adjusted_mode;
	uint32_t hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
	uint32_t vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	uint32_t vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
	uint32_t hback_porch = m->crtc_htotal - m->crtc_hsync_end;

	ctrl = ocdrm_readreg(priv, OCFB_CTRL);
	ocdrm_writereg(priv, OCFB_CTRL, ctrl & ~OCFB_CTRL_VEN);

	/* Horizontal timings */
	ocdrm_writereg(priv, OCFB_HTIM, (hsync_len - 1) << 24 |
		      (hback_porch - 1) << 16 | (m->crtc_hdisplay - 1));

	/* Vertical timings */
	ocdrm_writereg(priv, OCFB_VTIM, (vsync_len - 1) << 24 |
		      (vback_porch - 1) << 16 | (m->crtc_vdisplay - 1));

	ocdrm_writereg(priv, OCFB_HVLEN, ((uint32_t)m->crtc_htotal - 1) << 16 |
		(m->crtc_vtotal - 1));

	dev_dbg(priv->drm_dev->dev, "set mode H slen %u, bporch %u, tot %u\n",
		hsync_len, hback_porch, m->crtc_htotal);
	dev_dbg(priv->drm_dev->dev, "set mode V slen %u, bporch %u, tot %u\n",
		vsync_len, vback_porch, m->crtc_vtotal);

	if (m->flags & DRM_MODE_FLAG_NHSYNC)
		ctrl |= OCFB_CTRL_HSL;
	else
		ctrl &= ~OCFB_CTRL_HSL;

	if (m->flags & DRM_MODE_FLAG_NVSYNC)
		ctrl |= OCFB_CTRL_VSL;
	else
		ctrl &= ~OCFB_CTRL_VSL;

	dev_dbg(priv->drm_dev->dev, "VPOL %d, HPOL %d\n",
		m->flags & DRM_MODE_FLAG_NVSYNC,
		m->flags & DRM_MODE_FLAG_NHSYNC);

	/* Set maximum (8) VBL (video memory burst length) and sync polarity. */
	ctrl |= OCFB_CTRL_VBL8;
	ocdrm_writereg(priv, OCFB_CTRL, ctrl & ~OCFB_CTRL_VEN);

	if (priv->clk_enabled) {
		clk_disable_unprepare(priv->pixel_clock);
	}

	ret = clk_set_rate(priv->pixel_clock, m->crtc_clock * 1000);
	if (ret) {
		dev_err(priv->drm_dev->dev, "failed to set pixclk %d\n", ret);
		return;
	}

	if (priv->clk_enabled) {
		clk_prepare_enable(priv->pixel_clock);
	}

	dev_dbg(priv->drm_dev->dev,"pixel clock: %d\n", m->crtc_clock);

	/* if video was enabled, then enable it */
	ocdrm_writereg(priv, OCFB_CTRL, ctrl);
}

static bool ocdrm_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct ocdrm_priv *priv = crtc_to_ocdrm(crtc);

	if (mode->clock < 16000 || mode->clock > 165000) {
		return false;
	}

	adjusted_mode->clock = clk_round_rate(priv->pixel_clock,
					mode->clock * 1000) / 1000;
	return true;
}

static int ocdrm_crtc_atomic_check(struct drm_crtc *crtc,
				struct drm_crtc_state *state)
{
	struct ocdrm_priv *priv = crtc_to_ocdrm(crtc);
	struct drm_display_mode *m = &state->adjusted_mode;
	uint32_t hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
	uint32_t vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	uint32_t vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
	uint32_t hback_porch = m->crtc_htotal - m->crtc_hsync_end;
	int rate;

	if (m->clock < 16000 || m->clock > 165000) {
		return false;
	}

	rate = clk_round_rate(priv->pixel_clock, m->clock * 1000) / 1000;

	if (m->clock != rate)
		return -EINVAL;

	if (hsync_len > 255 || vsync_len > 255 ||
		vback_porch > 255 || hback_porch > 255)
		return -EINVAL;

	return 0;
}

static struct drm_crtc_helper_funcs ocdrm_crtc_helper_funcs = {
	.mode_fixup	= ocdrm_crtc_mode_fixup,
	.mode_set 	= drm_helper_crtc_mode_set,
	.mode_set_base	= drm_helper_crtc_mode_set_base,
	.mode_set_nofb	= ocdrm_crtc_mode_set_nofb,
 	.disable	= ocdrm_crtc_disable,
	.enable		= ocdrm_crtc_enable,
	.atomic_check 	= ocdrm_crtc_atomic_check,
};

static void ocdrm_crtc_destroy(struct drm_crtc *crtc)
{
	ocdrm_crtc_disable(crtc);
	drm_crtc_cleanup(crtc);
}

static struct drm_crtc_funcs ocdrm_crtc_funcs = {
	.page_flip 		= drm_atomic_helper_page_flip,
	.set_config		= drm_atomic_helper_set_config,
	.destroy		= ocdrm_crtc_destroy, // called on unload
	.reset		        = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state
};

static const struct drm_plane_helper_funcs ocdrm_plane_helper_funcs = {
	.atomic_update	= ocdrm_plane_atomic_update,
	.atomic_disable = NULL,
	.prepare_fb = NULL,
	.cleanup_fb = NULL
};

static const struct drm_plane_funcs ocdrm_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

int ocdrm_crtc_create(struct ocdrm_priv *priv)
{
	int ret;
	uint32_t format[] = { DRM_FORMAT_RGB565,
			      DRM_FORMAT_RGB888,
			      DRM_FORMAT_XRGB8888,
	};

	drm_plane_helper_add(&priv->plane, &ocdrm_plane_helper_funcs);

	ret = drm_universal_plane_init(priv->drm_dev, &priv->plane, 0,
				&ocdrm_plane_funcs,
				format, ARRAY_SIZE(format), DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) {
		dev_err(priv->drm_dev->dev, "cannot initialize plane");
		return ret;
	}

	drm_crtc_helper_add(&priv->crtc, &ocdrm_crtc_helper_funcs);
	ret = drm_crtc_init_with_planes(priv->drm_dev, &priv->crtc, &priv->plane, NULL,
					&ocdrm_crtc_funcs, NULL);
	if (ret)
		dev_err(priv->drm_dev->dev, "cannot initialize crtc");
		return ret;

	return 0;
}
