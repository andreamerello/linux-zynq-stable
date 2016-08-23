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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "ocdrm_drv.h"

#define DRIVER_NAME	"ocdrm"
#define DRIVER_DESC	"OpenCores DRM"
#define DRIVER_DATE	"20160629"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static inline struct ocdrm_priv *pipe_to_ocdrm(struct drm_simple_display_pipe *pipe)
{
       return container_of(pipe, struct ocdrm_priv, pipe);
}

static void ocdrm_output_poll_changed(struct drm_device *drm)
{
	struct ocdrm_priv *priv = drm->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

u32 ocdrm_readreg(struct ocdrm_priv *priv, loff_t offset)
{
	if (priv->little_endian)
		return ioread32(priv->regs + offset);
	else
		return ioread32be(priv->regs + offset);
}

void ocdrm_writereg(struct ocdrm_priv *priv, loff_t offset, u32 data)
{
	if (priv->little_endian)
		iowrite32(data, priv->regs + offset);
	else
		iowrite32be(data, priv->regs + offset);
}

static void ocdrm_detect_endian(struct ocdrm_priv *priv)
{
	priv->little_endian = false;
	ocdrm_writereg(priv, OCFB_VBARA, 0xfffffff0);
	if (ocdrm_readreg(priv, OCFB_VBARA) != 0xfffffff0)
		priv->little_endian = true;

	ocdrm_writereg(priv, OCFB_VBARA, 0x0);
}

static void ocdrm_send_vblank_event(struct drm_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event)
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
	crtc->state->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void ocdrm_enable(struct drm_simple_display_pipe *pipe,
			struct drm_crtc_state *crtc_state)
{
	u32 ctrl;
	struct ocdrm_priv *priv = pipe_to_ocdrm(pipe);

	if (!priv->clk_enabled)
		clk_prepare_enable(priv->pixel_clock);
	priv->clk_enabled = true;

	ctrl = ocdrm_readreg(priv, OCFB_CTRL);
	ocdrm_writereg(priv, OCFB_CTRL, ctrl | OCFB_CTRL_VEN);
}

static void ocdrm_disable(struct drm_simple_display_pipe *pipe)
{
	u32 ctrl;
	struct ocdrm_priv *priv = pipe_to_ocdrm(pipe);

	ctrl = ocdrm_readreg(priv, OCFB_CTRL);
	ocdrm_writereg(priv, OCFB_CTRL, ctrl & ~OCFB_CTRL_VEN);

	if (priv->clk_enabled)
		clk_disable_unprepare(priv->pixel_clock);
	priv->clk_enabled = false;
}

static void ocdrm_update(struct drm_simple_display_pipe *pipe,
			struct drm_plane_state *plane_state)
{
	struct drm_gem_cma_object *obj;
	int ret;
	u32 ctrl;
	uint32_t pixel_format;
	int hgate;
	uint32_t hsync_len;
	uint32_t vsync_len;
	uint32_t vback_porch;
	uint32_t hback_porch;
	unsigned long clock;
	struct drm_display_mode *m;
	struct ocdrm_priv *priv = pipe_to_ocdrm(pipe);
	struct drm_crtc *crtc = plane_state->crtc;

	ctrl = ocdrm_readreg(priv, OCFB_CTRL);

	if (drm_atomic_plane_disabling(plane_state->plane, plane_state)) {
		ocdrm_writereg(priv, OCFB_CTRL, ctrl & ~OCFB_CTRL_VEN);
	} else	if (crtc && plane_state->fb) {
		ocdrm_writereg(priv, OCFB_CTRL, ctrl & ~OCFB_CTRL_VEN);

		m = &crtc->state->adjusted_mode;
		hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
		vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
		vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
		hback_porch = m->crtc_htotal - m->crtc_hsync_end;
		hgate = m->crtc_hdisplay;
		pixel_format = plane_state->fb->pixel_format;

		obj = drm_fb_cma_get_gem_obj(plane_state->fb, 0);
		ocdrm_writereg(priv, OCFB_VBARA, obj->paddr);

		ctrl &= ~(OCFB_CTRL_CD8 | OCFB_CTRL_CD16 |
			OCFB_CTRL_CD24 | OCFB_CTRL_CD32);
		ctrl &= ~(OCFB_CTRL_VBL8 | OCFB_CTRL_VBL4 |
			OCFB_CTRL_VBL2 | OCFB_CTRL_VBL1);

		switch (pixel_format) {
			/* TODO
			 *case DRM_FORMAT_RGB332:
			 *	hgate /= 4;
			 *	val |= OCFB_CTRL_CD8;
			 *	val |= OCFB_CTRL_PC;
			 *	break;
			 */

		case DRM_FORMAT_RGB565:
			dev_dbg(priv->drm_dev->dev, "16 bpp\n");
			hgate /= 2;
			ctrl |= OCFB_CTRL_CD16;
			break;

		case DRM_FORMAT_RGB888:
			dev_dbg(priv->drm_dev->dev, "24 bpp\n");
			hgate = hgate * 3 / 4;
			ctrl |= OCFB_CTRL_CD24;
			break;

		case DRM_FORMAT_XRGB8888:
			dev_dbg(priv->drm_dev->dev, "32 bpp\n");
			ctrl |= OCFB_CTRL_CD32;
			break;

		default:
			dev_err(priv->drm_dev->dev, "Invalid pixelformat specified\n");
			return;
		}

		if ((0 == (obj->paddr & 0x1f)) && (0 == (hgate % 8))) {
			dev_dbg(priv->drm_dev->dev, "dma burst 8 cycles\n");
			ctrl |= OCFB_CTRL_VBL8;
		} else if ((0 == (obj->paddr & 0xf)) && (0 == (hgate % 4))) {
			dev_dbg(priv->drm_dev->dev, "dma burst 4 cycles\n");
			ctrl |= OCFB_CTRL_VBL4;
		} else if ((0 == (obj->paddr & 0x7)) && (0 == (hgate % 2))) {
			dev_dbg(priv->drm_dev->dev, "dma burst 2 cycles\n");
			ctrl |= OCFB_CTRL_VBL2;
		} else {
			dev_dbg(priv->drm_dev->dev, "dma burst 1 cycle\n");
			ctrl |= OCFB_CTRL_VBL1;
		}

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

		/* Set sync polarity. */
		ocdrm_writereg(priv, OCFB_CTRL, ctrl);

		if (priv->clk_enabled)
			clk_disable_unprepare(priv->pixel_clock);

		clock = clk_round_rate(priv->pixel_clock,
				m->clock * 1000);

		dev_dbg(priv->drm_dev->dev, "pixel clock: %u, rounded to %lu\n",
			m->clock, clock);

		ret = clk_set_rate(priv->pixel_clock, clock);
		if (ret) {
			dev_err(priv->drm_dev->dev, "failed to set pixclk %d\n", ret);
			return;
		}

		if (priv->clk_enabled) {
			clk_prepare_enable(priv->pixel_clock);
		}

		/* if video was enabled, then re-enable it */
		ocdrm_writereg(priv, OCFB_CTRL, ctrl);
	}

	ocdrm_send_vblank_event(&pipe->crtc);
}

static int ocdrm_check(struct drm_simple_display_pipe *pipe,
		     struct drm_plane_state *plane_state,
		     struct drm_crtc_state *crtc_state)
{
	struct drm_display_mode *m = &crtc_state->adjusted_mode;
	uint32_t hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
	uint32_t vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	uint32_t vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
	uint32_t hback_porch = m->crtc_htotal - m->crtc_hsync_end;

	if (m->clock < 16000 || m->clock > 165000)
		return false;

	if (hsync_len > 255 || vsync_len > 255 ||
		vback_porch > 255 || hback_porch > 255)
		return -EINVAL;

	return 0;
}

static struct drm_mode_config_funcs ocdrm_mode_config_funcs = {
	.output_poll_changed	= ocdrm_output_poll_changed,
	.fb_create		= drm_fb_cma_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void ocdrm_mode_config_init(struct ocdrm_priv *priv)
{
	struct drm_device *dev = priv->drm_dev;

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 1500;
	dev->mode_config.max_height = 1500;

	dev->mode_config.funcs = &ocdrm_mode_config_funcs;
}

static struct drm_simple_display_pipe_funcs ocdrm_pipe_funcs = {
	.enable		= ocdrm_enable,
	.disable	= ocdrm_disable,
	.check		= ocdrm_check,
	.update		= ocdrm_update,
};

static int ocdrm_load(struct drm_device *dev)
{
	struct ocdrm_priv *priv;
	int ret;
	struct resource *res;
	struct drm_bridge *bridge;
	struct device_node *ep, *bridge_node;
	struct platform_device *pdev = to_platform_device(dev->dev);
	uint32_t ocdrm_formats[] = {
		DRM_FORMAT_RGB565,
		DRM_FORMAT_RGB888,
		DRM_FORMAT_XRGB8888,
	};

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev->dev_private = priv;
	priv->drm_dev = dev;

	priv->pixel_clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->pixel_clock))
		return -EPROBE_DEFER;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)))
		return -ENOMEM;

	ocdrm_detect_endian(priv);

	drm_mode_config_init(dev);
	ocdrm_mode_config_init(priv);

	drm_simple_display_pipe_init(dev, &priv->pipe, &ocdrm_pipe_funcs,
				ocdrm_formats, ARRAY_SIZE(ocdrm_formats), NULL);

	ep = of_graph_get_next_endpoint(priv->drm_dev->dev->of_node, NULL);
	if (!ep)
		return -ENODEV;

	bridge_node = of_graph_get_remote_port_parent(ep);
	if (!bridge_node)
		return -ENODEV;

	bridge = of_drm_find_bridge(bridge_node);
	if (!bridge)
		return -EPROBE_DEFER;

	drm_simple_display_pipe_attach_bridge(&priv->pipe, bridge);

	drm_mode_config_reset(dev);
	drm_kms_helper_poll_init(dev);

	priv->fbdev = drm_fbdev_cma_init(dev, 16, dev->mode_config.num_crtc,
					dev->mode_config.num_connector);

	if (IS_ERR(priv->fbdev)) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		ret = PTR_ERR(priv->fbdev);
		goto err_crtc;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

err_crtc:
	drm_mode_config_cleanup(dev);
	return ret;
}

static int ocdrm_unload(struct drm_device *dev)
{
	struct ocdrm_priv *priv = dev->dev_private;

	if (priv->fbdev) {
		drm_fbdev_cma_fini(priv->fbdev);
		priv->fbdev = NULL;
	}
	drm_kms_helper_poll_fini(dev);
	drm_vblank_cleanup(dev);
	drm_mode_config_cleanup(dev);

	return 0;
}

static void ocdrm_lastclose(struct drm_device *dev)
{
	struct ocdrm_priv *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static const struct file_operations ocdrm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= drm_gem_cma_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.llseek		= no_llseek,
	.release	= drm_release,
};

static struct drm_driver ocdrm_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM |
		DRIVER_ATOMIC | DRIVER_PRIME,
	.lastclose		= ocdrm_lastclose,
	.fops			= &ocdrm_driver_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.get_vblank_counter	= drm_vblank_no_hw_counter,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

};

static const struct of_device_id ocdrm_of_match[] = {
	{ .compatible = "opencores,ocfb-drm", },
	{},
};
MODULE_DEVICE_TABLE(of, ocdrm_of_match);

static int ocdrm_probe(struct platform_device *pdev)
{
	int ret;
	struct drm_device *drm;

	drm = drm_dev_alloc(&ocdrm_drm_driver, &pdev->dev);
	if (!drm)
		return -ENOMEM;

	ret = ocdrm_load(drm);
	if (ret)
		goto err_unref;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	return 0;

err_unload:
	ocdrm_unload(drm);
err_unref:
	drm_dev_unref(drm);

	return ret;
}

static int ocdrm_remove(struct platform_device *pdev)
{
	struct ocdrm_priv *priv = platform_get_drvdata(pdev);
	struct drm_device *drm = priv->drm_dev;

	drm_dev_unregister(drm);
	ocdrm_unload(drm);
	drm_dev_unref(drm);

	return 0;
}

static struct platform_driver ocdrm_platform_driver = {
	.driver = {
		.name = "oc-drm",
		.owner = THIS_MODULE,
		.of_match_table = ocdrm_of_match,
	},
	.probe = ocdrm_probe,
	.remove = ocdrm_remove,
};
module_platform_driver(ocdrm_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrea Merello <andrea.merello@gmail.com>");
MODULE_DESCRIPTION("OpenCores DRM driver");
