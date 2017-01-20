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

#include "iitdgi.h"

#define DRIVER_NAME	"iitdgi"
#define DRIVER_DESC	"IIT DGI DRM"
#define DRIVER_DATE	"20170118"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static unsigned int test_mode = 0;
static unsigned int test_pattern = 0;

static inline struct iitdgi_priv *pipe_to_iitdgi(struct drm_simple_display_pipe *pipe)
{
       return container_of(pipe, struct iitdgi_priv, pipe);
}

static void iitdgi_output_poll_changed(struct drm_device *drm)
{
	struct iitdgi_priv *priv = drm->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

u32 iitdgi_readreg(struct iitdgi_priv *priv, loff_t offset)
{
	return ioread32(priv->regs + offset);
}

void iitdgi_writereg(struct iitdgi_priv *priv, loff_t offset, u32 data)
{
	iowrite32(data, priv->regs + offset);
}

static void iitdgi_send_vblank_event(struct drm_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (crtc->state->event)
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
	crtc->state->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int iitdgi_dma_disable(struct iitdgi_priv *priv)
{
	int was_enabled = priv->dma_enabled;
	dmaengine_terminate_all(priv->dma_chan);
	priv->dma_enabled = false;

	return was_enabled;
}

static int iitdgi_dma_enable(struct iitdgi_priv *priv)
{
	struct dma_async_tx_descriptor *dma_desc;
	dma_cookie_t cookie;
	int ret;

	iitdgi_dma_disable(priv);
	priv->dma_enabled = true;
	if (!priv->dma_size)
		return 0;

	dev_dbg(priv->drm_dev->dev, "DMA size %d\n", priv->dma_size);
	dma_desc = dmaengine_prep_dma_cyclic(priv->dma_chan, priv->dma_addr,
					priv->dma_size, priv->dma_size,
					DMA_MEM_TO_DEV, DMA_CTRL_ACK);

	if (!dma_desc) {
		dev_err(priv->drm_dev->dev, "cannot prep dma\n");
		return -1;
	}
	cookie = dmaengine_submit(dma_desc);

        ret = dma_submit_error(cookie);

	dma_async_issue_pending(priv->dma_chan);
	return ret;
}

static void iitdgi_enable(struct drm_simple_display_pipe *pipe,
			struct drm_crtc_state *crtc_state)
{
	u32 ctrl;
	struct iitdgi_priv *priv = pipe_to_iitdgi(pipe);

	dev_dbg(priv->drm_dev->dev, "Enabling..");
	if (!priv->clk_enabled)
		clk_prepare_enable(priv->pixel_clock);
	priv->clk_enabled = true;

	ctrl = iitdgi_readreg(priv, DGI_CTRL);
	iitdgi_writereg(priv, DGI_CTRL, ctrl | DGI_CTRL_VEN | DGI_CTRL_SYNC);
	iitdgi_dma_enable(priv);

		ctrl = iitdgi_readreg(priv, DGI_CTRL);
}

static void iitdgi_disable(struct drm_simple_display_pipe *pipe)
{
	u32 ctrl;
	struct iitdgi_priv *priv = pipe_to_iitdgi(pipe);

	dev_dbg(priv->drm_dev->dev, "Disabling..");
	iitdgi_dma_disable(priv);
	ctrl = iitdgi_readreg(priv, DGI_CTRL);
	iitdgi_writereg(priv, DGI_CTRL, ctrl & ~DGI_CTRL_VEN);

	if (priv->clk_enabled)
		clk_disable_unprepare(priv->pixel_clock);
	priv->clk_enabled = false;
}

static void iitdgi_update(struct drm_simple_display_pipe *pipe,
			struct drm_plane_state *plane_state)
{
	struct drm_gem_cma_object *obj;
	int ret;
	u32 ctrl;
	uint32_t pixel_format;
	uint32_t hsync_len;
	uint32_t vsync_len;
	uint32_t vback_porch, vfront_porch;
	uint32_t hback_porch, hfront_porch;
	unsigned long clock;
	struct drm_display_mode *m;
	bool dma_enable;
	struct iitdgi_priv *priv = pipe_to_iitdgi(pipe);
	struct drm_crtc *crtc = plane_state->crtc;

	ctrl = iitdgi_readreg(priv, DGI_CTRL);

	if (drm_atomic_plane_disabling(plane_state->plane, plane_state)) {
		iitdgi_dma_disable(priv);
		iitdgi_writereg(priv, DGI_CTRL, ctrl & ~DGI_CTRL_VEN);
	} else	if (crtc && plane_state->fb) {
		dma_enable = iitdgi_dma_disable(priv);
		iitdgi_writereg(priv, DGI_CTRL, ctrl & ~DGI_CTRL_VEN);
		m = &crtc->state->adjusted_mode;
		hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
		vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
		vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
		hback_porch = m->crtc_htotal - m->crtc_hsync_end;
		hfront_porch = m->crtc_hsync_start - m->crtc_hdisplay;
		vfront_porch = m->crtc_vsync_start - m->crtc_vdisplay;
		pixel_format = plane_state->fb->pixel_format;

		obj = drm_fb_cma_get_gem_obj(plane_state->fb, 0);
		if (!test_mode)
			priv->dma_addr = obj->paddr;

		ctrl &= ~(DGI_CTRL_CD16 | DGI_CTRL_CD24 | DGI_CTRL_CD32);
		dev_dbg(priv->drm_dev->dev, "Video %dx%d",
			m->crtc_hdisplay, m->crtc_vdisplay);
		priv->dma_size = m->crtc_hdisplay * m->crtc_vdisplay;
		switch (pixel_format) {
		case DRM_FORMAT_RGB565:
			dev_dbg(priv->drm_dev->dev, "16 bpp\n");
			ctrl |= DGI_CTRL_CD16;
			priv->dma_size *= 2;
			break;

		case DRM_FORMAT_RGB888:
			dev_dbg(priv->drm_dev->dev, "24 bpp\n");
			ctrl |= DGI_CTRL_CD24;
			priv->dma_size *= 3;
			break;

		case DRM_FORMAT_XRGB8888:
			dev_dbg(priv->drm_dev->dev, "32 bpp\n");
			ctrl |= DGI_CTRL_CD32;
			priv->dma_size *= 4;
			break;

		default:
			dev_err(priv->drm_dev->dev, "Invalid pixelformat specified\n");
			return;
		}

		/* Horizontal timings */
		iitdgi_writereg(priv, DGI_HTIM, (hsync_len - 1) << 23 |
			(hback_porch - 1) << 11 | (hfront_porch - 1));

		/* Vertical timings */
		iitdgi_writereg(priv, DGI_VTIM, (vsync_len - 1) << 23 |
			(vback_porch - 1) << 11 | (vfront_porch - 1));

		iitdgi_writereg(priv, DGI_HVLEN, ((uint32_t)m->crtc_vdisplay - 1) << 16 |
			(m->crtc_hdisplay - 1));

		dev_dbg(priv->drm_dev->dev, "set mode H slen %u, bporch %u, tot %u\n",
			hsync_len, hback_porch, m->crtc_htotal);
		dev_dbg(priv->drm_dev->dev, "set mode V slen %u, bporch %u, tot %u\n",
			vsync_len, vback_porch, m->crtc_vtotal);

		if (m->flags & DRM_MODE_FLAG_NHSYNC)
			ctrl |= DGI_CTRL_HSL;
		else
			ctrl &= ~DGI_CTRL_HSL;

		if (m->flags & DRM_MODE_FLAG_NVSYNC)
			ctrl |= DGI_CTRL_VSL;
		else
			ctrl &= ~DGI_CTRL_VSL;

		dev_dbg(priv->drm_dev->dev, "VPOL %d, HPOL %d\n",
			m->flags & DRM_MODE_FLAG_NVSYNC,
			m->flags & DRM_MODE_FLAG_NHSYNC);

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
		if (dma_enable)
			iitdgi_dma_enable(priv);
		iitdgi_writereg(priv, DGI_CTRL, ctrl);

		ctrl = iitdgi_readreg(priv, DGI_CTRL);
	}

	iitdgi_send_vblank_event(&pipe->crtc);
}

static int iitdgi_check(struct drm_simple_display_pipe *pipe,
		     struct drm_plane_state *plane_state,
		     struct drm_crtc_state *crtc_state)
{
	struct drm_display_mode *m = &crtc_state->adjusted_mode;
	uint32_t hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;
	uint32_t vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	uint32_t vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
	uint32_t hback_porch = m->crtc_htotal - m->crtc_hsync_end;
	uint32_t hfront_porch = m->crtc_hsync_start - m->crtc_hdisplay;
	uint32_t vfront_porch = m->crtc_vsync_start - m->crtc_vdisplay;

	struct iitdgi_priv *priv = pipe_to_iitdgi(pipe);
	dev_dbg(priv->drm_dev->dev, "Check..");

	if ((priv->max_clock && (m->clock > priv->max_clock)) || (m->clock < 1))
		return -EINVAL;

	if (hsync_len > 512 || vsync_len > 512 ||
		vback_porch > 512 || hback_porch > 512 ||
		vfront_porch > 512 || hfront_porch > 512)
		return -EINVAL;
	dev_dbg(priv->drm_dev->dev, "Check OK..");
	return 0;
}

static struct drm_mode_config_funcs iitdgi_mode_config_funcs = {
	.output_poll_changed	= iitdgi_output_poll_changed,
	.fb_create		= drm_fb_cma_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void iitdgi_mode_config_init(struct iitdgi_priv *priv)
{
	struct drm_device *dev = priv->drm_dev;

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = priv->max_width;
	dev->mode_config.max_height = priv->max_height;

	dev->mode_config.funcs = &iitdgi_mode_config_funcs;
}

static struct drm_simple_display_pipe_funcs iitdgi_pipe_funcs = {
	.enable		= iitdgi_enable,
	.disable	= iitdgi_disable,
	.check		= iitdgi_check,
	.update		= iitdgi_update,
};

static int iitdgi_load(struct drm_device *dev)
{
	struct iitdgi_priv *priv;
	int ret;
	int i;
	u32 id;
	struct resource *res;
	struct drm_bridge *bridge;
	struct device_node *ep, *bridge_node;
	struct platform_device *pdev = to_platform_device(dev->dev);
	uint32_t iitdgi_formats[] = {
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

	ret = of_property_read_u32(dev->dev->of_node,
				"max-pixclk", &priv->max_clock);
	if (ret < 0)
		priv->max_clock = 0;

	ret = of_property_read_u32(dev->dev->of_node,
				"max-width", &priv->max_width);
	if (ret < 0) {
		DRM_ERROR("missing max-width property in DT\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->dev->of_node,
				"max-height", &priv->max_height);
	if (ret < 0) {
		DRM_ERROR("missing max-height property in DT\n");
		return -EINVAL;
	}

	dev_dbg(dev->dev, "max clock: %d, max H: %d, max W: %d",
		priv->max_clock, priv->max_height, priv->max_width);

	priv->dma_chan = dma_request_slave_channel(&pdev->dev, "video");

	if (IS_ERR_OR_NULL(priv->dma_chan)) {
		priv->dma_chan = NULL;
		dev_err(&pdev->dev, "failed to get DMA channel");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)))
		return -ENOMEM;

	id = iitdgi_readreg(priv, DGI_ID);
	if (id != DGI_ID_MAGIC) {
		dev_err(&pdev->dev, "DGI error. Magic is %x, expected %x",
			id, DGI_ID_MAGIC);
		return -ENODEV;
	}
	dev_dbg(&pdev->dev, "DGI found!");

	drm_mode_config_init(dev);
	iitdgi_mode_config_init(priv);

	drm_simple_display_pipe_init(dev, &priv->pipe, &iitdgi_pipe_funcs,
				iitdgi_formats, ARRAY_SIZE(iitdgi_formats), NULL);

	ep = of_graph_get_next_endpoint(priv->drm_dev->dev->of_node, NULL);
	if (!ep)
		return -ENODEV;

	bridge_node = of_graph_get_remote_port_parent(ep);
	if (!bridge_node)
		return -ENODEV;

	bridge = of_drm_find_bridge(bridge_node);
	of_node_put(bridge_node);
	if (!bridge)
		return -EPROBE_DEFER;

	ret = drm_simple_display_pipe_attach_bridge(&priv->pipe, bridge);
	if (ret) {
		DRM_ERROR("failed to attach the bridge\n");
		return ret;
	}
	priv->bridge = bridge;

	drm_mode_config_reset(dev);
	drm_kms_helper_poll_init(dev);


	priv->fbdev = drm_fbdev_cma_init(dev, 16, dev->mode_config.num_crtc,
						dev->mode_config.num_connector);
	if (test_mode) {
		priv->test_vaddr = (u32*)dma_alloc_coherent(dev->dev,
					priv->max_height * priv->max_width * 4,
					&priv->dma_addr, GFP_KERNEL);

		for (i = 0; i < priv->max_height * priv->max_width; i++)
			priv->test_vaddr[i] = i;
	}

	if (test_pattern) {
		for (i = 0; i < priv->max_height * priv->max_width; i++)
			priv->test_vaddr[i] = test_pattern;
	}

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

static int iitdgi_unload(struct drm_device *dev)
{
	struct iitdgi_priv *priv = dev->dev_private;

	iitdgi_writereg(priv, DGI_CTRL, 0);

	if (priv->fbdev) {
		drm_fbdev_cma_fini(priv->fbdev);
		priv->fbdev = NULL;
	}
	drm_kms_helper_poll_fini(dev);
	drm_vblank_cleanup(dev);
	drm_simple_display_pipe_detach_bridge(&priv->pipe);
	drm_mode_config_cleanup(dev);

	dma_release_channel(priv->dma_chan);
	if (test_mode)
		dma_free_coherent(dev->dev,
				priv->max_height * priv->max_width * 4,
				priv->test_vaddr, priv->dma_addr);
	return 0;
}

static void iitdgi_lastclose(struct drm_device *dev)
{
	struct iitdgi_priv *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static const struct file_operations iitdgi_driver_fops = {
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

static struct drm_driver iitdgi_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM |
		DRIVER_ATOMIC | DRIVER_PRIME,
	.lastclose		= iitdgi_lastclose,
	.fops			= &iitdgi_driver_fops,
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

static const struct of_device_id iitdgi_of_match[] = {
	{ .compatible = "iit,dgi-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, iitdgi_of_match);

static int iitdgi_probe(struct platform_device *pdev)
{
	int ret;
	struct drm_device *drm;

	drm = drm_dev_alloc(&iitdgi_drm_driver, &pdev->dev);
	if (!drm)
		return -ENOMEM;

	ret = iitdgi_load(drm);
	if (ret)
		goto err_unref;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	return 0;

err_unload:
	iitdgi_unload(drm);
err_unref:
	drm_dev_unref(drm);

	return ret;
}

static int iitdgi_remove(struct platform_device *pdev)
{
	struct iitdgi_priv *priv = platform_get_drvdata(pdev);
	struct drm_device *drm = priv->drm_dev;

	drm_dev_unregister(drm);
	iitdgi_unload(drm);
	drm_dev_unref(drm);

	return 0;
}

static struct platform_driver iitdgi_platform_driver = {
	.driver = {
		.name = "iitdgi-drm",
		.owner = THIS_MODULE,
		.of_match_table = iitdgi_of_match,
	},
	.probe = iitdgi_probe,
	.remove = iitdgi_remove,
};
module_platform_driver(iitdgi_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andrea Merello <andrea.merello@gmail.com>");
MODULE_DESCRIPTION("IIT DGI DRM driver");
module_param(test_mode, uint, 0);
MODULE_PARM_DESC(test_mode, "enable injection of test pattern");
module_param(test_pattern, uint, 0);
MODULE_PARM_DESC(test_pattern, "spcify which test pattern to use for test. 0 for DMA test");
