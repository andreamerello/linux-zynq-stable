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
#include "ocdrm_crtc.h"
#include "ocdrm_encoder.h"

#define DRIVER_NAME	"ocdrm"
#define DRIVER_DESC	"OpenCores DRM"
#define DRIVER_DATE	"20160527"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static void ocdrm_output_poll_changed(struct drm_device *drm)
{
	struct ocdrm_priv *priv = drm->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static int ocdrm_atomic_commit(struct drm_device *dev,
				    struct drm_atomic_state *state, bool async)
{
	return drm_atomic_helper_commit(dev, state, false);
}

static struct drm_mode_config_funcs ocdrm_mode_config_funcs = {
	.output_poll_changed = ocdrm_output_poll_changed,
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = ocdrm_atomic_commit,
};

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

static void ocdrm_mode_config_init(struct ocdrm_priv *priv)
{
	struct drm_device *dev = priv->drm_dev;
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width =  1500;
	dev->mode_config.max_height = 1500;

	dev->mode_config.funcs = &ocdrm_mode_config_funcs;
}

static void ocdrm_detect_endian(struct ocdrm_priv *priv)
{
	priv->little_endian = false;
	ocdrm_writereg(priv, OCFB_VBARA, 0xfffffff0);
        if (ocdrm_readreg(priv, OCFB_VBARA) != 0xfffffff0)
		priv->little_endian = true;

	ocdrm_writereg(priv, OCFB_VBARA, 0x0);
}

static int ocdrm_load(struct drm_device *dev)
{
	struct ocdrm_priv *priv;
	int ret;
	struct resource *res;
	struct platform_device *pdev = to_platform_device(dev->dev);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev->dev_private = priv;
	priv->drm_dev = dev;

	priv->pixel_clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->pixel_clock)) {
		return -EPROBE_DEFER;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)))
		return -ENOMEM;

	ocdrm_detect_endian(priv);

	drm_mode_config_init(dev);
        ocdrm_mode_config_init(priv);

	ret = ocdrm_crtc_create(priv);
	if (ret)
		goto err_crtc;

	ret = ocdrm_encoder_create(priv);
	if (ret)
		goto err_crtc;

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
	.compat_ioctl = drm_compat_ioctl,
#endif
	.llseek = no_llseek,
	.release	= drm_release,
};

static struct drm_driver ocdrm_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM |
		DRIVER_ATOMIC | DRIVER_PRIME,
	.lastclose		    = ocdrm_lastclose,
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
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap 	= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap 	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap 	= drm_gem_cma_prime_mmap,

};

static const struct of_device_id ocdrm_of_match[] = {
	{ .compatible = "opencores,ocfb-drm", },
	{},
};
MODULE_DEVICE_TABLE(of, ocdrm_of_match);

void ocdrm_connector_unregister_all(struct drm_device *dev)
{
	struct drm_connector *connector;

	/* FIXME: taking the mode config mutex ends up in a clash with sysfs */
	list_for_each_entry(connector, &dev->mode_config.connector_list, head)
		drm_connector_unregister(connector);
}

int ocdrm_connector_register_all(struct drm_device *dev)
{
	struct drm_connector *connector;
	int ret;

	mutex_lock(&dev->mode_config.mutex);

	drm_for_each_connector(connector, dev) {
		ret = drm_connector_register(connector);
		if (ret)
			goto err;
	}

	mutex_unlock(&dev->mode_config.mutex);

	return 0;

err:
	mutex_unlock(&dev->mode_config.mutex);
	ocdrm_connector_unregister_all(dev);
	return ret;
}

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
	if(ret)
		goto err_unload;

	ret = ocdrm_connector_register_all(drm);
	if (ret)
		goto err_unregister;
	return 0;

err_unregister:
	drm_dev_unregister(drm);
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
#warning v4.7
	ocdrm_connector_unregister_all(priv->drm_dev);
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
