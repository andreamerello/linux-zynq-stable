/*
 * Taken by Andrea Merello from a patch floating around, which is:
 *
 * Copyright (C) 2014 Atmel
 *		      Bo Shen <voice.shen at atmel.com>
 *
 * Authors:	      Bo Shen <voice.shen at atmel.com>
 *		      Boris Brezillon <boris.brezillon at free-electrons.com>
 *		      Wu, Songjun <Songjun.Wu at atmel.com>
 *
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Few modification by Andrea Merello <andrea.merello@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#define SIL902X_TPI_VIDEO_DATA				0x0

#define SIL902X_TPI_PIXEL_REPETITION			0x8
#define SIL902X_TPI_AVI_PIXEL_REP_BUS_24BIT     	BIT(5)
#define SIL902X_TPI_AVI_PIXEL_REP_RISING_EDGE   	BIT(4)
#define SIL902X_TPI_AVI_PIXEL_REP_4X			3
#define SIL902X_TPI_AVI_PIXEL_REP_2X			1
#define SIL902X_TPI_AVI_PIXEL_REP_NONE			0
#define SIL902X_TPI_CLK_RATIO_HALF			(0 << 6)
#define SIL902X_TPI_CLK_RATIO_1X			(1 << 6)
#define SIL902X_TPI_CLK_RATIO_2X			(2 << 6)
#define SIL902X_TPI_CLK_RATIO_4X			(3 << 6)

#define SIL902X_TPI_AVI_IN_FORMAT			0x9
#define SIL902X_TPI_AVI_INPUT_BITMODE_12BIT		BIT(7)
#define SIL902X_TPI_AVI_INPUT_DITHER			BIT(6)
#define SIL902X_TPI_AVI_INPUT_RANGE_LIMITED		(2 << 2)
#define SIL902X_TPI_AVI_INPUT_RANGE_FULL		(1 << 2)
#define SIL902X_TPI_AVI_INPUT_RANGE_AUTO		(0 << 2)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_BLACK		(3 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_YUV422		(2 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_YUV444		(1 << 0)
#define SIL902X_TPI_AVI_INPUT_COLORSPACE_RGB		(0 << 0)

#define SIL902X_TPI_AVI_OUTPUT_FORMAT			0xA
#define SIL902X_TPI_AVI_OUTPUT_CONV_BT709		(1 << 4)
#define SIL902X_TPI_AVI_OUTPUT_CONV_BT601		(0 << 4)
#define SIL902X_TPI_AVI_OUTPUT_RANGE_LIMITED		(2 << 2)
#define SIL902X_TPI_AVI_OUTPUT_RANGE_FULL		(1 << 2)
#define SIL902X_TPI_AVI_OUTPUT_RANGE_AUTO		(0 << 2)
#define SIL902X_TPI_AVI_OUTPUT_COLORSPACE_RGBDVI	(3 << 0)
#define SIL902X_TPI_AVI_OUTPUT_COLORSPACE_YUV422	(2 << 0)
#define SIL902X_TPI_AVI_OUTPUT_COLORSPACE_YUV444	(1 << 0)
#define SIL902X_TPI_AVI_OUTPUT_COLORSPACE_RGBHDMI	(0 << 0)


#define SIL902X_TPI_AVI_INFOFRAME			0x0c

#define SIL902X_SYS_CTRL_DATA				0x1a
#define SIL902X_SYS_CTRL_PWR_DWN			BIT(4)
#define SIL902X_SYS_CTRL_AV_MUTE			BIT(3)
#define SIL902X_SYS_CTRL_DDC_BUS_REQ			BIT(2)
#define SIL902X_SYS_CTRL_DDC_BUS_GRTD			BIT(1)
#define SIL902X_SYS_CTRL_OUTPUT_MODE			BIT(0)
#define SIL902X_SYS_CTRL_OUTPUT_HDMI			1
#define SIL902X_SYS_CTRL_OUTPUT_DVI			0

#define SIL902X_REG_CHIPID(n)			(0x1b + (n))

#define SIL902X_PWR_STATE_CTRL			0x1e
#define SIL902X_AVI_POWER_STATE_MSK		GENMASK(1, 0)
#define SIL902X_AVI_POWER_STATE_D(l)		((l) & SIL902X_AVI_POWER_STATE_MSK)

#define SI902X_INT_ENABLE			0x3c
#define SI902X_INT_STATUS			0x3d
#define SI902X_HOTPLUG_EVENT			BIT(0)
#define SI902X_PLUGGED_STATUS			BIT(2)

#define SIL902X_REG_TPI_RQB			0xc7

struct sii902x_drvdata_t {
	int max_pixclk;
	int max_spdif_freq;
	char *model;
};

enum {
	sii9022a,
};

static struct sii902x_drvdata_t sii902x_drvdata[] = {
	[sii9022a] = {
		.max_pixclk = 165000,
		.max_spdif_freq = 192000,
		.model = "sii9022a",
	}
};

struct sii902x {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct gpio_desc *reset_gpio;
	struct work_struct hotplug_work;
	struct mutex ddc_mutex;
	struct sii902x_drvdata_t *drvdata;
	bool enabled;
};

static inline struct sii902x *bridge_to_sii902x(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sii902x, bridge);
}

static inline struct sii902x *connector_to_sii902x(struct drm_connector *con)
{
	return container_of(con, struct sii902x, connector);
}

static void sii902x_reset(struct sii902x *sii902x)
{
	if (!sii902x->reset_gpio)
		return;

	gpiod_set_value(sii902x->reset_gpio, 1);

	msleep(100);

	gpiod_set_value(sii902x->reset_gpio, 0);
}

static void sii902x_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
sii902x_connector_detect(struct drm_connector *connector, bool force)
{
	struct sii902x *sii902x = connector_to_sii902x(connector);
	unsigned int status;

	mutex_lock(&sii902x->ddc_mutex);
	regmap_read(sii902x->regmap, SI902X_INT_STATUS, &status);
	mutex_unlock(&sii902x->ddc_mutex);

	return (status & SI902X_PLUGGED_STATUS) ?
	       connector_status_connected : connector_status_disconnected;
}

static const struct drm_connector_funcs sii902x_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = sii902x_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = sii902x_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

#define DDC_SEGMENT_ADDR 0x30
static int sii902x_do_probe_ddc_edid(void *data, u8 *buf, unsigned int block, size_t len)
{
        struct i2c_adapter *adapter = data;
        unsigned char start = block * EDID_LENGTH;
        unsigned char segment = block >> 1;
        unsigned char xfers = segment ? 3 : 2;
        int ret, retries = 5;

        /*
         * The core I2C driver will automatically retry the transfer if the
         * adapter reports EAGAIN. However, we find that bit-banging transfers
         * are susceptible to errors under a heavily loaded machine and
         * generate spurious NAKs and timeouts. Retrying the transfer
         * of the individual block a few times seems to overcome this.
         */
        while(1) {
                struct i2c_msg msgs[] = {
                        {
                                .addr   = DDC_SEGMENT_ADDR,
                                .flags  = 0,
                                .len    = 1,
                                .buf    = &segment,
                        }, {
                                .addr   = DDC_ADDR,
                                .flags  = 0,
                                .len    = 1,
                                .buf    = &start,
                        }, {
                                .addr   = DDC_ADDR,
                                .flags  = I2C_M_RD,
                                .len    = len,
                                .buf    = buf,
                        }
                };

                /*
                 * Avoid sending the segment addr to not upset non-compliant
                 * DDC monitors.
                 */
                ret = i2c_transfer(adapter, &msgs[3 - xfers], xfers);

                if (ret == -ENXIO) {
                        DRM_DEBUG_KMS("drm: skipping non-existent adapter %s\n",
                                        adapter->name);
                        break;
                }
		if (ret == xfers || --retries == 0)
			break;

		udelay(100);
        }

        return ret == xfers ? 0 : -1;
}
static int sii902x_get_modes(struct drm_connector *connector)
{
	struct sii902x *sii902x = connector_to_sii902x(connector);
	struct regmap *regmap = sii902x->regmap;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	unsigned int status;
	struct edid *edid;
	int num = 0;
	int ret;
	int i;

	mutex_lock(&sii902x->ddc_mutex);

	ret = regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			   SIL902X_SYS_CTRL_PWR_DWN, SIL902X_SYS_CTRL_PWR_DWN);
	if (ret)
		goto exit;

	ret = regmap_update_bits(regmap, SIL902X_SYS_CTRL_DATA,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ);
	if (ret)
		goto exit;

	i = 0;
	do {
		ret = regmap_read(regmap, SIL902X_SYS_CTRL_DATA, &status);
		if (ret)
			goto exit;
		i++;
	} while (!(status & SIL902X_SYS_CTRL_DDC_BUS_GRTD));

	ret = regmap_write(regmap, SIL902X_SYS_CTRL_DATA, status);
	if (ret)
		goto exit;

	/* drm_get_edid() runs two I2C transfers. The sii902x seems
	 * to have problem with the 2nd I2C start. A wait seems needed.
	 * So, we don't perform use drm_get_edid(). We don't perform
	 * the first "probe" transfer, and we use a custom block read
	 * function that, in case the trasfer is split, does introduce
	 * a delay.
	 */
	edid = drm_do_get_edid(connector, sii902x_do_probe_ddc_edid,
			sii902x->i2c->adapter);
	if (!edid)
		goto exit;

	drm_mode_connector_update_edid_property(connector, edid);

	if (edid) {
		num += drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		goto exit;

exit:
	i = 0;
	do {
		ret = regmap_update_bits(regmap, SIL902X_SYS_CTRL_DATA,
				 SIL902X_SYS_CTRL_DDC_BUS_REQ |
				 SIL902X_SYS_CTRL_DDC_BUS_GRTD, 0);
		if (ret)
			goto exit2;

		ret = regmap_read(regmap, SIL902X_SYS_CTRL_DATA, &status);
		if (ret)
			goto exit2;
		i++;
	} while (status & (SIL902X_SYS_CTRL_DDC_BUS_REQ |
			   SIL902X_SYS_CTRL_DDC_BUS_GRTD));

exit2:
	if (sii902x->enabled)
		regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			   SIL902X_SYS_CTRL_PWR_DWN, 0);

	mutex_unlock(&sii902x->ddc_mutex);
	if (ret)
		return ret;
	return num;
}

static enum drm_mode_status sii902x_mode_valid(struct drm_connector *connector,
					       struct drm_display_mode *mode)
{
	struct sii902x *sii902x = connector_to_sii902x(connector);

	if (mode->clock > sii902x->drvdata->max_pixclk)
		return MODE_CLOCK_HIGH;

	/* Bha, I think old VGA or maybe EGA card used something like this.. */
	if (mode->clock < 16000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static struct drm_encoder *sii902x_best_encoder(struct drm_connector *connector)
{
	struct sii902x *sii902x = connector_to_sii902x(connector);

	return sii902x->bridge.encoder;

}

static const struct drm_connector_helper_funcs sii902x_connector_helper_funcs = {
	.get_modes = sii902x_get_modes,
	.mode_valid = sii902x_mode_valid,
	.best_encoder = sii902x_best_encoder,
};

static void sii902x_bridge_disable(struct drm_bridge *bridge)
{
	struct sii902x *sii902x = bridge_to_sii902x(bridge);

	mutex_lock(&sii902x->ddc_mutex);
	sii902x->enabled = false;
	regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			   SIL902X_SYS_CTRL_PWR_DWN,
			   SIL902X_SYS_CTRL_PWR_DWN);
	mutex_unlock(&sii902x->ddc_mutex);
}

static void sii902x_bridge_enable(struct drm_bridge *bridge)
{
	struct sii902x *sii902x = bridge_to_sii902x(bridge);

	mutex_lock(&sii902x->ddc_mutex);
	sii902x->enabled = true;
	regmap_update_bits(sii902x->regmap, SIL902X_PWR_STATE_CTRL,
			   SIL902X_AVI_POWER_STATE_MSK,
			   SIL902X_AVI_POWER_STATE_D(0));

	regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			   SIL902X_SYS_CTRL_PWR_DWN, 0);
	mutex_unlock(&sii902x->ddc_mutex);
}

static void sii902x_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj)
{
	u8 buf[HDMI_INFOFRAME_HEADER_SIZE + HDMI_AVI_INFOFRAME_SIZE];
	struct sii902x *sii902x = bridge_to_sii902x(bridge);
	struct regmap *regmap = sii902x->regmap;
	bool hdmi = false;
	bool hd_colorspace;
	struct edid *edid;
	u8 input_type;
	struct hdmi_avi_infoframe frame;
	int ret;

	mutex_lock(&sii902x->ddc_mutex);

	regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			SIL902X_SYS_CTRL_PWR_DWN, SIL902X_SYS_CTRL_PWR_DWN);

	edid = drm_get_edid(&sii902x->connector, sii902x->i2c->adapter);
	if (edid) {
		input_type = edid->input & 0x7;
		hdmi = input_type == DRM_EDID_DIGITAL_TYPE_HDMI_A ||
			input_type == DRM_EDID_DIGITAL_TYPE_HDMI_B;
	}

	hd_colorspace = adj->hdisplay >= 720;
	buf[0] = adj->clock;
	buf[1] = adj->clock >> 8;
	buf[2] = adj->vrefresh;
	buf[3] = 0x00;
	buf[4] = adj->hdisplay;
	buf[5] = adj->hdisplay >> 8;
	buf[6] = adj->vdisplay;
	buf[7] = adj->vdisplay >> 8;
	buf[8] = SIL902X_TPI_CLK_RATIO_1X | SIL902X_TPI_AVI_PIXEL_REP_NONE |
		 SIL902X_TPI_AVI_PIXEL_REP_BUS_24BIT;
	buf[9] = SIL902X_TPI_AVI_INPUT_RANGE_AUTO |
		 SIL902X_TPI_AVI_INPUT_COLORSPACE_RGB;
	buf[10] = (hdmi ? SIL902X_TPI_AVI_OUTPUT_COLORSPACE_RGBHDMI :
		SIL902X_TPI_AVI_OUTPUT_COLORSPACE_RGBDVI) |
		(hd_colorspace ? SIL902X_TPI_AVI_OUTPUT_CONV_BT709 :
		SIL902X_TPI_AVI_OUTPUT_CONV_BT601);

	ret = regmap_bulk_write(regmap, SIL902X_TPI_VIDEO_DATA, buf, 11);
	if (ret)
		goto exit;

	if (hdmi) {
		regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
				SIL902X_SYS_CTRL_OUTPUT_MODE,
				SIL902X_SYS_CTRL_OUTPUT_HDMI );

		ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, adj);
		if (ret < 0) {
			DRM_ERROR("couldn't fill AVI infoframe\n");
			goto exit;
		}

		ret = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
		if (ret < 0) {
			DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
			goto exit;
		}

		/* Do not send the infoframe header, but keep the CRC field. */
		regmap_bulk_write(regmap, SIL902X_TPI_AVI_INFOFRAME,
				buf + HDMI_INFOFRAME_HEADER_SIZE - 1,
				HDMI_AVI_INFOFRAME_SIZE + 1);
	} else {
		regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
				SIL902X_SYS_CTRL_OUTPUT_MODE,
				SIL902X_SYS_CTRL_OUTPUT_DVI );
	}

	if (sii902x->enabled)
		regmap_update_bits(sii902x->regmap, SIL902X_SYS_CTRL_DATA,
			SIL902X_SYS_CTRL_PWR_DWN, 0);

exit:
	mutex_unlock(&sii902x->ddc_mutex);
}

static int sii902x_bridge_attach(struct drm_bridge *bridge)
{
	struct sii902x *sii902x = bridge_to_sii902x(bridge);
	struct drm_device *drm = bridge->dev;
	int ret;

	drm_connector_helper_add(&sii902x->connector,
				 &sii902x_connector_helper_funcs);

	if (!drm_core_check_feature(drm, DRIVER_ATOMIC)) {
		dev_err(&sii902x->i2c->dev,
			"sii902x driver is only compatible with DRM device supporting atomic updates");
		return -ENOTSUPP;
	}
	ret = drm_connector_init(drm, &sii902x->connector,
				 &sii902x_connector_funcs,
				DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	if (sii902x->i2c->irq > 0)
		sii902x->connector.polled = DRM_CONNECTOR_POLL_HPD;
	else
		sii902x->connector.polled = DRM_CONNECTOR_POLL_CONNECT;

	drm_mode_connector_attach_encoder(&sii902x->connector, bridge->encoder);

	return 0;
}

static void sii902x_bridge_nop(struct drm_bridge *bridge)
{
}

static const struct drm_bridge_funcs sii902x_bridge_funcs = {
	.attach = sii902x_bridge_attach,
	.mode_set = sii902x_bridge_mode_set,
	.disable = sii902x_bridge_disable,
	.post_disable = sii902x_bridge_nop,
	.pre_enable = sii902x_bridge_nop,
	.enable = sii902x_bridge_enable,
};

static const struct regmap_range sii902x_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xff },
};

static const struct regmap_access_table sii902x_volatile_table = {
	.yes_ranges = sii902x_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(sii902x_volatile_ranges),
};

static const struct regmap_config sii902x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &sii902x_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t sii902x_interrupt(int irq, void *data)
{
	struct sii902x *sii902x = data;
	unsigned int status = 0;

	regmap_read(sii902x->regmap, SI902X_INT_STATUS, &status);
	regmap_write(sii902x->regmap, SI902X_INT_STATUS, status);

	if ((status & SI902X_HOTPLUG_EVENT) && sii902x->bridge.dev)
		drm_helper_hpd_irq_event(sii902x->bridge.dev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id sii902x_dt_ids[] = {
	{ .compatible = "sil,sii9022a", .data = &sii902x_drvdata[sii9022a] },
	{ }
};
MODULE_DEVICE_TABLE(of, sii902x_dt_ids);
#endif

static const struct i2c_device_id sii902x_i2c_ids[] = {
	{ "sii9022a", sii9022a },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sii902x_i2c_ids);


static int sii902x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	unsigned int status = 0;
	struct sii902x *sii902x;
	u8 chipid[4];
	int ret;

	sii902x = devm_kzalloc(dev, sizeof(*sii902x), GFP_KERNEL);
	if (!sii902x)
		return -ENOMEM;

	mutex_init(&sii902x->ddc_mutex);

	if (client->dev.of_node) {
		sii902x->drvdata =
			(struct sii902x_drvdata_t*)of_match_device(sii902x_dt_ids,
							&client->dev)->data;
	} else {
		sii902x->drvdata = &sii902x_drvdata[id->driver_data];
	}

	sii902x->i2c = client;
	sii902x->regmap = devm_regmap_init_i2c(client, &sii902x_regmap_config);
	if (IS_ERR(sii902x->regmap))
		return PTR_ERR(sii902x->regmap);

	sii902x->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_LOW);
	if (IS_ERR(sii902x->reset_gpio))
		dev_warn(dev, "Failed to retrieve/request reset gpio: %ld\n",
			 PTR_ERR(sii902x->reset_gpio));

	sii902x_reset(sii902x);

	ret = regmap_write(sii902x->regmap, SIL902X_REG_TPI_RQB, 0x0);
	if (ret)
		return ret;

	ret = regmap_bulk_read(sii902x->regmap, SIL902X_REG_CHIPID(0),
			       &chipid, 4);
	if (ret) {
		dev_err(dev, "regmap_read failed %d\n", ret);
		return ret;
	}

	if (chipid[0] != 0xb0) {
		dev_err(dev, "Invalid chipid: %02x (expecting 0xb0)\n",
			chipid[0]);
		return -EINVAL;
	}

	/* Clear all pending interrupts */
	regmap_read(sii902x->regmap, SI902X_INT_STATUS, &status);
	regmap_write(sii902x->regmap, SI902X_INT_STATUS, status);

	if (client->irq > 0) {
		regmap_write(sii902x->regmap, SI902X_INT_ENABLE,
			     SI902X_HOTPLUG_EVENT);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sii902x_interrupt,
						IRQF_ONESHOT, dev_name(dev),
						sii902x);
		if (ret)
			return ret;
	}

	sii902x->bridge.funcs = &sii902x_bridge_funcs;
	sii902x->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&sii902x->bridge);
	if (ret) {
		dev_err(dev, "Failed to add drm_bridge\n");
		return ret;
	}

	i2c_set_clientdata(client, sii902x);
	dev_info(dev, "Probed bridge encoder model: %s", sii902x->drvdata->model);
	return 0;
}

static int sii902x_remove(struct i2c_client *client)

{
	struct sii902x *sii902x = i2c_get_clientdata(client);

	drm_bridge_remove(&sii902x->bridge);

	return 0;
}


static struct i2c_driver sii902x_driver = {
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.driver = {
		.name = "sii902x",
		.of_match_table = of_match_ptr(sii902x_dt_ids),
	},
	.id_table = sii902x_i2c_ids,
};
module_i2c_driver(sii902x_driver);

MODULE_AUTHOR("Boris Brezillon <boris.brezillon at free-electrons.com>");
MODULE_DESCRIPTION("SIL902x RGB -> HDMI bridges");
MODULE_LICENSE("GPL");
