/*
 * File:         sound/soc/adi/adi-ssm2518.c
 * Author:       Francesco Diotalevi <francesco.diotalevi@iit.it>
 *
 * Created:      Tue March 06 2016
 * Description:  board driver for SSM2518 sound chip
 *
 * Modified:
 *               Copyright 2016 Istituto Italiano di Tecnologia
 *
 * Bugs:         
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "../codecs/ssm2518.h"

static int ssm2518_init(struct snd_soc_pcm_runtime *rtd)
{
    return snd_soc_codec_set_sysclk(rtd->codec, SSM2518_SYSCLK,
                SSM2518_SYSCLK_SRC_MCLK, 12288000, SND_SOC_CLOCK_IN);
}

static const struct snd_soc_dapm_widget ssm2518_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker Out", NULL),
};

static const struct snd_soc_dapm_route ssm2518_routes[] = {
	{ "Speaker Out", NULL, "OUTL" },
	{ "Speaker Out", NULL, "OUTR" },
};

static struct snd_soc_dai_link ssm2518_dai_link = {
	.name = "ssm2518",
	.stream_name = "ssm2518",
	.codec_dai_name = "ssm2518-hifi",
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	.init = ssm2518_init,
};

static struct snd_soc_card ssm2518_module = {
	.name = "SSM2518 Amplifier",
	.owner = THIS_MODULE,
	.dai_link = &ssm2518_dai_link,
	.num_links = 1,
	.dapm_widgets = ssm2518_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ssm2518_widgets),
	.dapm_routes = ssm2518_routes,
	.num_dapm_routes = ARRAY_SIZE(ssm2518_routes),
	.fully_routed = true,
};

static int ssm2518_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &ssm2518_module;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	ssm2518_dai_link.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	ssm2518_dai_link.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	ssm2518_dai_link.platform_of_node = ssm2518_dai_link.cpu_of_node;

	if (!ssm2518_dai_link.codec_of_node ||
		!ssm2518_dai_link.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int ssm2518_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id ssm2518_of_match[] = {
	{ .compatible = "digilent,ssm2518-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, ssm2518_of_match);

static struct platform_driver ssm2518_card_driver = {
	.driver = {
		.name = "ssm2518-snd",
		.owner = THIS_MODULE,
		.of_match_table = ssm2518_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = ssm2518_probe,
	.remove = ssm2518_remove,
};
module_platform_driver(ssm2518_card_driver);

MODULE_DESCRIPTION("ASoC ssm2518 driver");
MODULE_AUTHOR("Francesco Diotalevi (iit)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ssm2518-snd");
