/*
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "../codecs/adau17x1.h"

static const struct snd_soc_dapm_widget gla_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Out", NULL),
};

static const struct snd_soc_dapm_route gla_routes[] = {
	{ "Headphone Out", NULL, "LHPOUT" },
	{ "Headphone Out", NULL, "RHPOUT" },
};

static int gla_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{

// 	struct snd_soc_pcm_runtime *rtd = substream->private_data;
// 	struct snd_soc_dai *codec_dai = rtd->codec_dai;
// 	unsigned int pll_rate;
// 	int ret;
// 
// 	switch (params_rate(params)) {
// 	case 48000:
// 	case 8000:
// 	case 12000:
// 	case 16000:
// 	case 24000:
// 	case 32000:
// 	case 96000:
// 		pll_rate = 48000 * 1024;
// 		break;
// 	case 44100:
// 	case 7350:
// 	case 11025:
// 	case 14700:
// 	case 22050:
// 	case 29400:
// 	case 88200:
// 		pll_rate = 44100 * 1024;
// 		break;
// 	default:
// 		return -EINVAL;
// 	}
// 
// 	ret = snd_soc_dai_set_pll(codec_dai, ADAU17X1_PLL,
// 			ADAU17X1_PLL_SRC_MCLK, 12288000, pll_rate);
// 	if (ret)
// 		return ret;
// 
// 	ret = snd_soc_dai_set_sysclk(codec_dai, ADAU17X1_CLK_SRC_PLL, pll_rate,
// 			SND_SOC_CLOCK_IN);
// 
	return 0;
}

static struct snd_soc_ops gla_ops = {
	.hw_params = gla_hw_params,
};

static struct snd_soc_dai_link gla_dai_link = {
	.name = "gla",
	.codec_name = "snd-soc-dummy",
	.codec_dai_name = "snd-soc-dummy-dai",
	.stream_name = "gla",
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	.ops = &gla_ops,
};

static struct snd_soc_card gla_card = {
	.name = "GLASSENSE",
	.owner = THIS_MODULE,
	.dai_link = &gla_dai_link,
	.num_links = 1,
//	.dapm_widgets = gla_widgets,
//	.num_dapm_widgets = ARRAY_SIZE(gla_widgets),
//	.dapm_routes = gla_routes,
//	.num_dapm_routes = ARRAY_SIZE(gla_routes),
	.fully_routed = true,
};

static int gla_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &gla_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;
	card->dev = &pdev->dev;

	// gla_dai_link.codec_of_node = of_parse_phandle(of_node, "audio-codec", 0);
	gla_dai_link.codec_of_node = NULL;	
    gla_dai_link.cpu_of_node = of_parse_phandle(of_node, "cpu-dai", 0);
	gla_dai_link.platform_of_node = gla_dai_link.cpu_of_node;

	if (!gla_dai_link.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int gla_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id gla_of_match[] = {
	{ .compatible = "iit,glassense", },
	{},
};
MODULE_DEVICE_TABLE(of, gla_of_match);

static struct platform_driver gla_card_driver = {
	.driver = {
		.name = "zed-gla-snd",
		.owner = THIS_MODULE,
		.of_match_table = gla_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = gla_probe,
	.remove = gla_remove,
};
module_platform_driver(gla_card_driver);

MODULE_DESCRIPTION("ASoC ZED board Glassense driver");
MODULE_AUTHOR("Francesco Diotalevi <francesco.diotalevi@iit.it>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:zed-gla-snd");
