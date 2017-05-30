/*
 * PWM driver for EDL PWMWDT IP
 *
 * Based on pwm-ab500.c, pwm-rockchip.c, pwm_atmel.c, pwm-clps711x.c
 *
 * Copyright (C) 2017 Istituto Italiano di Tecnologia - EDL
 * Electronic Design Laboratory
 *
 * Written by Andrea Merello <andrea.merello@gmail.com>
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

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define PWMIIT_CH_SIZE 		(3 * 4)
#define PWMIIT_CH_BASE 		0x10
#define PWMIIT_CH_REG_FREQ 	0x0
#define PWMIIT_CH_REG_DC 	0x4
#define PWMIIT_CH_REG_LEN 	0x8
#define PWMIIT_EN_REG		0x4

struct pwmiit_priv {
	struct pwm_chip chip;
	struct regmap *regmap;
	struct clk *clk;
};

static int iit_edlpwm_ch_write(struct pwmiit_priv *priv, int ch,
			       int reg, unsigned int val)
{
	int offs = PWMIIT_CH_BASE + ch * PWMIIT_CH_SIZE + reg;

	dev_dbg(priv->chip.dev, "WR ch reg. offs 0x%x, val 0x%x\n", offs, val);
	return regmap_write(priv->regmap, offs, val);
}

static int iit_edlpwm_ch_read(struct pwmiit_priv *priv, int ch,
			      int reg, unsigned int *val)
{
	int offs = PWMIIT_CH_BASE + ch * PWMIIT_CH_SIZE + reg;

	return regmap_read(priv->regmap, offs, val);
}

static int iit_edlpwm_ch_enable(struct pwmiit_priv *priv, int ch, bool enable)
{
	int ret;
	u32 mask = 1 << ch;
	u32 val = (enable ? 1 : 0) << ch;

	dev_dbg(priv->chip.dev, "PWM ch %d, enable: %d\n", ch, enable);
	if (enable) {
		ret = iit_edlpwm_ch_write(priv, ch,
					  PWMIIT_CH_REG_LEN, 0xffffffff);
		if (ret < 0)
			return ret;
	}

	return regmap_update_bits(priv->regmap, PWMIIT_EN_REG, mask, val);
}

static int iit_edlpwm_ch_setcfg(struct pwmiit_priv *priv, int ch,
				u32 period, u32 duty)
{
	u32 val;
	unsigned long clk_freq;
	int ret;
	u32 step_ns;

	/*
	 * DOC: the value to be written is 1/(256 * freqHz * sysclk_periodS)
	 *
	 * so, reg = (periodS * sysclk_fHz) / 256
	 * NOTE: period from pwm layer is in nanoseconds
	 */
	clk_freq = clk_get_rate(priv->clk);
	step_ns = NSEC_PER_SEC / (clk_freq / 256);
	val = period / step_ns;

	dev_dbg(priv->chip.dev, "WR period: %d\n", val);
	ret = iit_edlpwm_ch_write(priv, ch, PWMIIT_CH_REG_FREQ, val);
	if (ret < 0)
		return ret;

	/*
	 * DOC: the value to be written is 8 bit wide value obtained by the
	 * following equation: (255 * Value[%]) / 100
	 * val = duty * 255 / period;
	 */
	val = duty / (period / 255);
	dev_dbg(priv->chip.dev, "WR duty: %d\n", val);
	return iit_edlpwm_ch_write(priv, ch, PWMIIT_CH_REG_DC, val);
}

static int iit_edlpwm_ch_getcfg(struct pwmiit_priv *priv, int ch,
				unsigned int *period, unsigned int *duty)
{
	u32 val;
	unsigned long clk_freq;
	int ret;
	u32 step_ns;

	ret = iit_edlpwm_ch_read(priv, ch, PWMIIT_CH_REG_FREQ, &val);
	if (ret < 0)
		return(ret);

	clk_freq = clk_get_rate(priv->clk);
	step_ns = NSEC_PER_SEC / (clk_freq / 256);
	*period = val * step_ns;

	ret = iit_edlpwm_ch_read(priv, ch, PWMIIT_CH_REG_DC, &val);
	if (ret < 0)
		return(ret);

	*duty = *period / 255 * val;

	return 0;
}

static int iit_edlpwm_clock_autodisable(struct pwmiit_priv *priv)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, PWMIIT_EN_REG, &val);
	if (ret < 0)
		return ret;

	/* mask with EN bits of all present PWM chs */
	val &= 0xff >> (8 - priv->chip.npwm);
	if (val == 0)
		clk_disable_unprepare(priv->clk);

	return 0;
}

static int iit_edlpwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      struct pwm_state *state)
{
	int ret;

	struct pwmiit_priv *priv = container_of(chip, struct pwmiit_priv, chip);

	dev_dbg(chip->dev, "PWM applying new cfg\n");
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(chip->dev, "Failed to enable clock\n");
		return ret;
	}

	/* FD syas: pwm must be disabled before writing cfg regs */
	ret = iit_edlpwm_ch_enable(priv, pwm->hwpwm, false);
	if (ret < 0)
		return ret;

	ret = iit_edlpwm_ch_setcfg(priv, pwm->hwpwm,
			     state->period, state->duty_cycle);
	if (ret < 0)
		return ret;

	if (state->enabled) {
		ret = iit_edlpwm_ch_enable(priv, pwm->hwpwm, true);
		if (ret < 0)
			return ret;
	}

	ret = iit_edlpwm_clock_autodisable(priv);
	if (ret)
		return ret;

	dev_dbg(chip->dev, "PWM apply cfg OK\n");
	return 0;
}

void iit_edlpwm_getstate(struct pwm_chip *chip, struct pwm_device *pwm,
			  struct pwm_state *state)
{
	u32 val;
	int ret;
	struct pwmiit_priv *priv = container_of(chip, struct pwmiit_priv, chip);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(chip->dev, "Failed to enable clock\n");
		return;
	}

	iit_edlpwm_ch_read(priv, pwm->hwpwm, PWMIIT_CH_REG_LEN, &val);
	if (val == 0xffffffff) {
		iit_edlpwm_ch_read(priv, pwm->hwpwm, PWMIIT_EN_REG, &val);
		state->enabled = (val >> pwm->hwpwm) & 1;
	} else {
		state->enabled = false;
	}

	iit_edlpwm_ch_getcfg(priv, pwm->hwpwm,
			     &state->period, &state->duty_cycle);

	dev_dbg(priv->chip.dev, "PWM getstate.enable: %d, period: %u, duty: %u\n",
		state->enabled, state->period, state->duty_cycle);

	iit_edlpwm_clock_autodisable(priv);
}

static const struct pwm_ops iitpwm_ops = {
	.apply = iit_edlpwm_apply,
	.get_state = iit_edlpwm_getstate,
	.owner = THIS_MODULE,
};

static struct pwm_device *iit_edlpwm_xlate(struct pwm_chip *chip,
					   const struct of_phandle_args *args)
{
	if (args->args[0] >= chip->npwm)
		return ERR_PTR(-EINVAL);

	return pwm_request_from_chip(chip, args->args[0], NULL);
}

static int iit_edlpwm_probe(struct platform_device *pdev)
{
	struct pwmiit_priv *priv;
	int ret;
	u32 npwm;
	struct device_node *np = pdev->dev.of_node;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->regmap = syscon_regmap_lookup_by_phandle(np, "iit,syscon");
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "can't access 'iit,syscon' regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = of_property_read_u32(np, "num-pwm", &npwm);
	if (ret)  {
		dev_err(&pdev->dev, "missing 'num-pwm' DT property\n");
		return ret;
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get clock");
		return PTR_ERR(priv->clk);
	}

	platform_set_drvdata(pdev, priv);

	priv->chip.dev = &pdev->dev;
	priv->chip.ops = &iitpwm_ops;
	priv->chip.npwm = npwm;
	priv->chip.base = -1;
	priv->chip.of_pwm_n_cells = 1;
	priv->chip.of_xlate = iit_edlpwm_xlate;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
	}
	dev_dbg(&pdev->dev, "PWM successfully probed\n");

	return ret;
}

static int iit_edlpwm_remove(struct platform_device *pdev)
{
	int ret;
	struct pwmiit_priv *priv = platform_get_drvdata(pdev);

	ret = iit_edlpwm_clock_autodisable(priv);
	if (ret)
		return ret;
	return pwmchip_remove(&priv->chip);
}

static const struct of_device_id iit_edlpwm_match[] = {
	{ .compatible = "iit,edlpwm"},
	{ }
};
MODULE_DEVICE_TABLE(of, iit_edlpwm_match);

static struct platform_driver iit_edlpwm_driver = {
	.probe  = iit_edlpwm_probe,
	.remove = iit_edlpwm_remove,
	.driver = {
		.name           = "iit-edlpwm",
		.of_match_table = iit_edlpwm_match,
	},
};
module_platform_driver(iit_edlpwm_driver);

MODULE_AUTHOR("Andrea Merello <andrea.merello@iit.it>");
MODULE_DESCRIPTION("IIT EDL Pulse Width Modulation IP driver");
MODULE_LICENSE("GPL v2");
