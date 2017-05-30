/*
 * Watchdog driver for EDL WDT IP
 *
 * Copyright (C) IIT EDL 2017
 */

#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

/* minimum and maximum watchdog trigger timeout, in seconds */
#define MIN_WDT_TIMEOUT			1

/* Register definitions */
#define REG_BASE                        0x0
#define REG_CFG                         0x4
#define REG_WD_ALARM_FREQ               0x110
#define REG_WD_ALARM_VOLUME             0x114
#define REG_WD_TIMEOUT                  0x118
#define REG_CFG_KICK_VALUE              (1 << 16)
#define WD_CLOCK_FREQ                   (12000000)
#define WD_KICK_FREQ                    (42000)

struct iit_edlwdt_data {
	struct watchdog_device	wdd;
        struct regmap           *wdt_regs;
};

#define WDT_HEARTBEAT 10
static int heartbeat = WDT_HEARTBEAT;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
	"Watchdog heartbeats in seconds. (default = "
	__MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int iit_edlwdt_start(struct watchdog_device *wdd)
{
        dev_dbg(wdd->parent, "(start): Watchdog always active\n");
	return 0;
}

static int iit_edlwdt_ping(struct watchdog_device *wdd)
{
	struct iit_edlwdt_data *wdt = watchdog_get_drvdata(wdd);
        unsigned int readValue = 0;
	int ret;

	regmap_read(wdt->wdt_regs, REG_WD_TIMEOUT, &readValue);
	dev_dbg(wdd->parent, "(ping): timeout is %08x", readValue);

	regmap_read(wdt->wdt_regs, REG_CFG, &readValue);
	ret = regmap_write_bits(wdt->wdt_regs, REG_CFG,
				  REG_CFG_KICK_VALUE, REG_CFG_KICK_VALUE);

	return ret;
}

static int iit_edlwdt_set_timeout(struct watchdog_device *wdd,
				 unsigned int timeout)
{
	unsigned int regValue;
	struct iit_edlwdt_data *wdt = watchdog_get_drvdata(wdd);
        unsigned int readValue = 0;

	wdd->timeout = timeout;

	timeout = min(wdd->max_hw_heartbeat_ms / 1000, timeout);
	regValue = timeout * WD_CLOCK_FREQ;

        regmap_write(wdt->wdt_regs, REG_WD_TIMEOUT, regValue);
        regmap_read(wdt->wdt_regs, REG_WD_TIMEOUT, &readValue);

	dev_dbg(wdd->parent, "(set_timeout): write %08x in register %08x --> %08x",
            regValue, REG_WD_TIMEOUT, readValue);

	return 0;
}

static const struct watchdog_info iit_edlwdt_info = {
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_ALARMONLY |
			  WDIOF_MAGICCLOSE |
			  WDIOF_KEEPALIVEPING,
	.firmware_version = 0,
	.identity	= "IIT EDL watchdog IP",
};

static const struct watchdog_ops iit_edlwdt_ops = {
	.owner = THIS_MODULE,
	.start = iit_edlwdt_start,
	.ping = iit_edlwdt_ping,
	.set_timeout = iit_edlwdt_set_timeout,
};

static int iit_edlwdt_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
        struct watchdog_device *wdd;
        struct iit_edlwdt_data *data;
        int ret;

        /* Allocate the driver's internal data structure. */
        data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
        if(!data)
          return -ENOMEM;

        data->wdt_regs = syscon_regmap_lookup_by_phandle(np, "iit,syscon");
	if (IS_ERR(data->wdt_regs)) {
		dev_err(&pdev->dev, "FAILED syscon_regmap_lookup_by_phandle\n");
		return PTR_ERR(data->wdt_regs);
        }

        /* Initialize watchdog device */
        wdd = &data->wdd;
        wdd->timeout = heartbeat;
        wdd->info = &iit_edlwdt_info;
        wdd->ops = &iit_edlwdt_ops;
        wdd->min_timeout = MIN_WDT_TIMEOUT;
        wdd->max_hw_heartbeat_ms = WD_KICK_FREQ;
        wdd->parent = &pdev->dev;

        /* Set the WDOG_HW_RUNNING bit to one so that the   */
        /* system take care of watchdog kicking by himself. */
        set_bit(WDOG_HW_RUNNING, &wdd->status);

        watchdog_set_drvdata(wdd, data);
        watchdog_set_nowayout(wdd, nowayout);

        /* Driver registrayion. */
        ret = watchdog_register_device(wdd);
        if (ret) {
		dev_err(&pdev->dev, "failed to register IIT EDL watchdog\n");
		return ret;
        }

        platform_set_drvdata(pdev, data);

        dev_info(&pdev->dev, "initialized (heartbeat = %d sec, nowayout = %d)\n",
            heartbeat, nowayout);

        /* Set the configurable timeout. */
        iit_edlwdt_set_timeout(wdd, heartbeat);

        dev_dbg(&pdev->dev, "iit_edlwdt_probe DONE\n");
	return 0;
}

static int iit_edlwdt_remove(struct platform_device *pdev)
{
	struct iit_edlwdt_data *data = platform_get_drvdata(pdev);

        watchdog_unregister_device(&data->wdd);
	return 0;
}


static const struct of_device_id iit_edlwdt_match[] = {
	{ .compatible = "iit,edlwdt"},
	{ }
};
MODULE_DEVICE_TABLE(of, iit_edlwdt_match);

static struct platform_driver iit_edlwdt_driver = {
	.probe  = iit_edlwdt_probe,
	.remove = iit_edlwdt_remove,
	.driver = {
		.name           = "iit-edlwdt",
		.of_match_table = iit_edlwdt_match,
	},
};
module_platform_driver(iit_edlwdt_driver);

MODULE_AUTHOR("Mirco Di Salvo <mirco.disalvo@iit.it>");
MODULE_AUTHOR("Andrea Merello <andrea.merello@iit.it>");
MODULE_DESCRIPTION("IIT EDL watchdog timer IP driver");
MODULE_LICENSE("GPL v2");
