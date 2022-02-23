/*
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb-deferred-drv.h>

static const struct of_device_id fb_dummy_match[] = {
	{ .compatible = "fb,deferred-probe-trigger",},
	{}
};

static int fb_dummy_probe(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver fb_dummy_driver = {
	.probe = fb_dummy_probe,
	.driver = {
		.name = "fb-trigger-deferred-drv",
		.owner = THIS_MODULE,
		.of_match_table = fb_dummy_match,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};

static DEFINE_MUTEX(fb_dummy_driver_mutext);

void fb_trigger_deferred_drivers(void)
{
	/*
	 * Due to race condition, audio sound card was not created
	 * if codec is up later then the last successful driver probe.
	 * To fix this, probe a dummy driver to trigger all pending
	 * deferred driver one more time when AUDIO is up.
	 */
	mutex_lock(&fb_dummy_driver_mutext);
	platform_driver_register(&fb_dummy_driver);
	platform_driver_unregister(&fb_dummy_driver);
	mutex_unlock(&fb_dummy_driver_mutext);
}
EXPORT_SYMBOL(fb_trigger_deferred_drivers);

MODULE_AUTHOR("Dinko Mironov <dmironov@mm-sol.com>");
MODULE_DESCRIPTION("Facebook trigger deferred drivers");
MODULE_LICENSE("GPL");
