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

#ifndef _LINUX_FB_PRIVACY_H
#define _LINUX_FB_PRIVACY_H

#ifdef CONFIG_FB_PRIVACY

/* Privacy notifiers */
#define FBP_PRECHANGE		(0)
#define FBP_POSTCHANGE		(1)
#define FBP_CHANGE_FAIL		(2)

struct fbp_notifier_info {
	uint8_t new_state;
};

extern int fbp_register_notifier(struct notifier_block *nb);
extern int fbp_unregister_notifier(struct notifier_block *nb);
extern bool fbp_enter_privacy(struct fbp_notifier_info *info);
extern bool fbp_exit_privacy(struct fbp_notifier_info *info);
extern int fbp_shutdown_apps(void);
extern void fbp_report_key_event(u32 key_code, u32 event);

#else /* CONFIG_FB_PRIVACY */

static int fbp_register_notifier(struct notifier_block *nb)
{
	return 0;
}

static int fbp_unregister_notifier(struct notifier_block *nb)
{
	return 0;
}

static bool fbp_enter_privacy(struct fbp_notifier_info *info)
{
	return false;
}

static bool fbp_exit_privacy(struct fbp_notifier_info *info)
{
	return false;
}

static int fbp_shutdown_apps(void)
{
	return 0;
}

static void fbp_report_key_event(u32 key_code, u32 event) { }

#endif /* CONFIG_FB_PRIVACY */

#endif
