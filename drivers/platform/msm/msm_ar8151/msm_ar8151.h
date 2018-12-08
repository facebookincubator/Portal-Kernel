 /* Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#ifndef __MSM_AR8151_H__
#define __MSM_AR8151_H__

struct device;

struct ar8151_platform_ops {
	int (*suspend)(void *handle);
	int (*resume)(void *handle);
	void (*uninit)(void *handle);
};

/*	msm_11ad_dev_init - call when binding to device, during probe()
 *	@dev:	device structure of pci device
 *	@ops:	pointer to operations supported by platform driver
 *		Will be filled by this function call
 */
void *msm_ar8151_dev_init(struct device *dev);

/* call on insmod */
int msm_ar8151_modinit(void);

/* call on rmmod */
void msm_ar8151_modexit(void);

/* get ops */
struct ar8151_platform_ops *msm_ar8151_get_ops(struct pci_dev *pcidev);

#endif /* __MSM_AR8151_H__ */
