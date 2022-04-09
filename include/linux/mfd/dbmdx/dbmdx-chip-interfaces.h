/*
 * dbmdx-chip-interfaces.h  --  DBMDX Chip Interfaces API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_CHIP_INTERFACES_H
#define _DBMDX_CHIP_INTERFACES_H



#ifdef CONFIG_OF
int dbmdx_get_fw_interfaces(struct dbmdx_private *p,
				   const char *tag,
				   int *iarray);
#endif

int dbmdx_interface_probe(struct dbmdx_private *p);

int dbmdx_set_active_interface(struct dbmdx_private *p,
				       int interface_idx);

#endif /* _DBMDX_CHIP_INTERFACES_H */
