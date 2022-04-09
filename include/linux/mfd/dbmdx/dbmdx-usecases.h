/*
 * dbmdx-usecases.h  --  DBMDX Usecases Common functions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASES_H
#define _DBMDX_USECASES_H

#include "dbmdx-usecase-config-def.h"


extern struct usecase_config *usecases_map[];

int dbmdx_get_number_of_registered_usecases(void);

int hw_stop_usecase_prepare(struct dbmdx_private *p,
					struct usecase_config *uc_config);

int hw_stop_usecase(struct dbmdx_private *p,
					struct usecase_config *uc_config);

int switch_to_usecase(struct dbmdx_private *p,
			struct usecase_config *uc_config);

int find_usecase_by_name(struct dbmdx_private *p, const char *usecase_name,
			struct usecase_config **usecase);

int dbmdx_usecase_manager(struct dbmdx_private *p, u32 cmd);

#endif /* _DBMDX_USECASES_H */

