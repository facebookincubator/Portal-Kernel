/*
 * dbmdx-usecase-config-def.h  --  DBMDX Definition of USE CASE configurations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_USECASE_CONFIG_DEF_H
#define _DBMDX_USECASE_CONFIG_DEF_H

#include "dbmdx-interface.h"


struct usecase_config {
	const char	*usecase_name;
	u32		id;
	u32		hw_rev;
	bool		use_hw_usecase_stop;
	void		*usecase_params;
	void		*usecase_default_params;
	void		*usecase_data;
	int		(*usecase_set_params_from_str)(struct dbmdx_private *p,
					struct usecase_config *uc_config,
					const char *cfg_str, u32 size);
	int		(*usecase_set_params)(struct dbmdx_private *p,
					struct usecase_config *uc_config,
					void *params);
	int		(*usecase_reset)(struct dbmdx_private *p,
					struct usecase_config *uc_config,
					bool reset_params);
	int		(*usecase_enter)(struct dbmdx_private *p,
					struct usecase_config *uc_config);
	int		(*usecase_exit)(struct dbmdx_private *p,
					struct usecase_config *uc_config);
	int		(*usecase_dump)(struct dbmdx_private *p,
					struct usecase_config *uc_config,
					char *buffer, u32 buf_len);
	int		(*usecase_is_active)(struct dbmdx_private *p,
					struct usecase_config *uc_config,
					bool *is_active);

};

#endif /* _DBMDX_USECASE_CONFIG_DEF_H */

