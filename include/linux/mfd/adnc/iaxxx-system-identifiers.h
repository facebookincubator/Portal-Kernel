/*
 * iaxxx-system-identifiers.h - IAxxx System Identifiers
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#ifndef __IAXXX_SYSTEM_IDENTIFIERS_H__
#define __IAXXX_SYSTEM_IDENTIFIERS_H__

enum iaxxx_resource_type {
	IAXXX_RESOURCE_TYPE_HOST = 0,
	IAXXX_RESOURCE_TYPE_PERIPHERALS,
	IAXXX_RESOURCE_TYPE_MODULES,
	IAXXX_RESOURCE_TYPE_PLUGIN,
	IAXXX_RESOURCE_TYPE_CHANNEL,
	IAXXX_RESOURCE_TYPE_TUNNEL,
	IAXXX_RESOURCE_TYPE_STREAM,
	IAXXX_RESOURCE_TYPE_ANY = 0xF
};

#define IAXXX_HOST_SYSTEM_ID	(IAXXX_RESOURCE_TYPE_HOST << 12)

#define IAXXX_STREAM_SYSTEM_ID(IX) \
	((IAXXX_RESOURCE_TYPE_STREAM << 12) | ((IX) & 0x1F))

#define IAXXX_CHANNEL_SYSTEM_ID(ID, DIR, IX) \
	((IAXXX_RESOURCE_TYPE_CHANNEL << 12) | (((ID) & 0x3F) << 6) | \
	(((DIR) & 0x1) << 5) | (((IX) & 0x1F)))

#define IAXXX_PLUGIN_SYSTEM_ID(ID, DIR, IX) \
	((IAXXX_RESOURCE_TYPE_PLUGIN << 12) | (((ID) & 0x3F) << 6) | \
	(((DIR) & 0x1) << 5) | (((IX) & 0x1F)))

#endif /* __IAXXX_SYSTEM_IDENTIFIERS_H__ */
