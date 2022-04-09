/*
 * tc358840.h - Toshiba HDMI-CSI bridge registers
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright (c) 2017, Plamen Valev <pvalev@mm-sol.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TC358840_H_
#define __TC358840_H_

extern void tc358840_start_stream(void);
extern void tc358840_stop_stream(void);

#endif /* __TC358840_H_ */
