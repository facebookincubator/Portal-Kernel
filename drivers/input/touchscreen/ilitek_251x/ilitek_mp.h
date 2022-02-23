/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Jijie Wang <jijie_wang@ilitek.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 *
 */

#ifndef __ILITEK_PARSER_H
#define __ILITEK_PARSER_H
#include "ilitek_ts.h"
#include "ilitek_common.h"
#define BENCHMARK_KEY_NAME "Uniformity_Benchmark"
#define ALLNODE_WIN2_KEY_NAME "AllNode_Win2"
#define ALLNODE_RAWDATA_KEY_NAME "AllNode_RawData"
#define ALLNODE_WIN1_KEY_NAME "AllNode_Win1"
#define NODE_KEY_NAME "node"
#define VALUE 0
#define DIVIDE_10M  150

/*//21408=2.45(Vrefm)*65535*4/30(dump)      216=60(cfb)*3.6V */
#define IMPEDANCE_MACRO(fout, dump1, dump2) ((int)(2140810 * abs(dump2-dump1)) /(216 * fout)) 

enum mp_test_catalog {
	MUTUAL_TEST = 0,
	SELF_TEST = 1,
	KEY_TEST = 2,
	ST_TEST = 3,
	TX_RX_DELTA = 4,
	UNTOUCH_P2P = 5,
	PIXEL = 6,
	OPEN_TEST = 7,
	PEAK_TO_PEAK_TEST = 8,
	SHORT_TEST = 9,
};


static inline void *ilitek_memcpy(void *dest, const void *src, int n, int dest_size)
{
	if (n > dest_size)
		 n = dest_size;

	return memcpy(dest, src, n);
}
extern void parser_node(char *pKeyValue, struct ilitek_node_info * ptr, int frame_len, int y);
extern void get_parser_node(char *section, struct ilitek_node_info *ptr, int x);
extern int parser_get_int_data(char *section, char *keyname, char *rv);
extern int parser_path(char *path);
extern void ilitek_printf_sensortest_data_v6(int32_t *short_xdata1, int32_t *short_ydata1, int32_t *open_data, 
						int32_t *allnode_data, int32_t *allnode_win1, int32_t *allnode_win2, struct seq_file *m);
extern void ilitek_printf_sensortest_data(int32_t *short_xdata1, int32_t *short_xdata2, int32_t *short_ydata1,
					  int32_t *short_ydata2, int32_t *open_data, int32_t *allnode_data, struct seq_file *m);
extern int32_t ilitek_allnode_test(int32_t *allnode_data);
extern int32_t ilitek_allnode_test(int32_t *allnode_data);
extern int32_t ilitek_short_test(int32_t *short_xdata1, int32_t *short_xdata2, int32_t *short_ydata1, int32_t *short_ydata2);
extern int32_t ilitek_short_test_v6(int32_t *short_xdata1, int32_t *short_ydata1);
extern int32_t ilitek_open_test_v6(int32_t *short_open);
extern int32_t ilitek_allnode_test_v6(int32_t *raw_data, int32_t *short_win1 , int32_t *short_win2);
extern int32_t ilitek_uniformity_test_v6(int32_t *short_allnode , int32_t *short_win1 , int32_t *short_win2);
extern int32_t ilitek_open_test(int32_t *open_data);
extern void ilitek_sensortest_init(void);
#endif
