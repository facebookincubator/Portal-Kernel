/************************************************************************
* Copyright (C) 2010-2017, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_ex_mode.h
*
*  Abstract:
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_EX_MODE_H__
#define __LINUX_FOCALTECH_EX_MODE_H__

/*****************************************************************************
* 5.Global variable or extern global variabls/functions
*****************************************************************************/
int fts_enter_glove_mode(struct i2c_client *client, int mode);
int fts_glove_init(struct i2c_client *client);
int fts_glove_exit(struct i2c_client *client);

int fts_enter_cover_mode(struct i2c_client *client, int mode);
int fts_cover_init(struct i2c_client *client);
int fts_cover_exit(struct i2c_client *client);

int fts_enter_charger_mode(struct i2c_client *client, int mode);
int fts_charger_init(struct i2c_client *client);
int fts_charger_exit(struct i2c_client *client);

#endif
