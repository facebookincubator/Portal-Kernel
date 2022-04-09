/************************************************************************
* Copyright (C) 2012-2017, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_supported_ic.c
*
* Author: Software Development
*
* Created: 2016-08-01
*
* Abstract: test item for FT8716
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/

#include "../include/focaltech_test_supported_ic.h"
#include "../focaltech_test_config.h"

#if (FTS_CHIP_TEST_TYPE == FT8607_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT8607_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT8607_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT8607;
	g_stTestFuncs.Start_Test = FT8607_StartTest;
	g_stTestFuncs.Get_test_data = FT8607_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT8716_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT8716_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT8716_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT8716;
	g_stTestFuncs.Start_Test = FT8716_StartTest;
	g_stTestFuncs.Get_test_data = FT8716_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT3D47_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT3D47_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT3D47_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT3D47;
	g_stTestFuncs.Start_Test = FT3D47_StartTest;
	g_stTestFuncs.Get_test_data = FT3D47_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT5X46_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT5X46_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT5X46_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT5X46;
	g_stTestFuncs.Start_Test = FT5X46_StartTest;
	g_stTestFuncs.Get_test_data = FT5X46_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT6X36_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT6X36_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT6X36_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT6X36;
	g_stTestFuncs.Start_Test = FT6X36_StartTest;
	g_stTestFuncs.Get_test_data = FT6X36_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT5822_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT5822_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT5822_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT5822;
	g_stTestFuncs.Start_Test = FT5822_StartTest;
	g_stTestFuncs.Get_test_data = FT5822_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT8006_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT8006_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT8006_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT8006;
	g_stTestFuncs.Start_Test = FT8006_StartTest;
	g_stTestFuncs.Get_test_data = FT8006_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT8736_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT8736_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT8736_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT8736;
	g_stTestFuncs.Start_Test = FT8736_StartTest;
	g_stTestFuncs.Get_test_data = FT8736_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FTE716_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FTE716_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FTE716_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FTE716;
	g_stTestFuncs.Start_Test = FTE716_StartTest;
	g_stTestFuncs.Get_test_data = FTE716_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT8606_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT8606_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT8606_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT8606;
	g_stTestFuncs.Start_Test = FT8606_StartTest;
	g_stTestFuncs.Get_test_data = FT8606_get_test_data;

	FTS_TEST_FUNC_EXIT();
}

#elif (FTS_CHIP_TEST_TYPE == FT3C47_TEST)

void fts_test_funcs(void)
{
	FTS_TEST_FUNC_ENTER();

	g_stTestFuncs.OnInit_TestItem = OnInit_FT3C47_TestItem;
	g_stTestFuncs.OnInit_BasicThreshold = OnInit_FT3C47_BasicThreshold;
	g_stTestFuncs.SetTestItem = SetTestItem_FT3C47;
	g_stTestFuncs.Start_Test = FT3C47_StartTest;
	g_stTestFuncs.Get_test_data = FT3C47_get_test_data;

	FTS_TEST_FUNC_EXIT();
}
#endif
