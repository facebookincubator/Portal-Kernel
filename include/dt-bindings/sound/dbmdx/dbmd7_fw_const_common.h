/**
 * dbmd7_fw_const_common.h -- Common FW constants
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DBMD7_FW_CONST_COMMON_H
#define _DBMD7_FW_CONST_COMMON_H

/****** Host message format ***********/

#define TASK_GROUP_BITN			24
#define TARGET_CORE_EXEC		30
/* bits 16-23 */
#define MSG_REGNUM_BITN			16
#define MSG_REGNUM_MASK			0xFF0000
/* bits 24,25,26,27 */
#define MSG_ID_BITN			24
#define MSG_ID_MASK			(0xF<<24)
/* bits 29,30,31 */
#define MSG_TYPE_BITN			28
/* 3 MSB bits */
#define MSG_TYPE_MASK			(7<<28)
#define MSG_PROC_IDLE_BITN		31

/* to be used as "or" (|) with register number */
#define RUN_ON_IDLE_LOOP		(1<<15)



/* #define SWAP(a,b) { int c = a ; a=b; b=c ;} */

#define DDF_FRAME_LENGTH		12

#define ATTRIB_SMP_WIDTH_8_BITS		0
#define ATTRIB_SMP_WIDTH_16_BITS	1
#define ATTRIB_SMP_WIDTH_24_BITS	2
#define ATTRIB_SMP_WIDTH_32_BITS	3

#define ATTRIB_SMP_RATE_16K		0
#define ATTRIB_SMP_RATE_44K1		1
#define ATTRIB_SMP_RATE_48K		2
#define ATTRIB_SMP_RATE_96K		3
#define ATTRIB_SMP_RATE_8K		4
#define ATTRIB_SMP_RATE_32K		5

/****** Task ID definitions  ***********************/

#define FW_TASK_ID_IDLE				0
#define FW_DATA_MIXER_TASK			1
#define FW_TASK_LOAD_CPU_TESTER			2
#define FW_TASK_COPY				FW_TASK_ID_MIC2TDM
#define FW_TASK_ID_INTERCORE1			3
#define FW_TASK_ID_INTERCORE2			4
#define FW_TEST_2MIC_INTO_TDM_TASK_ID		5

/* Mixer task command defines */
/* Terminate task activiry */
#define FW_TASK_MIXER_CMD_TERMINATE	(0)
/* Set input mailbox ID 1 */
#define FW_TASK_MIXER_CMD_INPUT_MB1_ID	(1)
/* Set input mailbox ID 2 (optional, can receive 0 to disable later) */
#define FW_TASK_MIXER_CMD_INPUT_MB2_ID	(2)
/* Set output mailbox ID 1 */
#define FW_TASK_MIXER_CMD_OUTPUT_MB1_ID	(3)
/* Set output mailbox ID 2 (optional) */
#define FW_TASK_MIXER_CMD_OUTPUT_MB2_ID	(4)
/* Set digital gain on samples from mailbox ID 1 (units of 1/16dB) */
#define FW_TASK_MIXER_CMD_INPUT_GAIN1	(5)
/* Set digital gain on samples from mailbox ID 2 (units of 1/16dB) */
#define FW_TASK_MIXER_CMD_INPUT_GAIN2	(6)

#define RESAMPLE_TASK_DECIMATE_48KHZ_16KHZ	0x10
#define RESAMPLE_TASK_DECIMATE_32KHZ_16KHZ	0x11
#define RESAMPLE_TASK_DECIMATE_44K1HZ_16KHZ	0x12
#define RESAMPLE_TASK_DECIMATE_48KHZ_32KHZ	0x13
#define RESAMPLE_TASK_INTERPOLATE_16KHZ_48KHZ	0x14
#define RESAMPLE_TASK_INTERPOLATE_16KHZ_32KHZ	0x15


#define USER_ADDON_ASRP_TASK_ID		1
#define USER_ADDON_SENSORY_TASK_ID	1

/****************************************************/

#define FW_RESMP_TASK__EXIT_TASK		0x00
#define FW_RESMP_TASK__INP_MB			0x01
#define FW_RESMP_TASK__OUT_MB			0x03
#define FW_RESMP_TASK__RES			0x04
#define FW_RESMP_TASK__NUM_SOS			0x05
#define FW_RESMP_TASK__COEF_ADR			0x06
#define FW_RESMP_TASK__COEF_MB			0x07
#define FW_RESMP_TASK__COEF_SV			0x08
#define FW_RESMP_TASK__FRAME_LEN		0x09

/* not the CPM core ID */
#define FW_COREID_HF0				1
#define FW_COREID_HF1				2
#define FW_COREID_LP				3

#define NULL_TASK_HANDLE			0
#define PRIORITY_0				0
#define PRIORITY_1				1
#define PRIORITY_2				2
#define PRIORITY_3				3

#define NO_RECORDING				0
#define RECORDING_ON_READ			1
#define RECORDING_ON_WRITE			2
#define RECORDING_ON_READ_AND_WRITE		3

#define DEBUG_MB_BEFORE_READ			1
#define DEBUG_MB_AFTER_READ			2
#define DEBUG_MB_BEFORE_WRITE			4
#define DEBUG_MB_AFTER_WRITE			8

#define USER_DEF_INT_NOTIFY_1000		1
#define USER_DEF_INT_NOTIFY_INCREMENT		2
#define USER_DEF_INT_CALL_BACK_FUNC		3
#define USER_DEF_INT_MAKE_PENDING_MSG		4

#define TDM_MB_CONFIG_DOUBLE_BUFF		0
#define TDM_MB_CONFIG_TRIPLE_BUFF		1
#define TDM_MB_CONFIG_BYPASS			7

#endif /* _DBMD7_FW_CONST_COMMON_H */
