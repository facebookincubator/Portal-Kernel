/*
 *****************************************************************************
 * Copyright by ams AG							*
 * All rights are reserved.						  *
 *										*
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING	*
 * THE SOFTWARE.								*
 *										*
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY	*
 * EXCLUDED.								 *
 *										*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS	*
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT	 *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS	 *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,	*
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	  *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,	*
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY	*
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT	*
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE	*
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.	*
 *****************************************************************************
 */

/*! \file
 * \brief Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), Gesture, and Beam functionality within the
 * AMS-TAOS TCS3440 family of devices.
 */

#ifndef __TCS3440_H
#define __TCS3440_H

#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>
#endif
#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
		pr_info("%s: Mutex Lock\n", __func__); \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		pr_info("%s: Mutex Unlock\n", __func__); \
		mutex_unlock(m); \
	}
#else
#define AMS_MUTEX_LOCK(m) { \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		mutex_unlock(m); \
	}
#endif

// fraction out of 10
#define TENTH_FRACTION_OF_VAL(v, x) ({ \
	int __frac = v; \
	if (((x) > 0) && ((x) < 10)) \
		__frac = (__frac*(x)) / 10 ; \
	__frac; \
})

// these are the numbers used for the one-shot spectral analysis. The spectral library processes 10 32 bit
// sensor values F1..F8, NIR and CLR. The Channel output data is 6 16 bit values at the end of each spectral
// cycle. For a spectral one-shot, one page or 4096 bytes or 1024 32 bit values are written. The continuous mode
// will continue to the next page until the user stops or the device is powered off

#define MAX_TH_VALUE 65535
#define MAX_ASTEP 65535
#define MAX_ATIME 255
#define MAX_PERSIST 15
#define MAX_WTIME 255
#define MAX_AZCONFIG 255
#define MAX_AGAIN 9
#define MAX_FD_TIME 1023
#define MAX_FD_CFG 255
#define MAX_INTENAB 255
#define MAX_REG_VAL 255
/* ALS Gain uses 5 bits with a max of 256x which is 9 - data sheet */
#define MAX_AGAIN 9
#define ONE_PAGE_CHANNEL_DATA 1024
// The spectral library expects rows of sensor data with each row being 10 32-bit values
#define SENSOR_ROW_SIZE 10
// Two consecutive spectral integration cycles yields 12 16-bit values , F1..F4, NIR and CLR and F5..F8, NIR and CLR
#define SPEC_CYCLE_SIZE 12
//Fifo
#define MAX_FIFO_LEN 512
#define MAX_FIFO_REPORT_SZ 8192
// Mode string length
#define MODE_STR_LEN 30
// Map linux events to TCS3440 events
#define FIFO_DEPTH_EVENT ABS_VOLUME
#define SINT_EVENT	ABS_DISTANCE

enum tcs3440_regs {
	TCS3440_REGADDR_RAM_START = 0x00,
	TCS3440_REGADDR_CHIP_ID = 0x0B,

	TCS3440_REGADDR_CONFIG = 0x70,
	TCS3440_REGADDR_STAT = 0x71,
	TCS3440_REGADDR_EDGE = 0x72,
	TCS3440_REGADDR_GPIO = 0x73,
	TCS3440_REGADDR_LED = 0x74,
	TCS3440_REGADDR_ENABLE = 0x80,
	TCS3440_REGADDR_ITIME = 0x63,
	TCS3440_REGADDR_ATIME = 0x81,
	TCS3440_REGADDR_WTIME = 0x83,
	TCS3440_REGADDR_SPEC_L = 0x84,
	TCS3440_REGADDR_SPEC_H = 0x86,

	TCS3440_REGADDR_AUXID = 0x90,
	TCS3440_REGADDR_REVID = 0x91,
	TCS3440_REGADDR_ID = 0x92,

	TCS3440_REGADDR_STATUS = 0x93,
	TCS3440_REGADDR_ASTATUS = 0x94,
	TCS3440_REGADDR_CH0_DATA = 0x95,
	TCS3440_REGADDR_CH1_DATA = 0x97,
	TCS3440_REGADDR_CH2_DATA = 0x99,
	TCS3440_REGADDR_CH3_DATA = 0x9B,
	TCS3440_REGADDR_CH4_DATA = 0x9D,
	TCS3440_REGADDR_CH5_DATA = 0x9F,
	TCS3440_REGADDR_STATUS2 = 0xA3,
	TCS3440_REGADDR_STATUS5 = 0xA6,
	TCS3440_REGADDR_STATUS6 = 0xA7,

	TCS3440_REGADDR_CFG0 = 0xA9,
	TCS3440_REGADDR_CFG1 = 0xAA,
	TCS3440_REGADDR_CFG3 = 0xAC,
	TCS3440_REGADDR_CFG4 = 0xAD,
	TCS3440_REGADDR_CFG6 = 0xAF,
	TCS3440_REGADDR_CFG8 = 0xB1,
	TCS3440_REGADDR_CFG9 = 0xB2,
	TCS3440_REGADDR_CFG10 = 0xB3,
	TCS3440_REGADDR_CFG11 = 0xB4,
	TCS3440_REGADDR_CFG12 = 0xB5,
	TCS3440_REGADDR_PERS = 0xBD,

	TCS3440_REGADDR_GPIO2 = 0xBE,

	TCS3440_REGADDR_ASTEP1 = 0xCA,
	TCS3440_REGADDR_ASTEP2 = 0xCB,
	TCS3440_REGADDR_AGC_GAIN_MAX = 0xCF,

	TCS3440_REGADDR_AZCONFIG = 0xD6,
	TCS3440_REGADDR_FD_CFG0 = 0xD7,
	TCS3440_REGADDR_FD_TIME_L = 0xD8,
	TCS3440_REGADDR_FD_TIME_H = 0xDA,
	TCS3440_REGADDR_FLICKR_STATUS = 0xDB,

	TCS3440_REGADDR_INTENAB = 0xF9,
	TCS3440_REGADDR_CONTROL = 0xFA,
	TCS3440_REGADDR_FIFO_MAP = 0xFC,
	TCS3440_REGADDR_FIFO_LVL = 0xFD,
	TCS3440_REGADDR_FDATA_L = 0xFE,
	TCS3440_REGADDR_FDATA_H = 0xFF
};

#define STR1_MAX 16
#define STR2_MAX 256
#define MAX_REGS 256
struct device;

enum tcs3440_pwr_state {
	POWER_ON, POWER_OFF, POWER_STANDBY,
};


/* pldrive */
#define PDRIVE_MA(p)	(((u8)((p) / 6) - 1) & 0x1f)
#define P_TIME_US(p)	((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define MULT_MS_US 1000
#define INTEGRATION_CYCLE 2800
#define AW_TIME_MS(p)  ((((p) * 1000) +\
	(INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

/* lux */
#define INDOOR_LUX_TRIGGER	6000
#define OUTDOOR_LUX_TRIGGER	10000
#define TCS3440_MAX_LUX		0xffff
#define TCS3440_MAX_ALS_VALUE	0xffff
#define TCS3440_MIN_ALS_VALUE	1

/**
 * struct aos_sensor_item_data - stores the data of an item which is processed by the chiplib
 * @p_data: - buffer holding the data of the item
 * @size: - size of the buffer
 */
struct aos_sensor_item_data {
	uint8_t *p_data;
	uint32_t size;
};

/**
 * struct aos_sensor_item - defines an item and its data which is processed by the chiplib
 * @item_id: - unique ID of the item for identification (must be equal to the item ids processed by the chiplib)
 * @data: - the data of the item
 */
struct aos_sensor_item {
	uint32_t item_id;
	struct aos_sensor_item_data data;
};

/**
 * struct aos_temp_node - A temperature node to read/write from
 * @id: - unique ID of the node for identification (shall be equal to the id processed by the chiplib)
 * @value: - the temperature of the node to read/write
 */
struct aos_temp_node {
	uint16_t id;
	int32_t value;
};

enum tcs3440_status{
	TCS3440_SINT = (1 << 0),
	TCS3440_CINT = (1 << 1),
	TCS3440_FINT = (1 << 2),
	TCS3440_AINT = (1 << 3),
	TCS3440_ASAT = (1 << 7)
};

enum tcs3440_auxid{
	TCS3440_AUXID = (1 << 1),
	TCS3440_OTP1 = (1 << 7)
};

enum tcs3440__reg {
	TCS3440_MASK_START_OFFSET_CALIB = 0x01,
	TCS3440_SHIFT_START_OFFSET_CALIB = 0,
	/*Note - MASK_AGAIN is for AGC_GAIN_MAX */
	TCS3440_MASK_AGC_AGAIN = 0x0f,
	TCS3440_MASK_AGAIN = 0x1f,
	TCS3440_MASK_CFG1 = 0x1f,
	TCS3440_MASK_FDTIME2 = 0x03,
	TCS3440_SHIFT_AGAIN = 0,

	TCS3440_MASK_APERS = 0x0f,
	TCS3440_SHIFT_APERS = 0,

	TCS3440_MASK_WLONG = 0x04,
	TCS3440_MASK_LOW_POWER = (1 << 5),
	TCS3440_SHIFT_WLONG = 2,
};

enum tcs3440_en_reg {
	TCS3440_PON = (1 << 0),
	TCS3440_AEN = (1 << 1),
	TCS3440_WEN = (1 << 3),
	TCS3440_MUXEN = (1 << 4),
	TCS3440_FDEN = (1 << 6),
	TCS3440_FD_ALS = (TCS3440_FDEN | TCS3440_MUXEN | TCS3440_AEN | TCS3440_WEN | TCS3440_PON),
	TCS3440_MUX_ON = (TCS3440_MUXEN | TCS3440_PON),
	TCS3440_MUX_ALS = (TCS3440_MUXEN | TCS3440_AEN | TCS3440_WEN | TCS3440_PON)
};

enum tcs3440_cfg_reg {
	TCS3440_SPECTRAL_MODE,
	TCS3440_SYNS_MODE,
	TCS3440_MODE_RESERVED,
	TCS3440_SYND_MODE,
	TCS3440_SYNC_INT = (1 << 2),
	TCS3440_LED_SEL = (1 << 3)
};

enum tcs3440_gpio_reg {
	TCS3440_PD_INT = (1 << 0),
	TCS3440_PD_GPIO = (1 << 1)
};

enum tcs3440_gpio2_reg {
	TCS3440_GPIO_OUT = (1 << 1),
	TCS3440_GPIO_IN_EN = (1 << 2),
	TCS3440_GPIO_INV = (1 << 3)
};

enum tcs3440_intenab_reg {
	TCS3440_INTENAB_SIEN = (1 << 0),
	TCS3440_INTENAB_FIEN = (1 << 2),
	TCS3440_INTENAB_AIEN = (1 << 3),
	TCS3440_INTENAB_ASIEN = (1 << 7),
	TCS3440_ANY_INTENAB = (TCS3440_INTENAB_SIEN | TCS3440_INTENAB_FIEN | TCS3440_INTENAB_AIEN | TCS3440_INTENAB_ASIEN)
};

enum tcs3440_cfg8_reg {
	TCS3440_SP_AGC = (1 << 2),
	TCS3440_FD_AGC = (1 << 3)
};

enum tcs3440_control_reg {
	TCS3440_CLEAR_SAI_ACT = (1 << 0),
	TCS3440_FIFO_CLR = (1 << 1),
	TCS3440_SP_MAN_AZ = (1 << 2)
};

enum tcs3440_status2_reg {
	TCS3440_FDSTAT_DIG = (1 << 0),
	TCS3440_FDSTAT_ANA = (1 << 1),
	TCS3440_ANA_SAT = (1 << 2),
	TCS3440_DIG_SAT = (1 << 3),
	TCS3440_AVALID = (1 << 6)
};

enum tcs3440_fifomap {
	TCS3440_CH0 = (1 << 1),
	TCS3440_CH1 = (1 << 2),
	TCS3440_CH2 = (1 << 3),
	TCS3440_CH3 = (1 << 4),
	TCS3440_CH4 = (1 << 5),
	TCS3440_CH5 = (1 << 6)
};

enum tcs3440_spec_measure_status {
	TCS3440_SPEC_STATUS = (1 << 0),
	TCS3440_SPEC_SYNCD = (1 << 1)
};

enum tcs3440_agc_again {
	AGAINL = (1 << 2),
	AGAINMAX = (1 << 4),
};

enum tcs3440_status6 {
	TCS3440_INIT_BUSY = (1 << 0),
	TCS3440_SAI_ACTIVE = (1 << 1),
	TCS3440_SP_TRIG = (1 << 2),
	TCS3440_FD_TRIG = (1 << 4),
	TCS3440_OVTEMP = (1 << 5),
	TCS3440_FIFO_OV = (1 << 7)
};

enum tcs3440_cfg6 {
	TCS3440_SMUX = (1 << 4),
	TCS3440_AGC_GAIN_MAX = (1 << 6)
};

enum tcs3440_cfg8 {
	TCS3440_AGC_ENABLE = (1 << 2)
};

enum tcs3440_cfg9 {
	TCS3440_SIEN_FD = (1 << 6)
};

enum tcs3440_cfg11 {
	TCS3440_AINT_DIRECT = (1 << 7)
};

enum tcs3440_cfg12 {
	TCS3440_MASK_TH_CHANNEL = 0x03
};

enum tcs3440_fd_cfg0 {
	TCS3440_FD_CFG0 = (1 << 7)
};

enum tcs3440_status5 {
	TCS3440_SINT_FD = (1 << 3)
};

enum tcs3440_cfg0_masks {
	TCS3440_LOW_POWER = (1 << 5),
	TCS3440_ALS_LONG = (1 << 2),
	TCS3440_RAM_BANK = 0x03,
	TCS3440_ANY_CFG0 = (TCS3440_LOW_POWER | TCS3440_ALS_LONG | TCS3440_RAM_BANK)
};

enum tcs3440_cfg3_masks {
	TCS3440_SAI = (1 << 4)
};

enum tcs3440_cfg4_masks {
	TCS3440_INTMAP = 0x70,
	TCS3440_INVERT = (1 << 3)
};


struct tsl2540_als_info {
	u16 als_ch0; /* photopic channel */
	u16 als_ch1; /* ir channel */
	u32 cpl;
	u32 saturation;
	u16 lux;
};

enum tcs3440_flickr_status {
	TCS3440_FLICKR_VALID = (1 << 5),
	TCS3440_FLICKR_SAT_DETECT = (1 << 4),
	TCS3440_FLICKR_120Hz_VALID = (1 << 3),
	TCS3440_FLICKR_100Hz_VALID = (1 << 2),
	TCS3440_FLICKR_120Hz = (1 << 1),
	TCS3440_FLICKR_100Hz = (1 << 0)
};


typedef enum tcs3440_modes_of_operation {
	TCS3440_MODE_IDLE,
	TCS3440_ALS_MODE,
} operation_mode;

typedef enum tcs3440_mux_state {
	TCS3440_MUX_CFG_A_STATE,
	TCS3440_MUX_CFG_B_STATE
} tcs3440_mux_state_t;

struct tcs3440_parameters {
	u32 itime;
	u8 config; /* register 0x70 - mode */
	u8 spectral_stat; /* register 0x71 */
	u8 led_reg; /* 0x74 */
	u8 gpio_cfg;
	u8 enable; /* reg 0x80 */
	u8 atime; /* als_time */
	u8 wtime;
	u16 spectral_l;
	u16 spectral_h;
	u8 status; /*reg 0x93 */
	u8 astatus; /* reg 0x94 */
	u8 status2; /* reg 0xA3 */
	u8 status3; /* reg 0xA4 */
	u8 status5;
	u8 status6;
	u8 cfg0; /* reg 0xA9 - ALS trigger long and bank selection */
	u8 cfg3; /* reg 0xAB - Sleep after interrupt */
	u8 cfg4; /* reg 0xAD - Interrupt pin map and interrupt invert */
	u8 cfg8; /* reg 0xB1 - spectral threshold - fd auto gain control and spectral again*/
	u8 cfg6; /* reg 0xAF - ALS saturation decrement */
	u8 cfg9; /* reg 0xB2 - flicker detection interrupt */
	u8 cfg10; /* reg 0xB3 - hysterisis - AGC - Low and High */
	u8 cfg11; /* reg 0xB4 - ALS interrupt */
	u8 cfg12; /* reg 0xB5 - spectral threshold channel */
	u8 persist; /* reg 0xBD */
	u8 gpio2;
	u8 ls_astep;
	u8 ms_astep;
	u8 again; /* CFG1 reg 0xAA - Auto gain or again */
	u8 again_max; /* reg 0xCF */
	u8 azconfig;
	u8 fd_time1;
	u8 fd_time2;
	u8 fd_cfg0;
	u8 fd_status;
	u8 intenab;
	u8 control;
	u8 fifo_map;
	u8 fifo_lvl;
	u8 valid;
	u8 init_state;
	u8 sai_enable;
};

#define MATRIX_ROW_SIZE 3
#define MATRIX_COL_SIZE 10
#define NOMINAL_ATIME_DEFAULT	204800 /* 50ms Q20.12 */
#define NOMINAL_AGAIN_DEFAULT	4096

#define TCS3440_CAL_STATUS_CCT_3000_MASK_H 0x0001
#define TCS3440_CAL_STATUS_CCT_5000_MASK_H 0x0002
#define TCS3440_CAL_STATUS_LUX_1000_MASK_H 0x0004
#define TCS3440_VALITATION_HIGH_IR_MASK_H       0x0008
#define TCS3440_VALITATION_LOW_IR_MASK_H        0x0010
#define TCS3440_CAL_STATUS_ALL_MASK_H 0x001F

enum cal_ch_index {
	CH_IDX_F1 = 0,
	CH_IDX_F2,
	CH_IDX_F3,
	CH_IDX_F4,
	CH_IDX_F5,
	CH_IDX_F6,
	CH_IDX_F7,
	CH_IDX_F8,
	CH_IDX_CLEAR,
	CH_IDX_NIR
};

enum cal_tri_index {
	TRI_IDX_X = 0,
	TRI_IDX_Y,
	TRI_IDX_Z
};

enum calibration_process_state {
	CALI_NONE = 0,
	CALI_3000,
	CALI_5000,
	CALI_4000H,
	CALI_4000L,
	CALI_DARK_COUNT,
	CALI_MAX
};

enum calibration_process_status {
	CALI_STATUS_STOP = 0,
	CALI_STATUS_PROCESSING,
	CALI_STATUS_FINISH,
	CALI_STATUS_FAIL,
	CALI_STATUS_MAX
};

struct matrix_data {
	int32_t data[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
	uint32_t q_factor;
};

struct calibration_data {
	struct matrix_data coef;
	uint32_t nominal_atime;
	uint32_t nominal_again;
};

struct adc_data {
	uint32_t f1;
	uint32_t f2;
	uint32_t f3;
	uint32_t f4;
	uint32_t f5;
	uint32_t f6;
	uint32_t f7;
	uint32_t f8;
	uint32_t clear;
	uint32_t nir;
	uint32_t again;
	uint32_t atime;
	uint32_t astep;
};

struct cie_tristimulus {
	uint64_t x;
	uint64_t y;
	uint64_t z;
};

struct als_xyz_data {
	struct cie_tristimulus tristimulus;
	uint64_t chromaticity_x;
	uint64_t chromaticity_y;
	uint32_t lux;
	uint32_t cct;
};

struct tcs3440_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct tcs3440_parameters params;
	struct tcs3440_i2c_platform_data *pdata;
	wait_queue_head_t fifo_wait;
	u8 shadow[MAX_REGS];

	struct regulator *vdd;
	struct input_dev *als_idev;
	struct input_dev *cct_idev;
#ifdef CONFIG_QUALCOMM_AP
	struct sensors_classdev als_cdev;
#endif
	u16 raw_data[13];
	int cal_lux_status;
	int cal_cct_status;
	int cal_level;

	struct als_xyz_data xyz;
	int in_suspend;
	int wake_irq;
	int irq_pending;

	bool unpowered;
	bool enabled;
	bool is_als_valid;
	bool is_spectral_ready;
	bool in_asat;
	bool amscalcomplete;
	bool is_first_smux_done;
	u8 auxid;
	uint8_t rev;
	u8 id;
	/* The output data CH0 thru CH5 u16 will be in an array */
	u32 saturation;
	struct task_struct *tcs3440_spectral;
	u8 device_index;
	bool is_data_read;
	operation_mode mode;
	bool als_input_open;
	bool valid_auxid;
	bool driver_remove;
	tcs3440_mux_state_t mux_state;
	struct adc_data adc_result;
	bool ams_output_enable;
};


/* Must match definition in ../arch file */
struct tcs3440_i2c_platform_data {
	/* The following callback for power events received and handled by
	 the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct tcs3440_chip *chip, enum tcs3440_pwr_state state);
	int (*platform_init)(struct tcs3440_chip *chip);
	void (*platform_teardown)(struct device *dev);

	char const *als_name;
	struct tcs3440_parameters parameters;
	bool als_can_wake;
	u32 ams_irq_gpio; /* as per DTS */
	u16 out_data[24];
	int pos;

#ifdef CONFIG_OF
	struct device_node *of_node;
#endif
};

#endif /* __TCS3440_H */
