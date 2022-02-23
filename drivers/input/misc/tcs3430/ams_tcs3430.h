/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

/*! \file
 * \brief Device driver for monitoring ambient light intensity in (lux)
 * and color temperature functionality within the AMS-TAOS TCS3430 family of
 * devices.
 */

#ifndef __TCS3430_H
#define __TCS3430_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>

#define AMS_MUTEX_LOCK(m) { \
	mutex_lock(m); \
}
#define AMS_MUTEX_UNLOCK(m) { \
	mutex_unlock(m); \
}

/* Default Params */
#define COEF_SCALE_DEFAULT 1000
#define DGF_SCALE_DEFAULT  1000
#define DGF_DEFAULT        801360
#define CLR_COEF_DEFAULT   (1290)
#define RED_COEF_DEFAULT   (-1040)
#define GRN_COEF_DEFAULT   2410
#define BLU_COEF_DEFAULT   (-2510)
#define CT_COEF_DEFAULT    (4865)
#define CT_OFFSET_DEFAULT  (1801)
#define CHIP_ID_MASK       0xfc
#define CHIP_REV_MASK      0x07

/* fraction out of 10 */
#define TENTH_FRACTION_OF_VAL(v, x) ({ \
	int __frac = v; \
	if (((x) > 0) && ((x) < 10)) \
	__frac = (__frac*(x)) / 10 ; \
	__frac; \
})

enum tcs3430_regs {
	TCS3430_REG_ENABLE       = 0x80,
	TCS3430_REG_ATIME        = 0x81,
	TCS3430_REG_WTIME        = 0x83,
	TCS3430_REG_AILTL        = 0x84,
	TCS3430_REG_AILTH        = 0x85,
	TCS3430_REG_AIHTL        = 0x86,
	TCS3430_REG_AIHTH        = 0x87,

	TCS3430_REG_PERS         = 0x8C,
	TCS3430_REG_CFG0         = 0x8D,
	TCS3430_REG_CFG1         = 0x90,
	TCS3430_REG_REVID        = 0x91,
	TCS3430_REG_ID           = 0x92,
	TCS3430_REG_STATUS       = 0x93,
	TCS3430_REG_CH0DATAL     = 0x94,
	TCS3430_REG_CH0DATAH     = 0x95,
	TCS3430_REG_CH1DATAL     = 0x96,
	TCS3430_REG_CH1DATAH     = 0x97,
	TCS3430_REG_CH2DATAL     = 0x98,
	TCS3430_REG_CH2DATAH     = 0x99,
	TCS3430_REG_CH3DATAL     = 0x9A,
	TCS3430_REG_CH3DATAH     = 0x9B,

	TCS3430_REG_CFG2         = 0x9F,
	TCS3430_REG_CFG3         = 0xAB,
	TCS3430_REG_AZ_CONFIG    = 0xD6,
	TCS3430_REG_INTENAB      = 0xDD,
};

enum tcs3430__reg {
	TCS3430_MASK_INT_RD_CLR = 0x80,
	TCS3430_SHIFT_INT_RD_CLR = 7,

	TCS3430_MASK_SAI = 0x10,
	TCS3430_SHIFT_SAI = 4,

	TCS3430_MASK_IR_MUX  = 0x08,
	TCS3430_SHIFT_IR_MUX = 3,

	TCS3430_MASK_AGAIN = 0x03,
	TCS3430_SHIFT_AGAIN = 0,

	TCS3430_MASK_HGAIN = 0x10,
	TCS3430_SHIFT_HGAIN = 4,

	TCS3430_MASK_APERS = 0x0F,
	TCS3430_SHIFT_APERS = 0,

	TCS3430_MASK_WLONG = 0x04,
	TCS3430_SHIFT_WLONG = 2,

	TCS3430_MASK_AZ_MODE = 0x80,
	TCS3430_SHIFT_AZ_MODE = 7,

	TCS3430_MASK_AZ_ITERATION = 0x7F,
	TCS3430_SHIFT_AZ_ITERATION = 0,
};

enum tcs3430_en_reg {
	TCS3430_PON  = (1 << 0),
	TCS3430_AEN  = (1 << 1),
	TCS3430_WEN  = (1 << 3),
	TCS3430_EN_ALL = (TCS3430_AEN |
			  TCS3430_WEN),
};

enum tcs3430_status {
	TCS3430_ST_ALS_IRQ    = (1 << 4),
	TCS3430_ST_ALS_SAT    = (1 << 7),
};

enum tcs3430_intenab_reg {
	TCS3430_AIEN = (1 << 4),
	TCS3430_ASIEN = (1 << 7),
};

#define MAX_REGS 256
struct device;

enum tcs3430_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tcs3430_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
};

#define WAIT_CYCLE        2780
#define INTEGRATION_CYCLE 2780
#define AW_TIME_MS(p)  (((((p) * 1000) + \
	(INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE) - 1)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

#define TCS3430_INTEGRATION_FACTOR         1000
#define ATIME_TO_MS(p) ((((p + 1) * WAIT_CYCLE) / TCS3430_INTEGRATION_FACTOR))

/* lux */
#define TCS3430_MAX_ALS_VALUE         0xFFFF
#define TCS3430_MAX_LUX               0xFFFF
#define TCS3430_ALS_CH_WIDTH          0xFFFF
#define TCS3430_MIN_ALS_VALUE         0
#define TCS3430_CFG2_DEFAULT_VALUE    0x4

#define TCS3430_CAL_STATUS_CCT_3000_MASK_H 0x0001
#define TCS3430_CAL_STATUS_CCT_5000_MASK_H 0x0002
#define TCS3430_CAL_STATUS_LUX_1000_MASK_H 0x0004
#define TCS3430_CAL_STATUS_ALL_MASK_H 0x0007

struct tcs3430_parameters {
	/* Common */
	u8  persist;
	/* ALS / Color */
	u8  als_gain;
	u8  als_auto_gain;
	u16 als_deltaP;
	u8  als_time;
};

struct tcs3430_als_info {
	u32 cpl;
	u32 saturation;
	u32 z_raw;
	u32 y_raw;
	u32 ir_raw;
	u32 x_raw;
	u32 cct;
	u32 lux;
};

struct tcs3430_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct tcs3430_als_info als_inf;
	struct tcs3430_als_info als_inf_temp;
	struct tcs3430_parameters params;
	struct tcs3430_i2c_platform_data *pdata;
	struct delayed_work  polling_work;
	u8 shadow[MAX_REGS];

	struct input_dev *a_idev;

	int in_suspend;
	u8 chipid;
	bool unpowered;
	bool als_enabled;
	bool cal_enabled;
	bool in_asat;
	bool als_polling;
	int cal_cct_status;
	int cal_lux_status;
	int cal_level;
	int delay;
	int cal_report_max_num;
	struct regulator *vdd;
	u8 device_index;
	int en_count;
};

/* Must match definition in ../arch file */
struct tcs3430_i2c_platform_data {
	/* The following callback for power events received and handled by
	 * the driver.  Currently only for SUSPEND and RESUME
	 */
	int (*platform_power)(struct device *dev, enum tcs3430_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);

	char const *als_name;
	struct tcs3430_parameters parameters;
	struct device_node  *of_node;
};

#endif /* __TCS3430_H */
