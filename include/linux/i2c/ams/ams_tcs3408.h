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

#ifndef __TCS3408_H
#define __TCS3408_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>

#define CLOCK_FREQUENCY 720000
//#define ABI_SET_GET_REGISTERS

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
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

/* Default Params */

#define COEF_SCALE_DEFAULT 1000
#define DGF_DEFAULT		898
#define CLR_COEF_DEFAULT   (210)
#define RED_COEF_DEFAULT   (-60)
#define GRN_COEF_DEFAULT   (10)
#define BLU_COEF_DEFAULT   (-290)
#define CT_COEF_DEFAULT	(4366)
#define CT_OFFSET_DEFAULT  (1849)

#define PROX_MAX_THRSHLD   (0x3FFF)

// fraction out of 10
#define TENTH_FRACTION_OF_VAL(v, x) { \
		int __frac = v; \
		if (((x) > 0) && ((x) < 10)) \
			__frac = (__frac*(x)) / 10; \
		__frac; \
	}


#define MAX_REGS 256
struct device;

enum tcs3408_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

// pldrive
#define PDRIVE_MA(p) ({ \
		u8 __reg = (((u8)((p) - 2) / 2) & 0xf); \
		/* artf24717 limit PLDRIVE to 19mA */ \
		__reg = (__reg > 0x08) ? 0x08 : __reg; \
		__reg; \
})
#define P_TIME_US(p) ((((p) / 88) - 1.0) + 0.5)
//#define PTIME_MS(p) (((p) * 1000) / INTEGRATION_CYCLE)
#define PTIME_MS (1000 / INTEGRATION_CYCLE)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define INTEGRATION_CYCLE 2780
#define AW_TIME_MS(p) ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) \
	/ INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

// lux
#define INDOOR_LUX_TRIGGER	6000
#define OUTDOOR_LUX_TRIGGER	10000
#define TCS3408_MAX_LUX		0xffff
#define TCS3408_MAX_ALS_VALUE	0xffff
#define TCS3408_MIN_ALS_VALUE	10

struct tcs3408_parameters {
	/* Common */
	u8 smux[7];
	u8 single_input;
};

struct ams_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct gpio_desc *gpiod_interrupt;
	struct tcs3408_parameters params;
	struct tcs3408_i2c_platform_data *pdata;
	struct work_struct channel_config_work;
	struct input_dev *input;
	struct feature_callbacks *als;
	struct feature_callbacks *flicker;
	struct feature_callbacks *fifo;
	struct regulator *vdd;
	uint32_t frequency;
	uint8_t shadow[MAX_REGS];
	uint8_t channel_config;
	uint8_t channels[7];
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool powered;
	bool amsCalComplete;
	bool amsFirstProx;
	bool amsIndoorMode;
	bool in_calib;
	bool agc_disable;
	struct task_struct *poll_irq;
	int brightness;
	uint8_t device_index;
};

struct feature_callbacks {
	void (*irq)(struct ams_chip *chip, uint8_t status);
	void (*fifo)(struct ams_chip *chip, uint8_t *data, int size);
};

// Must match definition in ../arch file
struct tcs3408_i2c_platform_data {
	/*
	 *The following callback for power events received and handled by
	 *the driver.  Currently only for SUSPEND and RESUME
	 */
	int (*platform_power)(struct device *dev, enum tcs3408_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);

	char const *name;
	struct tcs3408_parameters parameters;
	bool proximity_can_wake;
	bool als_can_wake;
	int poll_period;
#ifdef CONFIG_OF
	  struct device_node  *of_node;
#endif
};

#endif /* __TCS3408_H */
