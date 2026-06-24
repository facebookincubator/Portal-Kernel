/*
 *****************************************************************************
 * Copyright 2017 by ams AG                                                  *
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
 * functionality within the AMS TCS3440 family of devices.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>
#endif

#include "ams_tcs3440.h"
#include "ams_i2c.h"
#include "ams_tcs3440_als.h"
#include "ams_tcs3440_smux_configs.h"

#define FLICKER_VALUE ABS_MISC
#define AGC_ENABLE 0
#define POLLING_BIT_TIME_OUT	5000

#ifdef CONFIG_QUALCOMM_AP

static u8 data[256];
/*
 * warning: if we change the kfifo size, then
 * we also need to change the kfifo overflow value
 */

static struct sensors_classdev als_sensors_cdev = {
	.name = "tcs3440-als",
	.vendor = "AMS",
	.version = 1,
	.handle = 0,
	.type = 1,
	.max_range = "1",
	.resolution = "1",
	.sensor_power = "1",
	.min_delay = 10000,
	.max_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,
	.sensors_enable = NULL,
};
#endif

enum tcs3440_channel_index {
	IDX_F1 = 0,
	IDX_F2,
	IDX_F3,
	IDX_F4,
	IDX_F5,
	IDX_F6,
	IDX_F7,
	IDX_F8,
	IDX_CLEAR,
	IDX_NIR,
	IDX_AGAIN,
	IDX_ATIME,
	IDX_ASTEP
};

/* TCS3440 Identifiers */
static u8 const tcs3440_ids[] = {
	0, 2, 8};	/* TCS3440 auxids 0 - for present device, 2 - TCS3440 and 8 - AS7351 */

/*
 * TCS3440 uses bits 0->7 while AS7341 uses bits 2->7 to derive the part identification.
 * The expectation is that the derived ID matches between both parts to retain SW compatibility.
 */
static u8 tcs3440_chip_id = 0x24;
static u8 tcs3440_chip_mask = 0xFC;

static operation_mode operating_mode;
static DECLARE_KFIFO(ams_kfifo, u8, PAGE_SIZE);
static int tcs3440_flush_regs(struct tcs3440_chip *chip);
static u16 max_adc_value, high_threshold, low_threshold;

static u8 const restorable_regs[] = {
	TCS3440_REGADDR_CONFIG,
	TCS3440_REGADDR_LED,
	TCS3440_REGADDR_ATIME,
	TCS3440_REGADDR_ASTEP1,
	TCS3440_REGADDR_ASTEP2,
	TCS3440_REGADDR_CFG1,
	TCS3440_REGADDR_CFG3,
	TCS3440_REGADDR_CFG8,
	TCS3440_REGADDR_WTIME,
	TCS3440_REGADDR_CFG10,
	TCS3440_REGADDR_PERS,
	TCS3440_REGADDR_GPIO2,
	TCS3440_REGADDR_AGC_GAIN_MAX,
	TCS3440_REGADDR_AZCONFIG,
	TCS3440_REGADDR_ENABLE
};


static int wait_for_bit_toggled(struct tcs3440_chip *chip, u8 reg_id, u8 bit)
{
	u8 reg;
	int count;

	count = 0;
	do {
		ams_tcs3440_i2c_read(chip->client, reg_id, &reg);
		udelay(1000);
		count++;
		if (count == POLLING_BIT_TIME_OUT)
			return -1;
	} while (reg & bit);

	return 0;
}

operation_mode get_spectral_mode(void)
{
	return operating_mode;
}

int set_spectral_mode(operation_mode mode)
{
	operating_mode = mode;
	return 0;
}

static void report_als_event(struct tcs3440_chip *chip)
{
	input_report_abs(chip->als_idev, ABS_X, chip->pdata->out_data[IDX_F1]);
	input_report_abs(chip->als_idev, ABS_Y, chip->pdata->out_data[IDX_F2]);
	input_report_abs(chip->als_idev, ABS_Z, chip->pdata->out_data[IDX_F3]);
	input_report_abs(chip->als_idev, ABS_RX, chip->pdata->out_data[IDX_F4]);
	input_report_abs(chip->als_idev, ABS_RY, chip->pdata->out_data[IDX_F5]);
	input_report_abs(chip->als_idev, ABS_RZ, chip->pdata->out_data[IDX_F6]);
	input_report_abs(chip->als_idev, ABS_RUDDER, chip->pdata->out_data[IDX_F7]);
	input_report_abs(chip->als_idev, ABS_WHEEL, chip->pdata->out_data[IDX_F8]);
	input_report_abs(chip->als_idev, ABS_GAS, chip->pdata->out_data[IDX_CLEAR]);
	input_report_abs(chip->als_idev, ABS_BRAKE, chip->pdata->out_data[IDX_NIR]);
	input_report_abs(chip->als_idev, ABS_HAT0X, chip->pdata->out_data[IDX_AGAIN]);
	input_report_abs(chip->als_idev, ABS_HAT0Y, chip->pdata->out_data[IDX_ATIME]);
	input_report_abs(chip->als_idev, ABS_HAT1X, chip->pdata->out_data[IDX_ASTEP]);

	if (chip->ams_output_enable) {
		input_report_abs(chip->als_idev, ABS_MISC, chip->xyz.lux);
		input_report_abs(chip->als_idev, ABS_BRAKE, chip->xyz.cct);
	}
	input_sync(chip->als_idev);
}

int ams_tcs3440_read_kfifo(struct tcs3440_chip *chip, char *buf)
{
	int ret = 0;
	/* sys_fs dev attr always PAGE_SIZE */
	dev_info(&chip->client->dev, "Reading FIFO data for SYS_FS: \n");
	ret = kfifo_out(&ams_kfifo, buf, PAGE_SIZE);
	kfifo_reset(&ams_kfifo);
	return ret;
}

static void enable_als(struct tcs3440_chip *chip, bool enable)
{
	static unsigned int en_count;

	if (!enable && !en_count) {
		dev_warn(&chip->client->dev, "Attempting to disable an already disabled sensor!\n");
		return;
	}

	if (enable) {
		if (en_count) {
			dev_dbg(&chip->client->dev, "Color sensor already enabled. No-op.\n");
			en_count++;
			return;
		}

		set_spectral_mode(TCS3440_ALS_MODE);
		ams_tcs3440_i2c_modify(chip->client, chip->shadow, TCS3440_REGADDR_INTENAB,
			TCS3440_INTENAB_AIEN, TCS3440_INTENAB_AIEN);
		ams_tcs3440_i2c_modify(chip->client, chip->shadow,
			TCS3440_REGADDR_ENABLE, (TCS3440_AEN | TCS3440_PON),
				(TCS3440_AEN | TCS3440_PON));
		en_count++;
	} else {
		if (en_count > 1) {
			dev_dbg(&chip->client->dev,
				"Do not disable color sensor. en_count is %u\n", en_count);
			en_count--;
			return;
		}

		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, 0x00);
		ams_tcs3440_i2c_modify(chip->client, chip->shadow,
			TCS3440_REGADDR_INTENAB, TCS3440_INTENAB_AIEN, 0x00);
		set_spectral_mode(TCS3440_MODE_IDLE);
		chip->xyz.lux = 0;
		chip->xyz.cct = 0;
		en_count--;
	}
	chip->enabled = enable;
}

/* Enable or disable the device - PON in REGADDR_ENABLE */
int tcs3440_enable_device(struct tcs3440_chip *chip, u8 state)
{
        if (state)
        {
             ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
        }
        else
        {
             ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, 0);
        }
        return 0;
}

static void tcs3440_enable_sai(struct tcs3440_chip *chip)
{
	if (chip->pdata->parameters.sai_enable == 1) {
		ams_tcs3440_i2c_modify(chip->client, chip->shadow,
			TCS3440_REGADDR_CFG3, TCS3440_SAI, TCS3440_SAI);
	}
}

static void tcs3440_disable_sai(struct tcs3440_chip *chip)
{
	u8 status6;

	if (chip->pdata->parameters.sai_enable == 1) {
		ams_tcs3440_i2c_modify(chip->client, chip->shadow,
			TCS3440_REGADDR_CFG3, TCS3440_SAI, 0x00);

		/* Check for SAI */
		ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_STATUS6, &status6);
		if (status6 & TCS3440_SAI_ACTIVE) {
			/* Clear SAI condition */
			ams_tcs3440_i2c_modify(chip->client, chip->shadow,
				TCS3440_REGADDR_CONTROL,
				TCS3440_CLEAR_SAI_ACT,
				TCS3440_CLEAR_SAI_ACT);
		}
	}
}

void smux_config(struct tcs3440_chip *chip, bool init)
{
	u8 smux_chain, map_value;

	/* int channels = 6; */ /* F1 .. F4 or F5..F8 and CLR, NIR */
	map_value = 0;
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_FIFO_MAP, map_value);

	if (init == true) {
		chip->is_first_smux_done = false;
		chip->mux_state = TCS3440_MUX_CFG_A_STATE;
	}

	/* Set the SMUX chain CFG6 register. Note - it is also used for AGC_GAIN_MAX (bit 6) */
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_CFG6, &smux_chain);
	if (smux_chain & TCS3440_AGC_GAIN_MAX)
		smux_chain |= TCS3440_SMUX;
	else
		smux_chain = TCS3440_SMUX;

	/* Write SMUX chain command */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_CFG6, smux_chain);

	/* Write SMUX configuration for F1..F4 first and then F5..F8 next */
	if (chip->is_first_smux_done == false) {
		/* Write SMUX configuration for F1,F2,F3,F4,CLEAR,NIR */
		chip->mux_state = TCS3440_MUX_CFG_A_STATE;
		ams_tcs3440_i2c_reg_blk_write(chip->client, TCS3440_REGADDR_RAM_START,
			(*(&smux_config_data))[ALS_SMUX_CFG_1], SMUX_SIZE);
		chip->is_first_smux_done = true;
	} else {
		/* Write SMUX configuration for F5,F6,F7,F8,CLEAR,NIR */
		chip->mux_state = TCS3440_MUX_CFG_B_STATE;
		ams_tcs3440_i2c_reg_blk_write(chip->client, TCS3440_REGADDR_RAM_START,
			(*(&smux_config_data))[ALS_SMUX_CFG_2], SMUX_SIZE);
		chip->is_first_smux_done = false;
	}

	if (chip->pdata->parameters.sai_enable == 1) {
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_MUX_ON);

		if (wait_for_bit_toggled(chip, TCS3440_REGADDR_ENABLE, TCS3440_MUXEN))
			dev_warn(&chip->client->dev, "%s: MUXEN bit has not toggled\n", __func__);
	} else {
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_MUX_ALS);
	}
}

static void tcs3440_process_data(struct tcs3440_chip *chip)
{
	struct adc_data *_adc_data = &chip->adc_result;
	struct als_xyz_data *xyz = &chip->xyz;

	_adc_data->f1	= chip->pdata->out_data[IDX_F1];
	_adc_data->f2	= chip->pdata->out_data[IDX_F2];
	_adc_data->f3	= chip->pdata->out_data[IDX_F3];
	_adc_data->f4	= chip->pdata->out_data[IDX_F4];
	_adc_data->f5	= chip->pdata->out_data[IDX_F5];
	_adc_data->f6	= chip->pdata->out_data[IDX_F6];
	_adc_data->f7	= chip->pdata->out_data[IDX_F7];
	_adc_data->f8	= chip->pdata->out_data[IDX_F8];
	_adc_data->clear = chip->pdata->out_data[IDX_CLEAR];
	_adc_data->nir	= chip->pdata->out_data[IDX_NIR];
	_adc_data->again = chip->pdata->out_data[IDX_AGAIN];
	_adc_data->atime = chip->pdata->out_data[IDX_ATIME];
	_adc_data->astep = chip->pdata->out_data[IDX_ASTEP];

	if (chip->ams_output_enable) {
		tcs3440_calculate_lux_and_cct(chip, _adc_data, NULL, xyz);
	}
	return;
}

#define RAW_DATA_LENGTH 13

static void tcs3440_als_gain_update(struct tcs3440_chip *chip, uint32_t again) {
	u8 status;

	chip->shadow[TCS3440_REGADDR_CFG1] = again;
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_CFG6, &status);
	ams_tcs3440_i2c_write(chip->client, chip->shadow,
		TCS3440_REGADDR_CFG6, (status & (~TCS3440_AGC_GAIN_MAX)));
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_CFG8, &status);
	ams_tcs3440_i2c_write(chip->client, chip->shadow,
		TCS3440_REGADDR_CFG8, (status & (~TCS3440_AGC_ENABLE)));
	dev_warn(&chip->client->dev, "%s: Updating again to %u\n", __func__, again);
	tcs3440_flush_regs(chip);
}

/* Called from interrupt handler - put into kfifo */
static int tcs3440_read_data(struct tcs3440_chip *chip)
{
	/* struct device *dev = &chip->client->dev; */
	int ret;
	u8 data[RAW_DATA_LENGTH];
	u16 temp_value, max_value = 0;
	int i, j;
	u16 *data_ptr;
	int fifo_length;
	u8 atime_data;
	u16 astep_data;
	static u8 again;

	/* Read ASTATUS through ADATA5, 13 bytes */
	ret = ams_tcs3440_i2c_blk_read(chip->client, TCS3440_REGADDR_ASTATUS, data, sizeof(data));
	if (ret > 0) {
		data_ptr = (u16 *)&data[1];
		if (chip->mux_state == TCS3440_MUX_CFG_A_STATE) {
			chip->is_spectral_ready = false;

			/* Skip ASTATUS data in index 0
			 * Copy F1, F2, F3, F4
			 */
			for (i = 1, j = 0; i < 9; i += 2, j++) {
				temp_value = ((data[i+1] << 8) | data[i]);
				chip->pdata->out_data[j] = temp_value;
			}

			/* Copy CLEAR */
			temp_value = ((data[10] << 8) | data[9]);
			chip->pdata->out_data[IDX_CLEAR] = temp_value;

			/* Copy NIR */
			temp_value = ((data[12] << 8) | data[11]);
			chip->pdata->out_data[IDX_NIR] = temp_value;
		} else {
			/* Skip ASTATUS data in index 0
			 * Copy F5, F6, F7, F8
			 */
			for (i = 1, j = 4; i < 9; i += 2, j++) {
				temp_value = ((data[i+1] << 8) | data[i]);
				chip->pdata->out_data[j] = temp_value;
			}

			/* Record AGAIN */
			chip->pdata->out_data[IDX_AGAIN] = data[0] & TCS3440_MASK_AGC_AGAIN;
			again = chip->pdata->out_data[IDX_AGAIN];

			/* Record ATIME */
			ret = ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ATIME, &atime_data);
			chip->pdata->out_data[IDX_ATIME] = atime_data;

			/* Record ASTEP */
			ret = ams_tcs3440_i2c_blk_read(chip->client,
				TCS3440_REGADDR_ASTEP1, (u8 *)&astep_data, 2);
			chip->pdata->out_data[IDX_ASTEP] = le16_to_cpu(astep_data);
			chip->is_spectral_ready = true;
		}

		if (chip->is_spectral_ready == true) {
			kfifo_in(&ams_kfifo, (u8 *)chip->pdata->out_data, 24);
			fifo_length = kfifo_len(&ams_kfifo);
			memset(chip->raw_data, 0x00, sizeof(chip->raw_data));
			for (i = 0; i <= IDX_ASTEP; i++) {
				chip->raw_data[i] = chip->pdata->out_data[i];
				if (i < IDX_AGAIN) {
					max_value = max(max_value, chip->raw_data[i]);
				}
			}
			tcs3440_process_data(chip);
			report_als_event(chip);
			memset(chip->pdata->out_data, 0x00, sizeof(chip->pdata->out_data));

			if (max_value >= high_threshold) {
				if (again > 0 && (again <= (chip->params.again_max & 0x0F))) {
					again -= 1;
					tcs3440_als_gain_update(chip, again);
				}
			} else if (max_value <= low_threshold) {
				if (again < (chip->params.again_max & 0x0F)) {
					again += 1;
					tcs3440_als_gain_update(chip, again);
				}
			}
		}
	}

	if (chip->pdata->parameters.sai_enable == 1) {
		/* Disable the sensor */
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, 0x00);
		/* Confirm that AEN is off before continuing*/
		if (wait_for_bit_toggled(chip, TCS3440_REGADDR_ENABLE, TCS3440_AEN))
			dev_warn(&chip->client->dev, "%s: AEN was not toggled/reset after timeout\n", __func__);

		tcs3440_disable_sai(chip);
		
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
	}

	/* Swap the smux configuration */
	smux_config(chip, false);

	if (chip->pdata->parameters.sai_enable == 1) {
		/* Enable the sensor again */
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, 0x00);
		tcs3440_enable_sai(chip);
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON | TCS3440_AEN);
	}

	return ret;
}

static int tcs3440_irq_handler(struct tcs3440_chip *chip)
{
	u8 status, status2, status6;
	u8 astatus;
	int ret;
	/* struct device *dev = &chip->client->dev; */

	AMS_MUTEX_LOCK(&chip->lock);
	ret = ams_tcs3440_i2c_read(chip->client,
			TCS3440_REGADDR_STATUS, &status);
	/* dev_info(dev, " irq handler status = %02x \n", status); */

	if (status == 0) {
		AMS_MUTEX_UNLOCK(&chip->lock);
		return 0; /* not our interrupt */
	}

	do {
		ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_STATUS2, &status2);
		ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_STATUS6, &status6);
		ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ASTATUS, &astatus);

		/* Clear the interrupt */
		ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_STATUS, status);

		/* Check for SAI */
		if (chip->pdata->parameters.sai_enable == 1 && (status6 & TCS3440_SAI_ACTIVE)) {
			/* Clear SAI condition */
			ams_tcs3440_i2c_modify(chip->client, chip->shadow, TCS3440_REGADDR_CONTROL,
				TCS3440_CLEAR_SAI_ACT, TCS3440_CLEAR_SAI_ACT);
			//dev_dbg(&chip->client->dev, "clear SAI status\n");
		}

		if (status & TCS3440_ASAT) {
			if (status2 & (TCS3440_ANA_SAT | TCS3440_DIG_SAT)) {
				chip->in_asat = 1;
				dev_warn(&chip->client->dev,
					"Saturation, ASAT is %d STATUS2 = %x \n",
					chip->in_asat, status2);
				chip->is_als_valid = 0;
			}
		} else {
			chip->in_asat = 0;
			chip->is_als_valid = 1;
		}

		/*
		 * Calibration
		 */
		if (status & TCS3440_CINT) {
			chip->amscalcomplete = true;
		}

		/* ALS interrupt */
		if ((status & TCS3440_AINT) && (status2 & TCS3440_AVALID)) {
			ret = tcs3440_read_data(chip);
			if (chip->is_spectral_ready == true) {
				wake_up_interruptible(&chip->fifo_wait);
			}
		}

		ret = ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_STATUS,
				&status);
	} while (status != 0);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return 1; /* we handled the interrupt */
}

static irqreturn_t tcs3440_irq(int irq, void *handle)
{
	struct tcs3440_chip *chip = handle;
	struct device *dev = &chip->client->dev;
	int ret = 0;

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		goto bypass;
	}
	ret = tcs3440_irq_handler(chip);

bypass:
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

static int tcs3440_flush_regs(struct tcs3440_chip *chip)
{
	int i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_tcs3440_i2c_write(chip->client,
			chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
				__func__, reg);
			break;
		}
	}

	return rc;
}

static int tcs3440_pltf_power_on(struct tcs3440_chip *chip)
{
        int rc = 0;

        if (chip->pdata->platform_power) {
                rc = chip->pdata->platform_power(chip, POWER_ON);
                mdelay(10);
        }
        chip->unpowered = rc != 0;
        dev_info(&chip->client->dev, "%s: unpowered=%d\n", __func__,
                        chip->unpowered);
        return rc;
}

static int tcs3440_pltf_power_off(struct tcs3440_chip *chip)
{
        int rc = 0;

        if (chip->pdata->platform_power) {
                rc = chip->pdata->platform_power(chip, POWER_OFF);
                chip->unpowered = rc == 0;
        } else {
                chip->unpowered = false;
        }
        dev_info(&chip->client->dev, "%s: unpowered=%d\n", __func__,
                        chip->unpowered);
        return rc;
}

static void tcs3440_set_defaults(struct tcs3440_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;
	struct tcs3440_parameters *of_data = &chip->pdata->parameters;
	u16 astep;
	u8 cfg3_reg_val;

	/* Clear the register shadow area */
	memset(chip->shadow, 0x00, sizeof(chip->shadow));

	dev_info(dev, "%s: use of_data\n", __func__);
	/*Note - if it is a 0 in all bits in the data sheet - omit */
	chip->params.enable = of_data->enable;	/* PON -also enable Spectral? - 0x03 ? */
	chip->params.config = of_data->config;	/* No mode */
	chip->params.led_reg = of_data->led_reg;
	chip->params.atime = of_data->atime;
	chip->params.ms_astep = of_data->ms_astep;
	chip->params.ls_astep = of_data->ls_astep;
	chip->params.wtime = of_data->wtime;
	chip->params.again = of_data->again;	/* default in data sheet */
	chip->params.cfg8 = of_data->cfg8;	/* auto-gain */
	chip->params.cfg10 = of_data->cfg10;
	chip->params.persist = ALS_PERSIST(of_data->persist);
	chip->params.gpio2 = of_data->gpio2;
	chip->params.again_max = of_data->again_max;
	chip->params.azconfig = of_data->azconfig;
	chip->params.sai_enable = of_data->sai_enable;
	chip->is_spectral_ready = false;

	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_CFG3, &cfg3_reg_val);
	if (chip->params.sai_enable == 1) {
		dev_info(dev, "%s: SAI enabled\n", __func__);
		chip->params.cfg3 = cfg3_reg_val | TCS3440_SAI;
	}

	/* Copy the default values into the register shadow area */
	sh[TCS3440_REGADDR_ENABLE] = chip->params.enable;
	sh[TCS3440_REGADDR_CONFIG] = chip->params.config;
	sh[TCS3440_REGADDR_LED] = chip->params.led_reg;
	sh[TCS3440_REGADDR_ATIME] = chip->params.atime;
	sh[TCS3440_REGADDR_ASTEP1] = chip->params.ls_astep;
	sh[TCS3440_REGADDR_ASTEP2] = chip->params.ms_astep;
	sh[TCS3440_REGADDR_WTIME] = chip->params.wtime;
	sh[TCS3440_REGADDR_CFG1] = chip->params.again;
	sh[TCS3440_REGADDR_CFG3] = chip->params.cfg3;
	sh[TCS3440_REGADDR_CFG8] = chip->params.cfg8;
	sh[TCS3440_REGADDR_CFG10] = chip->params.cfg10;
	sh[TCS3440_REGADDR_PERS] = chip->params.persist;
	sh[TCS3440_REGADDR_GPIO2] = chip->params.gpio2;
	sh[TCS3440_REGADDR_AGC_GAIN_MAX] = chip->params.again_max;
	sh[TCS3440_REGADDR_AZCONFIG] = chip->params.azconfig;

	tcs3440_flush_regs(chip);

	astep = (chip->params.ms_astep << 8) | (chip->params.ls_astep);
	max_adc_value = min((chip->params.atime + 1) * (astep + 1), 65535); // 65535 = 0xFFFF (16 bit-channel max)
	high_threshold = (max_adc_value / 8 * 7) + ((max_adc_value % 8 * 7) / 8); //rounddown(value*0.875)
	low_threshold = (max_adc_value / 8 * 3) + ((max_adc_value % 8 * 3) / 8); //rounddown(value*0.375)
}

static int tcs3440_get_id(struct tcs3440_chip *chip, u8 *id, u8 *rev, u8 *auxid)
{
        ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_AUXID, auxid);
        ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_REVID, rev);
        ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ID, id);

        return 0;
}

static void config_als(struct tcs3440_chip *chip)
{
	operation_mode mode = get_spectral_mode();

	chip->mode = mode;

	chip->is_spectral_ready = false;
	chip->is_data_read = false;

	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_FIFO_MAP, 0);
#if AGC_ENABLE
	/* AGC enable */
	ams_tcs3440_i2c_modify(chip->client, chip->shadow,
		TCS3440_REGADDR_CFG6, TCS3440_AGC_GAIN_MAX, TCS3440_AGC_GAIN_MAX);

	ams_tcs3440_i2c_modify(chip->client, chip->shadow,
		TCS3440_REGADDR_CFG8, TCS3440_AGC_ENABLE, TCS3440_AGC_ENABLE);
#endif
	/* Turn only PON on and write smux config */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
	smux_config(chip, true);

	kfifo_reset(&ams_kfifo);
}

static int tcs3440_power_on(struct tcs3440_chip *chip)
{
        int rc;

        rc = tcs3440_pltf_power_on(chip);
        if (rc)
                return rc;
        dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
                        __func__);
        return tcs3440_flush_regs(chip);
}

static int tcs3440_als_idev_open(struct input_dev *idev)
{
	struct tcs3440_chip *chip = dev_get_drvdata(&idev->dev);
	int rc = 0;
	u8 status;

	dev_info(&idev->dev, "%s\n", __func__);

	if (chip->enabled == false) {
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->unpowered) {
			rc = tcs3440_power_on(chip);
			if (rc)
				goto chip_on_err;
		}

		ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ENABLE, &status);
		dev_info(&idev->dev, "als_idev_open Enable = %02x\n", status);
		set_spectral_mode(TCS3440_ALS_MODE);
		config_als(chip);
		enable_als(chip, 1);

chip_on_err:
		AMS_MUTEX_UNLOCK(&chip->lock);
	}

	return 0;
}

static void tcs3440_als_idev_close(struct input_dev *idev)
{
        struct tcs3440_chip *chip = dev_get_drvdata(&idev->dev);
        u8 status;

        dev_info(&idev->dev, "%s\n", __func__);
        AMS_MUTEX_LOCK(&chip->lock);

        enable_als(chip, 0);

        ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ENABLE, &status);
        dev_info(&idev->dev, "als_idev_close Enable = %02x\n", status);
        tcs3440_pltf_power_off(chip);
        AMS_MUTEX_UNLOCK(&chip->lock);
}

#ifdef CONFIG_QUALCOMM_AP
static int tcs3440_als_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	struct tcs3440_chip *chip = container_of(sensors_cdev, struct tcs3440_chip, als_cdev);

	if (enable) {
		chip->a_idev->open(chip->a_idev);
	} else {
		chip->a_idev->close(chip->a_idev);
	}
	return 0;
}
#endif

#ifdef CONFIG_OF
int tcs3440_init_dt(struct tcs3440_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	u32 val;
	const char *str;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "als_name", &str))
		pdata->als_name = str;

	if (!of_property_read_u32(np, "als_enable", &val))
		pdata->parameters.enable = val;

	if (!of_property_read_u32(np, "als_led_reg", &val))
		pdata->parameters.led_reg = val;

	if (!of_property_read_u32(np, "als_atime", &val))
		pdata->parameters.atime = val;

	if (!of_property_read_u32(np, "als_ms_astep", &val))
		pdata->parameters.ms_astep = val;

	if (!of_property_read_u32(np, "als_ls_astep", &val))
		pdata->parameters.ls_astep = val;

	if (!of_property_read_u32(np, "als_wtime", &val))
		pdata->parameters.wtime = val;

	if (!of_property_read_u32(np, "als_again", &val))
		pdata->parameters.again = val;

	if (!of_property_read_u32(np, "als_cfg8", &val))
		pdata->parameters.cfg8 = val;

	if (!of_property_read_u32(np, "als_cfg10", &val))
		pdata->parameters.cfg10 = val;

	if (!of_property_read_u32(np, "als_persist", &val))
		pdata->parameters.persist = val;

	if (!of_property_read_u32(np, "als_gpio2", &val))
		pdata->parameters.gpio2 = val;

	if (!of_property_read_u32(np, "als_again_max", &val))
		pdata->parameters.again_max = val;

	if (!of_property_read_u32(np, "als_azconfig", &val))
		pdata->parameters.azconfig = val;

	if (!of_property_read_u32(np, "als_sai_enable", &val))
                pdata->parameters.sai_enable = val;
        return 0;
}
#endif

u8 get_wait_time(struct tcs3440_chip *chip)
{
	u8 wtime;
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_WTIME, &wtime);
	/* Rounding 2.78 to 3 */
	return (3 * (wtime + 1));
}

int set_idle_mode(struct tcs3440_chip *chip)
{
	/* Disable the interrupts */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_INTENAB, 0);
	chip->params.intenab = 0;
	/* Set only PON */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
	return 0;
}

int spectral_mode(struct tcs3440_chip *chip)
{
	/* u8 status; */
	struct device *dev = &chip->client->dev;

	operation_mode mode = get_spectral_mode();
	/* If input device is not already open - open to handle events */
	if (chip->als_input_open == false) {
		chip->als_idev->open(chip->als_idev);
		chip->als_input_open = true;
	}
	AMS_MUTEX_LOCK(&chip->lock);
	chip->mode = mode;
	chip->pdata->pos = 0;
	/* Setting the Flicker data not sent to FIFO */
	chip->params.fd_cfg0 = 0x21; /* Not sure about bit 7 for this register */
	/* ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_FD_CFG0, chip->params.fd_cfg0); */

	chip->params.intenab = TCS3440_INTENAB_AIEN;
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_INTENAB, TCS3440_INTENAB_AIEN);
	/* ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ENABLE, &status); */
	chip->is_spectral_ready = false;
	chip->is_data_read = false;
	dev_info(dev, "Start spectral one shot\n");

	/* Turn only PON on */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
	smux_config(chip, true);
	kfifo_reset(&ams_kfifo);
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON | TCS3440_AEN);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

int stop_spectral_measurement(struct tcs3440_chip *chip)
{
	struct device *dev = &chip->client->dev;
	set_spectral_mode(false);
	AMS_MUTEX_LOCK(&chip->lock);
	/* Turn off Spectral measure - leave chip on  */
	ams_tcs3440_i2c_write_direct(chip->client, TCS3440_REGADDR_ENABLE, TCS3440_PON);
	AMS_MUTEX_UNLOCK(&chip->lock);
	dev_info(dev, "Spectral measurement stopped");
	return 0;
}

/*
 * Sysfs ABI
 */

static ssize_t tcs3440_cal_level_h_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n",
				chip->cal_level &
				~TCS3440_CAL_STATUS_ALL_MASK_H);
}

static ssize_t tcs3440_cal_level_h_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3440 change str to int fail\n");
		return size;
	}

	if ((chip->cal_level & ~TCS3440_CAL_STATUS_ALL_MASK_H) != value) {
		dev_dbg(dev, "tcs3440 enable sensor for calibration\n");
		chip->cal_level = value |
				(chip->cal_level &
				TCS3440_CAL_STATUS_ALL_MASK_H);
		switch (value) {
		case 3000:
			chip->cal_cct_status = -1;
			input_report_abs(chip->als_idev, ABS_THROTTLE, 3000);
			input_sync(chip->als_idev);
			dev_dbg(dev, "tcs3440 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
				~TCS3440_CAL_STATUS_CCT_3000_MASK_H;
			break;
		case 5000:
			chip->cal_cct_status = -1;
			input_report_abs(chip->als_idev, ABS_THROTTLE, 5000);
			input_sync(chip->als_idev);
			dev_dbg(dev, "tcs3440 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
				~TCS3440_CAL_STATUS_CCT_5000_MASK_H;
			break;
		case 4001:
			chip->cal_cct_status = -1;
			input_report_abs(chip->als_idev, ABS_THROTTLE, 4001);
			input_sync(chip->als_idev);
			dev_dbg(dev, "tcs3440 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
				~TCS3440_VALITATION_HIGH_IR_MASK_H;
			break;
		case 4002:
			chip->cal_cct_status = -1;
			input_report_abs(chip->als_idev, ABS_THROTTLE, 4002);
			input_sync(chip->als_idev);
			dev_dbg(dev, "tcs3440 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
				~TCS3440_VALITATION_LOW_IR_MASK_H;
			break;
        	case 1000:
			chip->cal_lux_status = -1;
			input_report_abs(chip->als_idev, ABS_THROTTLE, 1000);
			input_sync(chip->als_idev);
			dev_dbg(dev, "tcs3440 cal: %dLux is starting\n", value);
			chip->cal_level = chip->cal_level &
				~TCS3440_CAL_STATUS_LUX_1000_MASK_H;
			break;
		default:
			dev_err(dev, "tcs3440 cal: %d(K/Lux) is Non-existent\n",
				value);
		}
	} else {
        	dev_err(dev, "tcs3440 cal: %d(K/Lux) is running or Non-existent.\n",
			value);
    	}
	return size;
}

static ssize_t tcs3440_cal_cct_status_h_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->cal_cct_status);
}

static ssize_t tcs3440_cal_cct_status_h_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3440 change str to int fail\n");
		return size;
	}

	chip->cal_cct_status = value;

	dev_dbg(dev, " tcs3440 cal(%dK) result: %d\n",
			chip->cal_level & ~TCS3440_CAL_STATUS_ALL_MASK_H,
			chip->cal_cct_status);
	switch (chip->cal_level & ~TCS3440_CAL_STATUS_ALL_MASK_H) {
	case 3000:
		chip->cal_level = (chip->cal_level &
					TCS3440_CAL_STATUS_ALL_MASK_H)|
					TCS3440_CAL_STATUS_CCT_3000_MASK_H;
		break;
	case 5000:
		chip->cal_level = (chip->cal_level &
					TCS3440_CAL_STATUS_ALL_MASK_H)|
					TCS3440_CAL_STATUS_CCT_5000_MASK_H;
		break;
	default:
		break;
	}
	return size;
}

static ssize_t tcs3440_als_rawdata_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count = 0;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "f1:%d,f2:%d,f3:%d,f4:%d,f5:%d,f6:%d,f7:%d,f8:%d,clear:%d,nir:%d,again:%d,atime:%d,astep:%d\n",
                chip->raw_data[0],chip->raw_data[1],chip->raw_data[2],chip->raw_data[3],chip->raw_data[4],chip->raw_data[5],chip->raw_data[6],
                chip->raw_data[7],chip->raw_data[8],chip->raw_data[9],chip->raw_data[10],chip->raw_data[11],chip->raw_data[12]);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_cal_status_h_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->cal_level &
		TCS3440_CAL_STATUS_ALL_MASK_H);
}

static ssize_t tcs3440_cal_status_h_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3440 change str to int fail\n");
		return size;
	}

	if (value == 1) {
		input_report_abs(chip->als_idev, ABS_THROTTLE, 1);
		input_sync(chip->als_idev);
		chip->cal_cct_status = 0;
		chip->cal_lux_status = 0;
		chip->cal_level = 0;
		dev_dbg(dev, "tcs3440 clear&reset cal_status\n");
	}
	return size;
}

static ssize_t tcs3440_enable_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;
	u8 status;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ENABLE, &status);
	chip->params.enable = status;

	count =  snprintf(buf, PAGE_SIZE, "%x\n", chip->enabled);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int rc;
	bool enable;

	rc = kstrtobool(buf, &enable);
	if (rc != 0) {
		dev_err(&chip->client->dev, "kstrtobool() error.\n");
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);
	enable_als(chip, enable);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3440_als_atime_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;
	u8 value;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ATIME, &value);
	count =  snprintf(buf, PAGE_SIZE, "%x\n", value);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_atime_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int rc;
	unsigned int value;

	rc = kstrtouint(buf, 0, &(value));
	if (rc != 0) {
		dev_err(&chip->client->dev, "kstrtol() error.\n");
		return -EINVAL;
	}
	if (value > 255)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->shadow[TCS3440_REGADDR_ATIME] = value;
	tcs3440_flush_regs(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3440_als_astep_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count,ret;
	u16 astep_data,value;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = ams_tcs3440_i2c_blk_read(chip->client, TCS3440_REGADDR_ASTEP1, (u8 *)&astep_data, 2);
	value = le16_to_cpu(astep_data);
	count = snprintf(buf, PAGE_SIZE, "%d\n", value);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_astep_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int rc;
	unsigned int value;

	rc = kstrtouint(buf, 0, &(value));
	if (rc != 0) {
		dev_err(&chip->client->dev, "kstrtol() error.\n");
		return -EINVAL;
	}
	if (value > 65535)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->shadow[TCS3440_REGADDR_ASTEP1] = (value & 0x00FF);
	chip->shadow[TCS3440_REGADDR_ASTEP2] = (value >> 8);
	tcs3440_flush_regs(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3440_als_again_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;
	u8 status,value;

	AMS_MUTEX_LOCK(&chip->lock);
	    ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ASTATUS, &status);
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_CFG1, &value);
	count =  snprintf(buf, PAGE_SIZE, "astatus:%d, again:%d\n", status, value);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_again_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t size)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int rc;
	uint32_t again;

	rc = kstrtouint(buf, 10, &again);
	if (rc != 0) {
		dev_err(&chip->client->dev, "kstrtouint() error.\n");
		return -EINVAL;
	}

	if (again > 9) {
		dev_err(&chip->client->dev, "again over-range.\n");
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);
	tcs3440_als_gain_update(chip, again);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3440_als_id_show(struct device *dev,
                       struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->id);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_auxid_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->auxid);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_revid_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->rev);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_lux_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->xyz.lux);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static ssize_t tcs3440_als_cct_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);
	int count;

	AMS_MUTEX_LOCK(&chip->lock);
	count =  snprintf(buf, PAGE_SIZE, "%d\n", chip->xyz.cct);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

struct device_attribute tcs3440_als_attrs[] = {
        __ATTR(enable,               0660, tcs3440_enable_show,
                tcs3440_enable_store),
        __ATTR(cal_cct_status_h,     0660, tcs3440_cal_cct_status_h_show,
                tcs3440_cal_cct_status_h_store),
        __ATTR(cal_level_h,          0660, tcs3440_cal_level_h_show,
                tcs3440_cal_level_h_store),
        __ATTR(als_atime,            0660, tcs3440_als_atime_show,
                tcs3440_als_atime_store),
        __ATTR(als_astep,            0660, tcs3440_als_astep_show,
                tcs3440_als_astep_store),
        __ATTR(als_again,            0660, tcs3440_als_again_show,
                tcs3440_als_again_store),
        __ATTR(cal_status_h,         0660, tcs3440_cal_status_h_show,
                tcs3440_cal_status_h_store),
        __ATTR(als_auxid,            0440, tcs3440_als_auxid_show,
                NULL),
        __ATTR(als_lux,              0440, tcs3440_als_lux_show,
                NULL),
        __ATTR(als_revid,            0440, tcs3440_als_revid_show,
                NULL),
        __ATTR(als_id,               0440, tcs3440_als_id_show,
                NULL),
        __ATTR(als_cct,              0440, tcs3440_als_cct_show,
                NULL),
        __ATTR(als_rawdata,          0440, tcs3440_als_rawdata_show,
                NULL)

};

int tcs3440_als_attrs_size = ARRAY_SIZE(tcs3440_als_attrs);

#ifdef CONFIG_OF
static const struct of_device_id tcs3440_of_match[] = {
	{.compatible = "ams,tcs34x0"},
	{}
};
MODULE_DEVICE_TABLE(of, tcs3440_of_match);
#endif

int tcs3440_power_init(struct tcs3440_chip *chip) {
	int ret = 0;
	chip->vdd = regulator_get(&chip->client->dev, "vdd");

	if (IS_ERR(chip->vdd)) {
		ret = PTR_ERR(chip->vdd);
		dev_err(&chip->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(chip->vdd) > 0) {
		ret = regulator_set_voltage(chip->vdd,
				1800000,
				1800000);
		if (ret) {
			dev_err(&chip->client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}
	return 0;

reg_vdd_put:
	regulator_put(chip->vdd);
	return ret;
}

int tcs3440_power_enable(struct tcs3440_chip *chip, enum tcs3440_pwr_state state) {

	int ret = 0;
	if (state == POWER_ON) {
		ret = regulator_enable(chip->vdd);
		dev_err(&chip->client->dev,
				"Regulator powered on ret=%d\n",
				ret);
	} else {
		ret = regulator_disable(chip->vdd);
                dev_err(&chip->client->dev,
				"Regulator powered off ret=%d\n",
				ret);
	}
	return ret;
}

static int tcs3440_pinctrl_init(struct device *dev)
{
	struct pinctrl_state *active;
	int rc;
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get(dev);
	active = pinctrl_lookup_state(pinctrl, "tcs34x0_active");
	rc = pinctrl_select_state(pinctrl, active);
	if (rc)
		dev_err(dev, "tcs3440 failed to set pin state, rc=%d\n", rc);
	return rc;
}

static int tcs3440_add_sysfs_interfaces(struct device *dev,
                                struct device_attribute *a,
                                int size)
{
	int i;

	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;

undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tcs3440_remove_sysfs_interfaces(struct device *dev,
                                        struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int tcs3440_probe(struct i2c_client *client,
                         const struct i2c_device_id *idp) {
	int ret, i;
	u8 id, rev, auxid;
	struct device *dev = &client->dev;
	static struct tcs3440_chip *chip;
	struct tcs3440_i2c_platform_data *pdata = dev->platform_data;
	unsigned long default_irq_trigger = 0;
	u8 status;

	bool powered = 0;

	pr_info("\nTCS3440: probe()\n");
	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);

#ifdef CONFIG_OF
	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(struct tcs3440_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		if (of_match_device(tcs3440_of_match, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tcs3440_init_dt(pdata);
			if (ret)
				return ret;
		}
	}
#endif

	/*
	* Validate bus and device registration
	*/

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	dev_info(dev, "%s: als_name = %s\n", __func__, pdata->als_name);
	if (!i2c_check_functionality(client->adapter,
				 I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (!(pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	pdata->platform_init = tcs3440_power_init;
	pdata->platform_power = tcs3440_power_enable;

	chip = devm_kzalloc(dev, sizeof(struct tcs3440_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->driver_remove = false;
	chip->pdata = pdata;
	chip->enabled = false;
	chip->cal_cct_status = 0;
	chip->cal_lux_status = 0;
	chip->cal_level = 0;

	i2c_set_clientdata(client, chip);
	tcs3440_pinctrl_init(dev);

	if (pdata->platform_init) {
		ret = pdata->platform_init(chip);
		if (ret)
			goto pon_failed;
	}

	if (pdata->platform_power) {
		ret = pdata->platform_power(chip, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}

	/*
	* Validate the appropriate ams device is available for this driver
	*/
	ret = tcs3440_get_id(chip, &id, &rev, &auxid);
	dev_info(dev, "%s: device id:%02x device aux id:%02x device rev:%02x\n",
		__func__, id, auxid, rev);

	for (i = 0; i < ARRAY_SIZE(tcs3440_ids); i++) {
		if (auxid == tcs3440_ids[i] &&
			(id & tcs3440_chip_mask) == tcs3440_chip_id) {
			chip->valid_auxid = true;
			break;
		}
	}

	if (chip->valid_auxid == true) {
		chip->id = id;
		chip->rev = rev;
		chip->auxid = auxid;
	} else {
		dev_err(dev, "%s: not supported chip id: 0x%x\n", __func__,
				id);
		ret = -ENODEV;
		goto id_failed;
	}

	/*
	 * Initialize ALS
	 */
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->als_idev = devm_input_allocate_device(dev);

	if (!chip->als_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n", __func__,
			pdata->als_name);
		ret = -ENODEV;
		goto id_failed;
	}

	chip->als_idev->name = pdata->als_name;
	chip->als_idev->id.bustype = BUS_I2C;

	set_bit(ABS_X, chip->als_idev->evbit);
	set_bit(ABS_Y, chip->als_idev->absbit);
	set_bit(ABS_Z, chip->als_idev->absbit);
	set_bit(ABS_RX, chip->als_idev->absbit);
	set_bit(ABS_RY, chip->als_idev->absbit);
	set_bit(ABS_RZ, chip->als_idev->absbit);
	set_bit(ABS_RUDDER, chip->als_idev->absbit);
	set_bit(ABS_WHEEL, chip->als_idev->absbit);
	set_bit(ABS_GAS, chip->als_idev->absbit);
	set_bit(ABS_BRAKE, chip->als_idev->absbit);
	set_bit(ABS_HAT0X, chip->als_idev->absbit);
	set_bit(ABS_HAT0Y, chip->als_idev->absbit);
	set_bit(ABS_HAT1X, chip->als_idev->absbit);
	set_bit(ABS_THROTTLE, chip->als_idev->absbit);
	set_bit(ABS_MISC, chip->als_idev->absbit);
	set_bit(ABS_BRAKE, chip->als_idev->absbit);
	set_bit(FIFO_DEPTH_EVENT, chip->als_idev->absbit);

	input_set_abs_params(chip->als_idev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_Y, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_Z, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_RX, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_RY, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_RZ, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_RUDDER, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_WHEEL, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_GAS, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_BRAKE, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_HAT0X, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_HAT0Y, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_HAT1X, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_THROTTLE, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_MISC, 0, 65535, 0, 0);
	input_set_abs_params(chip->als_idev, ABS_BRAKE, 0, 65535, 0, 0);

	chip->als_idev->open = tcs3440_als_idev_open;
	chip->als_idev->close = tcs3440_als_idev_close;
	input_set_drvdata(chip->als_idev, chip);
	chip->params.wtime = get_wait_time(chip);
	init_waitqueue_head(&chip->fifo_wait);

	/*
	* Set chip defaults
	*/
	tcs3440_set_defaults(chip);
	ret = tcs3440_flush_regs(chip);
	if (ret)
		goto input_a_alloc_failed;

	ret = input_register_device(chip->als_idev);
	if (ret) {
		input_free_device(chip->als_idev);
		dev_err(dev, "%s: cant register input '%s'\n", __func__,
			pdata->als_name);
		goto input_a_alloc_failed;
	}

	chip->als_input_open = false;
	/* If both Spectral enable and Wait Enable are set - use the thread */
	ams_tcs3440_i2c_read(chip->client, TCS3440_REGADDR_ENABLE, &status);
	dev_info(dev, "%s: After read Enable 2 = %02x \n", __func__, status);

	ret = tcs3440_add_sysfs_interfaces(&chip->als_idev->dev,
			tcs3440_als_attrs, tcs3440_als_attrs_size);
	if (ret)
		goto input_a_sysfs_failed;

#ifdef CONFIG_QUALCOMM_AP
	chip->als_cdev = als_sensors_cdev;
	chip->als_cdev.sensors_enable = tcs3440_als_set_enable;
	if (sensors_classdev_register(&chip->a_idev->dev, &chip->als_cdev)) {
		dev_err(dev, "sensors class register failed.\n");
	}
#endif

bypass_als_idev:
	/* Initialize IRQ & Handler */

	default_irq_trigger = irqd_get_trigger_type(irq_get_irq_data(client->irq));
	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, &tcs3440_irq,
					default_irq_trigger |
					IRQF_SHARED         |
					IRQF_ONESHOT,
					dev_name(dev), chip);
	if (ret) {
		dev_err(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}
	INIT_KFIFO(ams_kfifo);

	dev_info(dev, "Probe ok.\n");
	return 0;

	/*
	 * This must be unwound in the correct order, reverse
	 * from initialization above
	 */
irq_register_fail:
	devm_free_irq(&client->dev, client->irq, chip);
input_a_sysfs_failed:
	if (chip->als_idev)
		tcs3440_remove_sysfs_interfaces(&chip->als_idev->dev,
			tcs3440_als_attrs, tcs3440_als_attrs_size);
input_a_alloc_failed:
	if (chip->als_idev)
		input_unregister_device(chip->als_idev);
id_failed:
	i2c_set_clientdata(client, NULL);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(chip, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	if (!pdata)
		devm_kfree(dev, pdata);
	if (!chip)
		devm_kfree(dev, chip);
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tcs3440_suspend(struct device *dev)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "TCS3440: suspend()\n");
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tcs3440_pltf_power_off(chip);
	}

	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

static int tcs3440_resume(struct device *dev)
{
	struct tcs3440_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "TCS3440: resume()\n");
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 0;

	dev_info(dev, "%s: powerd %d, enabled %d", __func__,
			!chip->unpowered, chip->enabled);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}

	/* err_power: */
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs3440_remove(struct i2c_client *client)
{
	struct tcs3440_chip *chip = i2c_get_clientdata(client);

	chip->driver_remove = true;
	dev_info(&client->dev, "%s\n", __func__);
	devm_free_irq(&client->dev, client->irq, chip);
	tcs3440_enable_device(chip, 0);
	tcs3440_pltf_power_off(chip);

	if (chip->als_idev) {
		tcs3440_remove_sysfs_interfaces(&chip->als_idev->dev,
                            tcs3440_als_attrs, tcs3440_als_attrs_size);
		input_unregister_device(chip->als_idev);
	}

	if (chip->als_input_open == true) {
		chip->als_idev->close(chip->als_idev);
		chip->als_input_open = false;
	}

#ifdef CONFIG_QUALCOMM_AP
	sensors_classdev_unregister(&chip->als_cdev);
#endif
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
#ifdef CONFIG_OF
	kfree(chip->pdata);
#endif
	kfree(chip);
	return 0;
}

static struct i2c_device_id tcs3440_idtable[] = {{"tcs3440", 0}, {} };
MODULE_DEVICE_TABLE(i2c, tcs3440_idtable);

static const struct dev_pm_ops tcs3440_pm_ops = {
	.suspend = tcs3440_suspend,
	.resume = tcs3440_resume,
};

static struct i2c_driver tcs3440_driver = {
	.driver = {
		.name = "tcs3440",
		.pm = &tcs3440_pm_ops,
		.of_match_table = of_match_ptr(tcs3440_of_match),
	},
	.id_table = tcs3440_idtable,
	.probe = tcs3440_probe,
	.remove = tcs3440_remove,
};

module_i2c_driver(tcs3440_driver);

MODULE_DESCRIPTION("AMS tcs3440 Spectral ALS sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.6");
