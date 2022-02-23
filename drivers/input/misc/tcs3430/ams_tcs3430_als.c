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
 * functionality within the AMS-TAOS TCS3430 family of devices.
 */

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "ams_tcs3430.h"
#include "ams_i2c.h"

#define GAIN1         0
#define GAIN4         1
#define GAIN16        2
#define GAIN64        3
#define GAIN128       3 /* GAIN128 requires HGAIN to be set */
#define ALS_NUM_CH    4
#define ALS_CH_SIZE   (sizeof(u8) * 2)

static u16 const als_gains[] = {
	1,    4,
	16,   64,
	128,
};

static u8 const restorable_als_regs[] = {
	TCS3430_REG_ATIME,
	TCS3430_REG_WTIME,
	TCS3430_REG_PERS,
	TCS3430_REG_CFG0,
	TCS3430_REG_CFG1,
	TCS3430_REG_CFG2,
};

static int tcs3430_flush_als_regs(struct tcs3430_chip *chip)
{
	unsigned int i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow,
				reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

static void tcs3430_get_als(struct tcs3430_chip *chip)
{
	u8 *sh = chip->shadow;

	/* extract raw channel data */
	chip->als_inf.z_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH0DATAL]);
	chip->als_inf.y_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH1DATAL]);
	chip->als_inf.ir_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH2DATAL]);
	chip->als_inf.x_raw =
		le16_to_cpup((const __le16 *)&sh[TCS3430_REG_CH3DATAL]);
}

int tcs3430_read_als(struct tcs3430_chip *chip)
{
	int ret;

	ret = ams_i2c_blk_read(chip->client, TCS3430_REG_CH0DATAL,
			&chip->shadow[TCS3430_REG_CH0DATAL],
			ALS_NUM_CH * ALS_CH_SIZE);

	if (ret >= 0) {
		tcs3430_get_als(chip);
		ret = 0;
	}

	return ret;
}

static void tcs3430_calc_cpl(struct tcs3430_chip *chip)
{
	u64 cpl;
	u32 sat;
	u8 atime;
	u8 gain_idx;

	atime = chip->shadow[TCS3430_REG_ATIME];

	cpl = atime;
	cpl *= INTEGRATION_CYCLE;
	gain_idx = chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN;
	gain_idx >>= TCS3430_SHIFT_AGAIN;
	cpl *= als_gains[gain_idx];
	if (chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN)
		cpl *= 2;
	cpl *= DGF_SCALE_DEFAULT;

	sat = min_t(u32, TCS3430_MAX_ALS_VALUE, (u32) atime << 10);

	chip->als_inf.cpl = (u32) cpl;
	chip->als_inf.saturation = TENTH_FRACTION_OF_VAL(sat, 8);
}

int tcs3430_configure_als_mode(struct tcs3430_chip *chip, u8 state,
	bool flag)
{
	static int als_en_count, cal_en_count;
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) {/* Turning on ALS */
		if (flag) {
			cal_en_count++;
			chip->cal_enabled = true;
		} else {
			als_en_count++;
			chip->als_enabled = true;
		}

		dev_dbg(&client->dev, "%s: cal_en_count is %d, als_en_count is %d\n",
				__func__, cal_en_count, als_en_count);

		if ((cal_en_count + als_en_count) != 1)
			return 0;

		chip->shadow[TCS3430_REG_ATIME] = chip->params.als_time;
		tcs3430_calc_cpl(chip);

		/* set PERS.apers to 2 consecutive ALS values out of range */
		chip->shadow[TCS3430_REG_PERS] &= (~TCS3430_MASK_APERS);
		chip->shadow[TCS3430_REG_PERS] |= 0x02;

		tcs3430_flush_als_regs(chip);
		ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
				TCS3430_WEN | TCS3430_AEN | TCS3430_PON,
				TCS3430_WEN | TCS3430_AEN | TCS3430_PON);
	} else  {/* Turning off ALS */
		if (!als_en_count && !cal_en_count) {
			dev_warn(&client->dev,
			"%s: Calling ALS disable without first enabling it!", __func__);
			return -EINVAL;
		}

		if (flag) {
			if (cal_en_count)
				cal_en_count--;
			if (!cal_en_count)
				chip->cal_enabled = false;
		} else {
			if (als_en_count)
				als_en_count--;
			if (!als_en_count)
				chip->als_enabled = false;
		}

		dev_dbg(&client->dev, "%s: cal_en_count is %d, als_en_count is %d\n",
				__func__, cal_en_count, als_en_count);

		if ((cal_en_count + als_en_count) != 0)
			return 0;

		/* Disable ALS, Wait and ALS Interrupt */
		ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
				TCS3430_WEN | TCS3430_AEN, 0);

		/* If nothing else is enabled set PON = 0;*/
		if (!(sh[TCS3430_REG_ENABLE] & TCS3430_EN_ALL))
			ams_i2c_modify(client, sh, TCS3430_REG_ENABLE,
					TCS3430_PON, 0);
	}

	return 0;
}

static int tcs3430_set_als_gain(struct tcs3430_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg;
	u8 hgain = 0;

	switch (gain) {
	case 1:
		ctrl_reg = AGAIN_1;
		break;
	case 4:
		ctrl_reg = AGAIN_4;
		break;
	case 16:
		ctrl_reg = AGAIN_16;
		break;
	case 64:
		ctrl_reg = AGAIN_64;
		break;
	case 128:
		ctrl_reg = AGAIN_64;
		hgain = 1;
		break;
	default:
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}

	ctrl_reg <<= TCS3430_SHIFT_AGAIN;
	/**
	 * Turn off ALS, so that new ALS gain value will take effect at start of
	 * new integration cycle.
	 * New ALS gain value will then be used in next lux calculation.
	 */
	rc = ams_i2c_modify(chip->client, chip->shadow, TCS3430_REG_CFG1,
			TCS3430_MASK_AGAIN, ctrl_reg);
	rc |= ams_i2c_modify(chip->client, chip->shadow, TCS3430_REG_CFG2,
			TCS3430_MASK_HGAIN, ((hgain << TCS3430_SHIFT_HGAIN) |
			4));
	if (rc >= 0) {
		chip->params.als_gain =
			(chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >>
				TCS3430_SHIFT_AGAIN;
		chip->params.als_gain += hgain;
		dev_dbg(&chip->client->dev, "%s: new als gain %d\n",
				__func__, chip->params.als_gain);
	}

	return rc;
}

static void tcs3430_inc_gain(struct tcs3430_chip *chip)
{
	int rc;

	u8 gain_idx = (chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >>
					TCS3430_SHIFT_AGAIN;
	s8 idx;
	u8 gain = als_gains[gain_idx];

	if ((chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN) &&
			(gain == 64)) {
		gain = 128;
		gain_idx = ARRAY_SIZE(als_gains) - 1;
	}

	if (gain >= als_gains[(ARRAY_SIZE(als_gains) - 1)])
		return;
	for (idx = 0; idx <= (ARRAY_SIZE(als_gains) - 1); idx++) {
		if ((als_gains[idx] == 0) || (idx <= gain_idx))
			continue;
		else if (idx > gain_idx) {
			gain_idx = idx;
			break;
		}
	}
	dev_dbg(&chip->client->dev, "Autogain INC to %d\n",
			als_gains[gain_idx]);
	rc = tcs3430_set_als_gain(chip, als_gains[gain_idx]);
	if (rc == 0)
		tcs3430_calc_cpl(chip);
}

static void tcs3430_dec_gain(struct tcs3430_chip *chip)
{
	int rc;
	u8 gain_idx = (chip->shadow[TCS3430_REG_CFG1] & TCS3430_MASK_AGAIN) >>
					TCS3430_SHIFT_AGAIN;
	s8 idx;
	u8 gain = als_gains[gain_idx];

	if ((chip->shadow[TCS3430_REG_CFG2] & TCS3430_MASK_HGAIN) &&
			(gain == 64)) {
		gain = 128;
		gain_idx = ARRAY_SIZE(als_gains) - 1;
	}

	if (gain <= als_gains[0])
		return;
	for (idx = (ARRAY_SIZE(als_gains) - 1); idx >= 0; idx--) {
		if ((als_gains[idx] == 0) || (idx >= gain_idx))
			continue;
		else if (idx < gain_idx) {
			gain_idx = idx;
			break;
		}
	}
	dev_dbg(&chip->client->dev, "Autogain DEC to %d\n",
			als_gains[gain_idx]);
	rc = tcs3430_set_als_gain(chip, als_gains[gain_idx]);
	if (rc == 0)
		tcs3430_calc_cpl(chip);
}

int tcs3430_get_lux(struct tcs3430_chip *chip)
{

	int quintile = 200;

	/* Auto gain moved to end of function */

	/* use time in ms get scaling factor */

	if (!chip->params.als_auto_gain) {
		if (chip->als_inf.z_raw <= TCS3430_MIN_ALS_VALUE)
			dev_dbg(&chip->client->dev, "%s: darkness (%d <= %d)\n",
				__func__, chip->als_inf.z_raw,
				TCS3430_MIN_ALS_VALUE);
		else if (chip->als_inf.z_raw >=
			chip->als_inf.saturation)
			dev_dbg(&chip->client->dev,
				"%s: saturation (%d >= %d\n",
				__func__, chip->als_inf.z_raw,
				chip->als_inf.saturation);
	} else {
		if (chip->als_inf.z_raw < quintile ||
			chip->als_inf.x_raw < quintile ||
			chip->als_inf.y_raw < quintile) {
			tcs3430_inc_gain(chip);
			tcs3430_flush_als_regs(chip);
		} else if (chip->als_inf.z_raw >= chip->als_inf.saturation ||
			chip->als_inf.x_raw >= chip->als_inf.saturation ||
			chip->als_inf.y_raw >= chip->als_inf.saturation ||
			chip->als_inf.ir_raw >= chip->als_inf.saturation) {
			tcs3430_dec_gain(chip);
			tcs3430_flush_als_regs(chip);
		}
	}
	chip->als_inf.cct = 0;
	chip->als_inf.lux = 0;

	return 0;
}

int tcs3430_update_als_thres(struct tcs3430_chip *chip, bool on_enable)
{
	s32 ret;
	u16 deltaP = chip->params.als_deltaP;
	u32 from, to, cur;
	u32 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.z_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > (saturation / 2) ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;
	}
	from = cpu_to_le16(from);
	to = cpu_to_le16(to);
	memcpy(&chip->shadow[TCS3430_REG_AILTL], &from, ALS_CH_SIZE);
	memcpy(&chip->shadow[TCS3430_REG_AIHTL],   &to, ALS_CH_SIZE);

	ret = ams_i2c_reg_blk_write(chip->client, TCS3430_REG_AILTL,
			&chip->shadow[TCS3430_REG_AILTL],
			(TCS3430_REG_AIHTH - TCS3430_REG_AILTL) + 1);

	return (ret < 0) ? ret : 0;
}

/*****************/
/* ABI Functions */
/*****************/

static ssize_t tcs3430_device_als_lux(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);

	tcs3430_read_als(chip);
	tcs3430_get_lux(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tcs3430_lux_coef_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int k = 0;
	/* reserved */
	return k;
}

static ssize_t tcs3430_lux_coef_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}

static ssize_t tcs3430_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tcs3430_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tcs3430_configure_als_mode(chip, 1, false);
	else
		tcs3430_configure_als_mode(chip, 0, false);

	dev_dbg(dev, " tcs3430 enabled=%d, polling=%d, value=%d\n",
			chip->als_enabled, chip->als_polling, value);

	if (chip->als_enabled & chip->als_polling)
		schedule_delayed_work(&chip->polling_work,
			msecs_to_jiffies(1000));

	return size;
}

static ssize_t tcs3430_auto_gain_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->params.als_auto_gain ? "auto" : "manual");
}

static ssize_t tcs3430_auto_gain_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		chip->params.als_auto_gain = true;
	else
		chip->params.als_auto_gain = false;

	return size;
}

static ssize_t tcs3430_als_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "gain:%d (%s)\n",
			als_gains[(chip->params.als_gain)],
			chip->params.als_auto_gain ? "auto" : "manual");
}

static ssize_t tcs3430_als_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long gain;
	int i = 0;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;
	if (gain != 0   && gain != 1   && gain != 4   &&
			gain != 16  &&  gain != 64 && gain != 128)
		return -EINVAL;

	while (i < ARRAY_SIZE(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (i >= ARRAY_SIZE(als_gains)) {
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, (int)gain);
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);

	if (gain) {
		chip->params.als_auto_gain = false;
		rc = tcs3430_set_als_gain(chip, als_gains[i]);
		if (!rc)
			tcs3430_calc_cpl(chip);
	} else {
		chip->params.als_auto_gain = true;
	}
	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return rc ? -EIO : size;
}

static ssize_t tcs3430_als_z_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.z_raw);
}

static ssize_t tcs3430_als_y_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.y_raw);
}

static ssize_t tcs3430_als_cpl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t tcs3430_als_ir_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ir_raw);
}

static ssize_t tcs3430_als_x_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.x_raw);
}

static ssize_t tcs3430_als_cct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	tcs3430_read_als(chip);
	tcs3430_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tcs3430_als_persist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TCS3430_REG_PERS]) &
			TCS3430_MASK_APERS)));
}

static ssize_t tcs3430_als_persist_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	long persist;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	chip->shadow[TCS3430_REG_PERS] &= ~TCS3430_MASK_APERS;
	chip->shadow[TCS3430_REG_PERS] |= ((u8)persist & TCS3430_MASK_APERS);

	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tcs3430_als_atime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int t;

	t = chip->shadow[TCS3430_REG_ATIME];
	t += 1; /* t = INTEGRATION_CYCLE if atime == 0 */
	t *= INTEGRATION_CYCLE;
	return snprintf(buf, PAGE_SIZE, "atime:%dms (%dus)\n", t / 1000, t);
}

static ssize_t tcs3430_als_atime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	long atime;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &atime);
	if (rc)
		return -EINVAL;
	atime = AW_TIME_MS(atime); /* Assume value in ms */

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TCS3430_REG_ATIME] = (u8) atime;
	chip->params.als_time = chip->shadow[TCS3430_REG_ATIME];
	tcs3430_calc_cpl(chip);
	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tcs3430_als_wtime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int t;
	u8 wlongcurr;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);

	t = chip->shadow[TCS3430_REG_WTIME];

	wlongcurr = chip->shadow[TCS3430_REG_CFG0] & TCS3430_MASK_WLONG;
	if (wlongcurr)
		t *= 12;

	t *= WAIT_CYCLE;
	t /= 1000;

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tcs3430_als_wtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	unsigned long wtime;
	int wlong;
	int rc;

	rc = kstrtoul(buf, 10, &wtime);
	if (rc)
		return -EINVAL;

	wtime *= 1000;
	if (wtime > (256 * WAIT_CYCLE)) {
		wlong = 1;
		wtime /= 12;
	} else
		wlong = 0;
	wtime /= WAIT_CYCLE;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TCS3430_REG_WTIME] = (u8) wtime;
	if (wlong)
		chip->shadow[TCS3430_REG_CFG0] |= TCS3430_MASK_WLONG;
	else
		chip->shadow[TCS3430_REG_CFG0] &= ~TCS3430_MASK_WLONG;

	tcs3430_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}


static ssize_t tcs3430_als_deltaP_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tcs3430_als_deltaP_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &deltaP);
	if (rc || deltaP > 100)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->params.als_deltaP = deltaP;
	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tcs3430_als_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	tcs3430_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "lux:%d,cct:%d,x:%d,y:%d,z:%d,ir:%d\n",
			chip->als_inf.lux,
			chip->als_inf.cct,
			chip->als_inf.x_raw,
			chip->als_inf.y_raw,
			chip->als_inf.z_raw,
			chip->als_inf.ir_raw);
}

static ssize_t tcs3430_als_adc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	u32 z, y, ir, x;

	if (sscanf(buf, "%10d,%10d,%10d,%10d", &x, &y, &z, &ir) != 4)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->als_inf.x_raw = x;
	chip->als_inf.y_raw = y;
	chip->als_inf.z_raw = z;
	chip->als_inf.ir_raw = ir;

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tcs3430_als_rgb_rawh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	u8 low[4], high[4];
	uint8_t i = 0;
	ssize_t len = 0;
	int temp[4];
	char *channel = " X Y ZIR";
	int t;

	uint8_t reg_map[] = {
		TCS3430_REG_CH3DATAL,
		TCS3430_REG_CH3DATAH,
		TCS3430_REG_CH1DATAL,
		TCS3430_REG_CH1DATAH,
		TCS3430_REG_CH0DATAL,
		TCS3430_REG_CH0DATAH,
		TCS3430_REG_CH2DATAL,
		TCS3430_REG_CH2DATAH
	};
	for (i = 0; i < 8;) {
		ams_i2c_read(chip->client, reg_map[i], &low[i/2]);
		ams_i2c_read(chip->client, reg_map[i+1], &high[i/2]);
		i += 2;
	}
	t = chip->shadow[TCS3430_REG_ATIME];
	t += 1; /* t = INTEGRATION_CYCLE if atime == 0 */
	t *= INTEGRATION_CYCLE;

	for (i = 0; i < 4; ++i) {
		temp[i] = (high[i] << 8) | low[i];
		len += snprintf(buf+len, PAGE_SIZE-len,
			"Reg:0x%x(H),0x%x(L) %c%c: %d \t0x%x\n",
			reg_map[i*2+1], reg_map[i*2], channel[i*2],
			channel[i*2+1], temp[i], temp[i]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len,
			"atime: %d\n", t / 1000);
	len += snprintf(buf+len, PAGE_SIZE-len,
			"gain: %d\n", als_gains[(chip->params.als_gain)]);

	len += snprintf(buf+len, PAGE_SIZE-len,
			"%d %d %d %d %d %d\n",
			temp[0], temp[1], temp[2], temp[3], t / 1000,
			 als_gains[(chip->params.als_gain)]);

	return len;
}

static ssize_t tcs3430_als_rgb_rawh_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t tcs3430_als_polling_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->als_polling);
}

static ssize_t tcs3430_als_polling_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool value;

	strtobool(buf, &value);
	cancel_delayed_work(&chip->polling_work);
	chip->als_polling = value;
	if (chip->als_enabled & chip->als_polling)
		schedule_delayed_work(&chip->polling_work,
			msecs_to_jiffies(100));

	return size;
}

static ssize_t tcs3430_cal_level_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n",
				chip->cal_level &
				~TCS3430_CAL_STATUS_ALL_MASK_H);
}

static ssize_t tcs3430_cal_level_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3430 change str to int fail\n");
		return size;
	}

	if ((chip->cal_level & ~TCS3430_CAL_STATUS_ALL_MASK_H) != value) {
		dev_dbg(dev, "tcs3430 enable sensor for calibration\n");
		tcs3430_configure_als_mode(chip, 1, true);
		chip->cal_level = value |
			(chip->cal_level &
			TCS3430_CAL_STATUS_ALL_MASK_H);
		switch (value) {
		case 3000:
			chip->cal_cct_status = -1;
			input_report_abs(chip->a_idev, ABS_THROTTLE, 3000);
			input_sync(chip->a_idev);
			schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(1000));
			dev_dbg(dev, "tcs3430 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
					~TCS3430_CAL_STATUS_CCT_3000_MASK_H;
			break;
		case 5000:
			chip->cal_cct_status = -1;
			input_report_abs(chip->a_idev, ABS_THROTTLE, 5000);
			input_sync(chip->a_idev);
			schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(1000));
			dev_dbg(dev, "tcs3430 cal: %dK is starting\n", value);
			chip->cal_level = chip->cal_level &
					~TCS3430_CAL_STATUS_CCT_5000_MASK_H;
			break;
		case 1000:
			chip->cal_lux_status = -1;
			input_report_abs(chip->a_idev, ABS_THROTTLE, 1000);
			input_sync(chip->a_idev);
			schedule_delayed_work(&chip->polling_work,
					msecs_to_jiffies(1000));
			dev_dbg(dev, "tcs3430 cal: %dLux is starting\n", value);
			chip->cal_level = chip->cal_level &
					~TCS3430_CAL_STATUS_LUX_1000_MASK_H;
			break;
		default:
			dev_err(dev, "tcs3430 cal: %d(K/Lux) is Non-existent\n",
					value);
		}
	} else {
		dev_err(dev, "tcs3430 cal: %d(K/Lux) is running or Non-existent.\n",
					value);
	}
	return size;
}

static ssize_t tcs3430_cal_cct_status_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->cal_cct_status);
}

static ssize_t tcs3430_cal_cct_status_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3430 change str to int fail\n");
		return size;
	}

	chip->cal_cct_status = value;

	dev_dbg(dev, " tcs3430 cal(%dK) result: %d\n",
			chip->cal_level & ~TCS3430_CAL_STATUS_ALL_MASK_H,
			chip->cal_cct_status);
	switch (chip->cal_level & ~TCS3430_CAL_STATUS_ALL_MASK_H) {
	case 3000:
		chip->cal_level = (chip->cal_level &
				TCS3430_CAL_STATUS_ALL_MASK_H)|
				TCS3430_CAL_STATUS_CCT_3000_MASK_H;
		break;
	case 5000:
		chip->cal_level = (chip->cal_level &
				TCS3430_CAL_STATUS_ALL_MASK_H)|
				TCS3430_CAL_STATUS_CCT_5000_MASK_H;
		break;
	default:
		break;
	}
	return size;
}

static ssize_t tcs3430_cal_lux_status_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->cal_lux_status);
}

static ssize_t tcs3430_cal_lux_status_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3430 change str to int fail\n");
		return size;
	}

	chip->cal_lux_status = value;

	dev_dbg(dev, " tcs3430 cal(%dLux) result: %d\n",
			chip->cal_level & ~TCS3430_CAL_STATUS_ALL_MASK_H,
			chip->cal_lux_status);
	if ((chip->cal_level & ~TCS3430_CAL_STATUS_ALL_MASK_H) == 1000) {
		chip->cal_level = (chip->cal_level &
				TCS3430_CAL_STATUS_ALL_MASK_H) |
				TCS3430_CAL_STATUS_LUX_1000_MASK_H;
	}
	return size;
}

static ssize_t tcs3430_cal_status_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->cal_level &
					TCS3430_CAL_STATUS_ALL_MASK_H);
}

static ssize_t tcs3430_cal_status_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3430 change str to int fail\n");
		return size;
	}

	if (value == 1) {
		input_report_abs(chip->a_idev, ABS_THROTTLE, 1);
		input_sync(chip->a_idev);
		chip->cal_cct_status = 0;
		chip->cal_lux_status = 0;
		chip->cal_level = 0;
		dev_dbg(dev, "tcs3430 clear&reset cal_status\n");
	}
	return size;
}

static ssize_t tcs3430_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d(ms)\n", chip->delay);
}

static ssize_t tcs3430_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	int value;
	int error;

	error = kstrtoint(buf, 10, &value);
	if (error < 0) {
		dev_err(dev, "tcs3430 change str to int fail\n");
		return size;
	}

	if ((chip->delay != value) || !chip->als_enabled)
		chip->delay = value;

	return size;
}

static ssize_t tcs3430_als_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	return  snprintf(buf, PAGE_SIZE, "%d\n", chip->chipid);
}

struct device_attribute tcs3430_als_attrs[] = {
	__ATTR(als_atime,            0660, tcs3430_als_atime_show,
		tcs3430_als_atime_store),
	__ATTR(als_wtime,            0660, tcs3430_als_wtime_show,
		tcs3430_als_wtime_store),
	__ATTR(als_lux,              0440, tcs3430_device_als_lux,
		NULL),
	__ATTR(als_id,               0440, tcs3430_als_id_show,
		NULL),
	__ATTR(als_gain,             0660, tcs3430_als_gain_show,
		tcs3430_als_gain_store),
	__ATTR(als_cpl,              0440, tcs3430_als_cpl_show,
		NULL),
	__ATTR(als_cct,              0440, tcs3430_als_cct_show,
		NULL),
	__ATTR(als_z,                0440, tcs3430_als_z_raw_show,
		NULL),
	__ATTR(als_y,                0440, tcs3430_als_y_raw_show,
		NULL),
	__ATTR(als_ir,               0440, tcs3430_als_ir_raw_show,
		NULL),
	__ATTR(als_x,                0440, tcs3430_als_x_raw_show,
		NULL),
	__ATTR(als_thresh_deltaP,    0660, tcs3430_als_deltaP_show,
		tcs3430_als_deltaP_store),
	__ATTR(als_auto_gain,        0660, tcs3430_auto_gain_enable_show,
		tcs3430_auto_gain_enable_store),
	__ATTR(als_lux_coef,         0660, tcs3430_lux_coef_show,
		tcs3430_lux_coef_store),
	__ATTR(enable,               0660, tcs3430_enable_show,
		tcs3430_enable_store),
	__ATTR(als_persist,          0660, tcs3430_als_persist_show,
		tcs3430_als_persist_store),
	__ATTR(als_adc,              0660, tcs3430_als_adc_show,
		tcs3430_als_adc_store),
	__ATTR(als_rgb_rawh,         0660, tcs3430_als_rgb_rawh_show,
		tcs3430_als_rgb_rawh_store),
	__ATTR(als_polling_h,        0660, tcs3430_als_polling_h_show,
		tcs3430_als_polling_h_store),
	__ATTR(cal_level_h,          0660, tcs3430_cal_level_h_show,
		tcs3430_cal_level_h_store),
	__ATTR(cal_cct_status_h,     0660, tcs3430_cal_cct_status_h_show,
		tcs3430_cal_cct_status_h_store),
	__ATTR(cal_lux_status_h,     0660, tcs3430_cal_lux_status_h_show,
		tcs3430_cal_lux_status_h_store),
	__ATTR(cal_status_h,         0660, tcs3430_cal_status_h_show,
		tcs3430_cal_status_h_store),
	__ATTR(delay,                0660, tcs3430_delay_show,
		tcs3430_delay_store),
};

int tcs3430_als_attrs_size = ARRAY_SIZE(tcs3430_als_attrs);
