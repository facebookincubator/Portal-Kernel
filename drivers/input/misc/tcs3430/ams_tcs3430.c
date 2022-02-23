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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>

#include "ams_tcs3430.h"
#include "ams_i2c.h"
#include "ams_tcs3430_als.h"

#define COMPATIBLE "ams,tcs34x0"
static struct tcs3430_chip *mChip;
static int als_sensor_report_raw_data(int times);
static void als_sensor_polling_work(struct work_struct *work);

/* TCS3430 Identifiers */
static u8 const tcs3430_ids[] = {
/*    ID,    REVID,    REVID2 */
	0xDC,    0x01, 0x00,
};

/* TCS3430 Device Names */
static char const *tcs3430_names[] = {
	"tcs3430",
};

/* Registers to restore */
static u8 const restorable_regs[] = {
	TCS3430_REG_PERS,
	TCS3430_REG_CFG0,
	TCS3430_REG_CFG1,
	TCS3430_REG_ATIME,
};

static u16 const als_gains[] = {
	1,    4,
	16,   64,
	128,
};

static int als_sensor_report_raw_data(int times)
{
	struct tcs3430_chip *chip = mChip;

	u8 ch0_low, ch0_high;
	u8 ch1_low, ch1_high;
	u8 ch2_low, ch2_high;
	u8 ch3_low, ch3_high;
	u16 atime;
	u8 again;
	int temp = 0;

	ams_i2c_read(chip->client, TCS3430_REG_CH0DATAL, &ch0_low);
	ams_i2c_read(chip->client, TCS3430_REG_CH0DATAH, &ch0_high);
	ams_i2c_read(chip->client, TCS3430_REG_CH1DATAL, &ch1_low);
	ams_i2c_read(chip->client, TCS3430_REG_CH1DATAH, &ch1_high);
	ams_i2c_read(chip->client, TCS3430_REG_CH2DATAL, &ch2_low);
	ams_i2c_read(chip->client, TCS3430_REG_CH2DATAH, &ch2_high);
	ams_i2c_read(chip->client, TCS3430_REG_CH3DATAL, &ch3_low);
	ams_i2c_read(chip->client, TCS3430_REG_CH3DATAH, &ch3_high);

	chip->als_inf.x_raw = (ch3_high << 8) | ch3_low;
	chip->als_inf.y_raw = (ch1_high << 8) | ch1_low;
	chip->als_inf.z_raw = (ch0_high << 8) | ch0_low;
	chip->als_inf.ir_raw = (ch2_high << 8) | ch2_low;
	atime = ATIME_TO_MS(chip->params.als_time);
	again = als_gains[chip->params.als_gain];
	tcs3430_get_lux(chip);

	if (chip->als_inf.x_raw == 0xffff ||
		chip->als_inf.y_raw == 0xffff ||
		chip->als_inf.z_raw == 0xffff ||
		chip->als_inf.ir_raw == 0xffff) {
		chip->als_inf.x_raw = chip->als_inf_temp.x_raw;
		chip->als_inf.y_raw = chip->als_inf_temp.y_raw;
		chip->als_inf.z_raw = chip->als_inf_temp.z_raw;
		chip->als_inf.ir_raw = chip->als_inf_temp.ir_raw;
	} else {
		chip->als_inf_temp.x_raw = chip->als_inf.x_raw;
		chip->als_inf_temp.y_raw = chip->als_inf.y_raw;
		chip->als_inf_temp.z_raw = chip->als_inf.z_raw;
		chip->als_inf_temp.ir_raw = chip->als_inf.ir_raw;
	}
	if (chip->cal_enabled) {
		temp = chip->als_inf.x_raw - chip->als_inf_temp.x_raw;
		if ((times < 5) || (temp > 20))
			return 1;
		if (chip->als_inf.x_raw == 0 ||
			chip->als_inf.y_raw == 0 ||
			chip->als_inf.z_raw == 0 ||
			chip->als_inf.ir_raw == 0)
			return 1;
	}

	dev_dbg(&chip->client->dev,
			"data_info(x=%d,y=%d,z=%d,ir=%d,at=%d,ag=%d)\n",
			chip->als_inf.x_raw, chip->als_inf.y_raw,
			chip->als_inf.z_raw, chip->als_inf.ir_raw,
			atime, again);

	input_report_abs(chip->a_idev, ABS_X, chip->als_inf.x_raw);
	input_report_abs(chip->a_idev, ABS_Y, chip->als_inf.y_raw);
	input_report_abs(chip->a_idev, ABS_Z, chip->als_inf.z_raw);
	input_report_abs(chip->a_idev, ABS_RX, chip->als_inf.ir_raw);
	input_report_abs(chip->a_idev, ABS_RY, atime);
	input_report_abs(chip->a_idev, ABS_RZ, again);
	input_sync(chip->a_idev);
	return 0;

}

static void als_sensor_polling_work(struct work_struct *work)
{
	struct tcs3430_chip *chip = mChip;
	int rel = 0;
	int i = 0;

	if (chip->als_enabled) {
		schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(chip->delay));
		if (chip->als_polling)
			als_sensor_report_raw_data(0);
	} else {
		if (chip->cal_enabled) {
			do {
				rel = als_sensor_report_raw_data(++i);
				msleep(100);
			} while (rel && (i <= chip->cal_report_max_num));
			if ((i == chip->cal_report_max_num) && rel)
				dev_err(&chip->client->dev, "cal_report fail\n");
			tcs3430_configure_als_mode(chip, 0, true);
		}
	}
}

static int tcs3430_flush_regs(struct tcs3430_chip *chip)
{
	unsigned int i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg,
				chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

static int tcs3430_pltf_power_on(struct tcs3430_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
				POWER_ON);
		usleep_range(10000, 11000);
	}
	chip->unpowered = rc != 0;
	dev_dbg(&chip->client->dev, "\n\n%s: unpowered=%d\n",
			__func__, chip->unpowered);
	return rc;
}

static int tcs3430_pltf_power_off(struct tcs3430_chip *chip)
{
	int rc = 0;

	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	dev_dbg(&chip->client->dev, "%s: unpowered=%d\n",
			__func__, chip->unpowered);
	return rc;
}

static void tcs3430_set_defaults(struct tcs3430_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	/* Clear the register shadow area */
	memset(chip->shadow, 0x00, sizeof(chip->shadow));

	/* If there is platform data use it */
	if (chip->pdata) {
		dev_dbg(dev, "%s: Loading pltform data\n", __func__);
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.als_auto_gain =
				chip->pdata->parameters.als_auto_gain;
		chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
		chip->params.als_time = chip->pdata->parameters.als_time;
	} else {
		dev_dbg(dev, "%s: use defaults\n", __func__);
		chip->params.persist = ALS_PERSIST(2);
		chip->params.als_gain = AGAIN_4;
		chip->params.als_deltaP = 10;
		chip->params.als_time = AW_TIME_MS(200);
		chip->params.als_auto_gain = false;
	}

	/* Copy the default values into the register shadow area */
	sh[TCS3430_REG_PERS]     = chip->params.persist;
	sh[TCS3430_REG_ATIME]    = chip->params.als_time;
	sh[TCS3430_REG_CFG1]     = chip->params.als_gain << TCS3430_SHIFT_AGAIN;
	sh[TCS3430_REG_CFG2]     = TCS3430_CFG2_DEFAULT_VALUE;

	tcs3430_flush_regs(chip);
}

static int tcs3430_add_sysfs_interfaces(struct device *dev,
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

static void tcs3430_remove_sysfs_interfaces(struct device *dev,
					struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int tcs3430_get_id(struct tcs3430_chip *chip, u8 *id, u8 *rev, u8 *rev2)
{

	*rev2 = 0; /* TCS3430 does not have a REVID2 register */
	ams_i2c_read(chip->client, TCS3430_REG_REVID, rev);
	ams_i2c_read(chip->client, TCS3430_REG_ID, id);

	return 0;
}

static int tcs3430_power_on(struct tcs3430_chip *chip)
{
	int rc;

	rc = tcs3430_pltf_power_on(chip);
	if (rc)
		return rc;
	dev_dbg(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tcs3430_flush_regs(chip);
}

/************************************************/
/* Specific Device Setup.  Configured in Probe. */
/************************************************/
static int tcs3430_als_idev_open(struct input_dev *idev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(&idev->dev);
	int rc = 0;

	dev_dbg(&idev->dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->unpowered) {
		rc = tcs3430_power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = tcs3430_configure_als_mode(chip, 1, false);
	if (rc)
		tcs3430_pltf_power_off(chip);
chip_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

static void tcs3430_als_idev_close(struct input_dev *idev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(&idev->dev);

	dev_dbg(&idev->dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	tcs3430_configure_als_mode(chip, 0, false);
	tcs3430_pltf_power_off(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);
}

int tcs3430_init_dt(struct tcs3430_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	const char *str;
	u32 val;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "als_name", &str))
		pdata->als_name = str;

	if (!of_property_read_u32(np, "persist", &val))
		pdata->parameters.persist = val;

	if (!of_property_read_u32(np, "als_gain", &val))
		pdata->parameters.als_gain = val;

	if (!of_property_read_u32(np, "als_auto_gain", &val))
		pdata->parameters.als_auto_gain = val;

	if (!of_property_read_u32(np, "als_deltap", &val))
		pdata->parameters.als_deltaP = val;

	if (!of_property_read_u32(np, "als_time", &val))
		pdata->parameters.als_time = val;

	return 0;
}

static const struct of_device_id tcs3430_of_match[] = {
	{ .compatible = COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, tcs3430_of_match);

static int tcs3430_power_init(struct tcs3430_chip *chip)
{
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

	ret = regulator_enable(chip->vdd);
	if (ret) {
		dev_err(&chip->client->dev,
			"Regulator vdd enable failed ret=%d\n", ret);
		return ret;
	}

	return 0;

reg_vdd_put:
	regulator_put(chip->vdd);
	return ret;
}

static int tcs3430_pinctrl_init(struct device *dev)
{
	struct pinctrl_state *suspend;
	int rc;
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get(dev);
	suspend = pinctrl_lookup_state(pinctrl, "tcs34x0_suspend");
	rc = pinctrl_select_state(pinctrl, suspend);
	if (rc)
		dev_err(dev, "tcs3430 failed to set pin state, rc=%d\n", rc);
	return rc;
}

static int tcs3430_probe(struct i2c_client *client,
						const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev, rev2;
	struct device *dev = &client->dev;
	static struct tcs3430_chip *chip;
	struct tcs3430_i2c_platform_data *pdata = dev->platform_data;
	bool powered = 0;

	/****************************************/
	/* Validate bus and device registration */
	/****************************************/

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

	if (!pdata) {
		pdata = devm_kzalloc(dev,
			sizeof(struct tcs3430_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		if (of_match_device(tcs3430_of_match, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tcs3430_init_dt(pdata);
			if (ret)
				return ret;
		}
	}

	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto platform_init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		msleep(20);
	}
	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	/********************************************************************/
	/* Validate the appropriate ams device is available for this driver */
	/********************************************************************/
	tcs3430_pinctrl_init(dev);
	tcs3430_power_init(chip);
	msleep(20);
	ret = tcs3430_get_id(chip, &id, &rev, &rev2);

	id &= CHIP_ID_MASK;
	rev &= CHIP_REV_MASK;
	for (i = 0; i < ARRAY_SIZE(tcs3430_ids)/3; i++) {
		if (id == (tcs3430_ids[i*3+0]))
			if (rev == (tcs3430_ids[i*3+1]))
				if (rev2 == (tcs3430_ids[i*3+2]))
					break;
	}
	if (i < ARRAY_SIZE(tcs3430_names)) {
		dev_info(dev, "%s: '%s revid: 0x%x' detected\n", __func__,
				tcs3430_names[i], rev);
		chip->device_index = i;
	} else {
		dev_err(dev, "%s: not supported chip id: 0x%x\n", __func__,
				id);
		ret = -ENODEV;
		goto id_failed;
	}
	chip->chipid = id;
	/*********************/
	/* Set chip defaults */
	/*********************/

	tcs3430_set_defaults(chip);
	ret = tcs3430_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	/*********************/
	/* Initialize ALS    */
	/*********************/

	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = devm_input_allocate_device(dev);
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	input_set_abs_params(chip->a_idev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_Y, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_Z, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_RX, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_RY, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_RZ, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_THROTTLE, 0, 65535, 0, 0);
	chip->a_idev->open = tcs3430_als_idev_open;
	chip->a_idev->close = tcs3430_als_idev_close;
	input_set_drvdata(chip->a_idev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->als_name);
		goto input_a_alloc_failed;
	}
	ret = tcs3430_add_sysfs_interfaces(&chip->a_idev->dev,
			tcs3430_als_attrs, tcs3430_als_attrs_size);
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:

	/* Power up device */
	ams_i2c_write(chip->client, chip->shadow, TCS3430_REG_ENABLE, 0x01);

	mChip = chip;
	chip->cal_level = 0;
	chip->als_polling = 1;
	chip->cal_cct_status = 0;
	chip->cal_lux_status = 0;
	chip->delay = 500;
	chip->cal_report_max_num = 5000/200;
	chip->als_inf_temp.x_raw = 0;
	chip->als_inf_temp.y_raw = 0;
	chip->als_inf_temp.z_raw = 0;
	chip->als_inf_temp.ir_raw = 0;
	INIT_DELAYED_WORK(&chip->polling_work, als_sensor_polling_work);
	return 0;

input_a_sysfs_failed:
	input_unregister_device(chip->a_idev);
input_a_alloc_failed:
flush_regs_failed:
id_failed:
	i2c_set_clientdata(client, NULL);
	if (!chip)
		devm_kfree(dev, chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
platform_init_failed:
	if (!pdata)
		devm_kfree(dev, pdata);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tcs3430_suspend(struct device *dev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;

	if (!chip->unpowered)
		tcs3430_pltf_power_off(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs3430_resume(struct device *dev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 0;

	dev_dbg(dev, "%s: powerd %d, enabled %d",
			__func__, !chip->unpowered, chip->als_enabled);

	/* err_power: */
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs3430_remove(struct i2c_client *client)
{
	struct tcs3430_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);
	if (chip->a_idev) {
		tcs3430_remove_sysfs_interfaces(&chip->a_idev->dev,
				tcs3430_als_attrs, tcs3430_als_attrs_size);
		input_unregister_device(chip->a_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_device_id tcs3430_idtable[] = {
	{ "tcs3430", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, tcs3430_idtable);

static const struct dev_pm_ops tcs3430_pm_ops = {
	.suspend = tcs3430_suspend,
	.resume  = tcs3430_resume,
};

static struct i2c_driver tcs3430_driver = {
	.driver = {
		.name = "tcs3430",
		.pm = &tcs3430_pm_ops,
		.of_match_table = of_match_ptr(tcs3430_of_match),
	},
	.id_table = tcs3430_idtable,
	.probe = tcs3430_probe,
	.remove = tcs3430_remove,
};

module_i2c_driver(tcs3430_driver);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tcs3430 ALS, Color sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
