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

#ifndef __AMS_TCS3440_ALS_H
#define __AMS_TCS3440_ALS_H

uint32_t tcs3440_calculate_cct(struct tcs3440_chip *chip, uint32_t x, uint32_t y, uint32_t q_factor);
int tcs3440_calculate_lux_and_cct(struct tcs3440_chip *chip,
	const struct adc_data *adc, const struct calibration_data *cal_data,
	struct als_xyz_data *xyz);

extern struct device_attribute tcs3440_als_attrs[];
extern int tcs3440_als_attrs_size;

extern int tcs3440_enable_device(struct tcs3440_chip *chip, u8 state);
extern int tcs3440_configure_mode(struct tcs3440_chip *chip, u8 state);
extern int tcs3440_read_als(struct tcs3440_chip *chip);
extern int start_spectral_measurement(struct tcs3440_chip *chip);
extern int stop_spectral_measurement(struct tcs3440_chip *chip);
extern int tcs3440_get_channel_data(struct tcs3440_chip *chip, int raw_count,
		operation_mode mode, int *sensor_row_count);
extern operation_mode get_spectral_mode(void);
extern int set_spectral_mode(operation_mode mode);

#endif /* __AMS_TCS3440_ALS_H */
