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
 * \brief Device driver for ambient light sensing in
 *  * TCS3440
*/

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/div64.h>

#include "ams_tcs3440.h"
#include "ams_i2c.h"
#include "qmath.h"
#include "tcs3440_calib.h"

#define QF	12
#define QCCT	12
#define QATIME 8
#define Q_STEP_SIZE_MICROSECONDS 712	/* 2.78	Q24.8 */

static uint64_t calculate_atime_in_ms_qvalue(struct tcs3440_chip *chip, uint8_t atime, uint16_t astep, uint32_t q_factor)
{
	/* return (ATIME + 1) * (ASTEP + 1) * 2.78 / 1000; */
	uint64_t q_atime = (atime + 1) * (astep + 1);
	q_atime = _int2q(q_atime, q_factor);
	q_atime = _qmul(q_atime, Q_STEP_SIZE_MICROSECONDS, q_factor, QATIME, q_factor);
	q_atime = _qkdiv(q_atime, 1000, q_factor, 0, q_factor);
	return q_atime;
}

static uint64_t calculate_again_qvalue(struct tcs3440_chip *chip, uint8_t again, uint32_t q_factor)
{
	/* return (0x01 << AGAIN) / 2.0; */
	uint64_t q_again = _int2q((0x01 << again), q_factor);
	q_again = _qkdiv(q_again, 2, q_factor, 0, q_factor);
	return q_again;
}

/* McCamy's Formula */
uint32_t tcs3440_calculate_cct(struct tcs3440_chip *chip, uint32_t x, uint32_t y, uint32_t q_factor)
{
	int32_t n, lx, ly;
	int64_t n_cubed;
	int64_t n_squared;
	uint64_t cct;
	int64_t tmp1, tmp2;

	/* convert to QCCT as required by this function */
	lx = q2q(x, q_factor, QCCT);
	ly = q2q(y, q_factor, QCCT);

	/* dev_info(&chip->client->dev, "lx = %d, ly = %d\n", lx, ly); */

	if (ly == 761) {
		/* prevent divide by zero */
		cct = 0;
	} else {
		tmp1 = qsub(lx, 1360);
		tmp2 = qsub(761, ly);
		n = _qk_s64_div(tmp1, tmp2, QCCT, QCCT, QCCT);
		/* dev_info(&chip->client->dev, "tmp1 = %lld, tmp2 = %lld\n", tmp1, tmp2); */

		/* brute force computations because some platforms don't have pow() */
		n_squared = _qmul(n, n, QCCT, QCCT, QCCT);
		n_cubed	= _qmul(n_squared, n, QCCT, QCCT, QCCT);
		/* dev_info(&chip->client->dev, "n = %d, n_squared = %lld, n_cubed = %lld\n", n, n_squared, n_cubed); */

		cct = qadd(_qmul(1839104, n_cubed, QCCT, QCCT, QCCT), _qmul(14438400, n_squared, QCCT, QCCT, QCCT));
		cct = qadd(cct, _qmul(27948237, n, QCCT, QCCT, QCCT));
		cct = qadd(cct, 22611272);
	}

	/* Return CCT in same format as input */
	return q2q(cct, QCCT, q_factor);
}


int tcs3440_calculate_lux_and_cct(struct tcs3440_chip *chip, const struct adc_data *adc,
			struct calibration_data *cal_data,
			struct als_xyz_data *xyz)
{
	struct calibration_data *p_cal_data;
	int32_t qf;
	int32_t cal_matrix[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
	uint64_t atime, nom_atime;
	uint64_t again, nom_again;
	int64_t tmp1, tmp2, tmp3;
	int64_t sum;
	int64_t norm_f1, norm_f2, norm_f3, norm_f4, norm_f5, norm_f6, norm_f7, norm_f8, norm_clear, norm_nir;
	int32_t q_f1, q_f2, q_f3, q_f4, q_f5, q_f6, q_f7, q_f8, q_clear, q_nir;
	uint32_t q_cct;
	int tmp_lux;
	int tmp_cct;

	if (!xyz || !adc)
		return -1;

	if (cal_data) {
		p_cal_data = cal_data;
	} else {
		p_cal_data = &tcs3440_default_cal_data;
	}

	qf = p_cal_data->coef.q_factor;

	memcpy(&cal_matrix, p_cal_data->coef.data, sizeof(int32_t) * MATRIX_ROW_SIZE * MATRIX_COL_SIZE);

	if (p_cal_data->nominal_atime == 0) {
		nom_atime = q2q(NOMINAL_ATIME_DEFAULT, QF, qf);
	} else {
		nom_atime = _int2q(p_cal_data->nominal_atime, qf);
	}
	if (p_cal_data->nominal_again == 0) {
		nom_again = q2q(NOMINAL_AGAIN_DEFAULT, QF, qf);
	} else {
		nom_again = _int2q(p_cal_data->nominal_again, qf);
	}

	/* Qformat for atime and again */
	atime = calculate_atime_in_ms_qvalue(chip, adc->atime, adc->astep, qf);
	again = calculate_again_qvalue(chip, adc->again, qf);

	/* Convert input data to Qformat */
	q_f1 = _int2q(adc->f1, qf);
	q_f2 = _int2q(adc->f2, qf);
	q_f3 = _int2q(adc->f3, qf);
	q_f4 = _int2q(adc->f4, qf);
	q_f5 = _int2q(adc->f5, qf);
	q_f6 = _int2q(adc->f6, qf);
	q_f7 = _int2q(adc->f7, qf);
	q_f8 = _int2q(adc->f8, qf);
	q_clear = _int2q(adc->clear, qf);
	q_nir = _int2q(adc->nir, qf);

	/* Normalize channel data to nominal atime and again */
	/* channel' = channel * (nom_atime * nom_again) / (atime * again) */
	tmp1 = _qmul(nom_again, nom_atime, qf, qf, qf);
	tmp2 = _qmul(atime, again, qf, qf, qf);
	tmp3 = _qkdiv(tmp1, tmp2, qf, qf, qf);
	norm_f1 = _qmul(q_f1, tmp3, qf, qf, qf);
	norm_f2 = _qmul(q_f2, tmp3, qf, qf, qf);
	norm_f3 = _qmul(q_f3, tmp3, qf, qf, qf);
	norm_f4 = _qmul(q_f4, tmp3, qf, qf, qf);
	norm_f5 = _qmul(q_f5, tmp3, qf, qf, qf);
	norm_f6 = _qmul(q_f6, tmp3, qf, qf, qf);
	norm_f7 = _qmul(q_f7, tmp3, qf, qf, qf);
	norm_f8 = _qmul(q_f8, tmp3, qf, qf, qf);
	norm_clear = _qmul(q_clear, tmp3, qf, qf, qf);
	norm_nir = _qmul(q_nir, tmp3, qf, qf, qf);

	/* Compute tristimulus values */
	/* X */
	xyz->tristimulus.x = 0;
	tmp1 = _qmul(norm_f1, cal_matrix[TRI_IDX_X][CH_IDX_F1], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f2, cal_matrix[TRI_IDX_X][CH_IDX_F2], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f3, cal_matrix[TRI_IDX_X][CH_IDX_F3], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f4, cal_matrix[TRI_IDX_X][CH_IDX_F4], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f5, cal_matrix[TRI_IDX_X][CH_IDX_F5], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f6, cal_matrix[TRI_IDX_X][CH_IDX_F6], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f7, cal_matrix[TRI_IDX_X][CH_IDX_F7], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_f8, cal_matrix[TRI_IDX_X][CH_IDX_F8], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_clear, cal_matrix[TRI_IDX_X][CH_IDX_CLEAR], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);
	tmp1 = _qmul(norm_nir, cal_matrix[TRI_IDX_X][CH_IDX_NIR], qf, qf, qf);
	xyz->tristimulus.x = qadd(xyz->tristimulus.x, tmp1);

	/* Y */
	xyz->tristimulus.y = 0;
	tmp1 = _qmul(norm_f1, cal_matrix[TRI_IDX_Y][CH_IDX_F1], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f2, cal_matrix[TRI_IDX_Y][CH_IDX_F2], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f3, cal_matrix[TRI_IDX_Y][CH_IDX_F3], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f4, cal_matrix[TRI_IDX_Y][CH_IDX_F4], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f5, cal_matrix[TRI_IDX_Y][CH_IDX_F5], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f6, cal_matrix[TRI_IDX_Y][CH_IDX_F6], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f7, cal_matrix[TRI_IDX_Y][CH_IDX_F7], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_f8, cal_matrix[TRI_IDX_Y][CH_IDX_F8], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_clear, cal_matrix[TRI_IDX_Y][CH_IDX_CLEAR], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);
	tmp1 = _qmul(norm_nir, cal_matrix[TRI_IDX_Y][CH_IDX_NIR], qf, qf, qf);
	xyz->tristimulus.y = qadd(xyz->tristimulus.y, tmp1);

	/* Z */
	xyz->tristimulus.z = 0;
	tmp1 = _qmul(norm_f1, cal_matrix[TRI_IDX_Z][CH_IDX_F1], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f2, cal_matrix[TRI_IDX_Z][CH_IDX_F2], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f3, cal_matrix[TRI_IDX_Z][CH_IDX_F3], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f4, cal_matrix[TRI_IDX_Z][CH_IDX_F4], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f5, cal_matrix[TRI_IDX_Z][CH_IDX_F5], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f6, cal_matrix[TRI_IDX_Z][CH_IDX_F6], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f7, cal_matrix[TRI_IDX_Z][CH_IDX_F7], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_f8, cal_matrix[TRI_IDX_Z][CH_IDX_F8], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_clear, cal_matrix[TRI_IDX_Z][CH_IDX_CLEAR], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);
	tmp1 = _qmul(norm_nir, cal_matrix[TRI_IDX_Z][CH_IDX_NIR], qf, qf, qf);
	xyz->tristimulus.z = qadd(xyz->tristimulus.z, tmp1);

	/* Calculate CIE Chromaticity Coordinates */
	sum = qadd(qadd(xyz->tristimulus.x, xyz->tristimulus.y), xyz->tristimulus.z);
	if (sum > 0) {
		xyz->chromaticity_x = _qkdiv(xyz->tristimulus.x, sum, qf, qf, qf);
		xyz->chromaticity_y = _qkdiv(xyz->tristimulus.y, sum, qf, qf, qf);
	}

	/* Convert from Q to int lux */
	tmp_lux = _q2int(xyz->tristimulus.y, qf);
	xyz->lux = max(tmp_lux, 0);

	/* Calculate cct */
	q_cct = tcs3440_calculate_cct(chip, xyz->chromaticity_x, xyz->chromaticity_y, qf);
	tmp_cct =  _q2int(q_cct, qf);
	xyz->cct = max(tmp_cct, 0);

	return 0;
}
