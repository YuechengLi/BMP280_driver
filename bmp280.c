/*
****************************************************************************
* Copyright (C) 2012 - 2014 Bosch Sensortec GmbH
*
* File : bmp280.h
*
* Date : 2014/12/12
*
* Revision : 2.0.3(Pressure and Temperature compensation code revision is 1.1)
*
* Usage: Sensor Driver for BMP280 sensor
*
****************************************************************************
*
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
#include "bmp280.h"
static struct bmp280_t *p_bmp280; /**< pointer to BMP280 */

/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *bmp280 structure pointer.
 *
 *	@note While changing the parameter of the p_bmp280
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	p_bmp280 = bmp280;/* assign BMP280 ptr */
	/* read chip id */
	com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
	BMP280_CHIP_ID_REG, &v_data_u8, BMP280_ONE_U8X);/* read Chip Id */
	p_bmp280->chip_id = v_data_u8;
	/* readout bmp280 calibparam structure */
	com_rslt += bmp280_get_calib_param();
	return com_rslt;
}
/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 *	@param v_uncomp_temperature_s32 : The uncompensated temperature.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_temperature(
s32 *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	a_data_u8r[0] - Temperature MSB
	a_data_u8r[1] - Temperature LSB
	a_data_u8r[2] - Temperature LSB
	*/
	u8 a_data_u8r[ARRAY_SIZE_THREE] = {
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read temperature data */
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_TEMPERATURE_MSB_REG,
			a_data_u8r, BMP280_THREE_U8X);
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8r[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[INDEX_TWO]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return Actual temperature output as s32
 *
*/
s32 bmp280_compensate_T_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = BMP280_ZERO_U8X;
	s32 v_x2_u32r = BMP280_ZERO_U8X;
	s32 temperature = BMP280_ZERO_U8X;
	/* calculate true temperature*/
	v_x1_u32r  = ((((v_uncomp_temperature_s32
	>> SHIFT_RIGHT_3_POSITION) - ((s32)
	p_bmp280->calib_param.dig_T1 << SHIFT_LEFT_1_POSITION))) *
	((s32)p_bmp280->calib_param.dig_T2))
	>> SHIFT_RIGHT_11_POSITION;
	v_x2_u32r  = (((((v_uncomp_temperature_s32
	>> SHIFT_RIGHT_4_POSITION) -
	((s32)p_bmp280->calib_param.dig_T1)) *
	((v_uncomp_temperature_s32 >> SHIFT_RIGHT_4_POSITION) -
	((s32)p_bmp280->calib_param.dig_T1)))
	>> SHIFT_RIGHT_12_POSITION) *
	((s32)p_bmp280->calib_param.dig_T3))
	>> SHIFT_RIGHT_14_POSITION;
	p_bmp280->calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bmp280->calib_param.t_fine *
	BMP20_DEC_TRUE_TEMP_FIVE_DATA
	+ BMP20_DEC_TRUE_TEMP_ONE_TWO_EIGHT_DATA)
	>> SHIFT_RIGHT_8_POSITION;

	return temperature;
}
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure(
s32 *v_uncomp_pressure_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	a_data_u8[0] - Pressure MSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[2] - Pressure LSB
	*/
	u8 a_data_u8[ARRAY_SIZE_THREE] = {
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG,
			a_data_u8, BMP280_THREE_U8X);
			*v_uncomp_pressure_s32 = (s32)(
			(((u32)(a_data_u8[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[INDEX_TWO] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  @return Returns the Actual pressure out put as s32
 *
*/
u32 bmp280_compensate_P_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32r = BMP280_ZERO_U8X;
	s32 v_x2_u32r = BMP280_ZERO_U8X;
	u32 v_pressure_u32 = BMP280_ZERO_U8X;
	/* calculate true pressure*/
	v_x1_u32r = (((s32)p_bmp280->calib_param.t_fine)
	>> SHIFT_RIGHT_1_POSITION) -
	(s32)BMP20_DEC_TRUE_PRESSURE_6_4_0_0_0_DATA;
	v_x2_u32r = (((v_x1_u32r >> SHIFT_RIGHT_2_POSITION) *
	(v_x1_u32r >> SHIFT_RIGHT_2_POSITION))
	>> SHIFT_RIGHT_11_POSITION) *
	((s32)p_bmp280->calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
	((s32)p_bmp280->calib_param.dig_P5))
	<< SHIFT_LEFT_1_POSITION);
	v_x2_u32r = (v_x2_u32r >> SHIFT_RIGHT_2_POSITION) +
	(((s32)p_bmp280->calib_param.dig_P4)
	<< SHIFT_LEFT_16_POSITION);
	v_x1_u32r = (((p_bmp280->calib_param.dig_P3 *
	(((v_x1_u32r >> SHIFT_RIGHT_2_POSITION) *
	(v_x1_u32r >> SHIFT_RIGHT_2_POSITION))
	>> SHIFT_RIGHT_13_POSITION)) >> SHIFT_RIGHT_3_POSITION) +
	((((s32)p_bmp280->calib_param.dig_P2) *
	v_x1_u32r) >> SHIFT_RIGHT_1_POSITION))
	>> SHIFT_RIGHT_18_POSITION;
	v_x1_u32r = ((((BMP20_DEC_TRUE_PRESSURE_3_2_7_6_8_DATA
	+ v_x1_u32r)) *
	((s32)p_bmp280->calib_param.dig_P1))
	>> SHIFT_RIGHT_15_POSITION);
	v_pressure_u32 =
	(((u32)(((s32)BMP20_DEC_TRUE_PRESSURE_1_0_4_8_5_7_6_DATA)
	- v_uncomp_pressure_s32) -
	(v_x2_u32r >> SHIFT_RIGHT_12_POSITION)))
	* BMP20_DEC_TRUE_PRESSURE_3_1_2_5_DATA;
	if (v_pressure_u32
	< BMP20_HEX_TRUE_PRESSURE_8_0_0_0_0_0_0_0_DATA)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_ZERO_U8X)
			v_pressure_u32 =
			(v_pressure_u32 << SHIFT_LEFT_1_POSITION)
			/ ((u32)v_x1_u32r);
		else
			return BMP280_ZERO_U8X;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_ZERO_U8X)
			v_pressure_u32 = (v_pressure_u32 /
			(u32)v_x1_u32r)
			* BMP20_DEC_TRUE_PRESSURE_TWO_DATA;
		else
			return BMP280_ZERO_U8X;
		v_x1_u32r = (((s32)
		p_bmp280->calib_param.dig_P9) *
		((s32)(((v_pressure_u32
		>> SHIFT_RIGHT_3_POSITION)
		* (v_pressure_u32 >> SHIFT_RIGHT_3_POSITION))
		>> SHIFT_RIGHT_13_POSITION)))
		>> SHIFT_RIGHT_12_POSITION;
		v_x2_u32r = (((s32)(v_pressure_u32
		>> SHIFT_RIGHT_2_POSITION)) *
		((s32)p_bmp280->calib_param.dig_P8))
		>> SHIFT_RIGHT_13_POSITION;
		v_pressure_u32 = (u32)
		((s32)v_pressure_u32 +
		((v_x1_u32r + v_x2_u32r +
		p_bmp280->calib_param.dig_P7)
		>> SHIFT_RIGHT_4_POSITION));

	return v_pressure_u32;
}
/*!
 * @brief reads uncompensated pressure and temperature
 *
 *
 * @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 * @param  v_uncomp_temperature_s32: The value of uncompensated temperature.
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure_temperature(
s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the temperature and pressure data
	a_data_u8[0] - Pressure MSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[2] - Pressure LSB
	a_data_u8[3] - Temperature MSB
	a_data_u8[4] - Temperature LSB
	a_data_u8[5] - Temperature LSB
	*/
	u8 a_data_u8[ARRAY_SIZE_SIX] = {BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_PRESSURE_MSB_REG, a_data_u8, BMP280_SIX_U8X);
			/*Pressure*/
			*v_uncomp_pressure_s32 = (s32)(
			(((u32)(a_data_u8[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[INDEX_TWO] >>
			SHIFT_RIGHT_4_POSITION));

			/* Temperature */
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8[INDEX_THREE]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_FOUR]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8[INDEX_FIVE]
			>> SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/*!
 * @brief This API reads the true pressure and temperature
 *
 *
 *  @param  v_pressure_u32 : The value of compensated pressure.
 *  @param  v_temperature_s32 : The value of compensated temperature.
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_pressure_temperature(
u32 *v_pressure_u32,
s32 *v_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = BMP280_ZERO_U8X;
	s32 v_uncomp_temperature_s32 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read uncompensated pressure and temperature*/
			com_rslt = bmp280_read_uncomp_pressure_temperature(
			&v_uncomp_pressure_s32, &v_uncomp_temperature_s32);
			/* read trure pressure and temperature*/
			*v_temperature_s32 = bmp280_compensate_T_int32(
			v_uncomp_temperature_s32);
			*v_pressure_u32 = bmp280_compensate_P_int32(
			v_uncomp_pressure_s32);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param()
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[ARRAY_SIZE_TWENTY_SIX] = {
	BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X,
	BMP280_ZERO_U8X, BMP280_ZERO_U8X, BMP280_ZERO_U8X};
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_DIG_T1_LSB_REG,
			a_data_u8, BMP280_TWENTY_FOUR_U8X);
			/* read calibration values*/
			p_bmp280->calib_param.dig_T1 = (u16)(((
			(u16)((u8)a_data_u8[INDEX_ONE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_ZERO]);
			p_bmp280->calib_param.dig_T2 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_THREE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWO]);
			p_bmp280->calib_param.dig_T3 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_FIVE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_FOUR]);
			p_bmp280->calib_param.dig_P1 = (u16)(((
			(u16)((u8)a_data_u8[INDEX_SEVEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_SIX]);
			p_bmp280->calib_param.dig_P2 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_NINE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_EIGHT]);
			p_bmp280->calib_param.dig_P3 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_ELEVEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TEN]);
			p_bmp280->calib_param.dig_P4 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_THIRTEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWELVE]);
			p_bmp280->calib_param.dig_P5 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_FIVETEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_FOURTEEN]);
			p_bmp280->calib_param.dig_P6 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_SEVENTEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_SIXTEEN]);
			p_bmp280->calib_param.dig_P7 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_NINETEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_EIGHTEEN]);
			p_bmp280->calib_param.dig_P8 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_TWENTY_ONE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWENTY]);
			p_bmp280->calib_param.dig_P9 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_TWENTY_THREE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWENTY_TWO]);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  Skipped
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_temperature(
u8 *v_value_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read temperature over sampling*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);
			/* assign temperature oversampling*/
			p_bmp280->oversamp_temperature = *v_value_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  Skipped
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_temperature(
u8 v_value_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* write over sampling*/
				v_data_u8 =
				BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				 v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, BMP280_ONE_U8X);
				p_bmp280->oversamp_temperature = v_value_u8;
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  Skipped
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_oversamp_pressure(
u8 *v_value_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read pressure over sampling */
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE);

			p_bmp280->oversamp_pressure = *v_value_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  Skipped
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_oversamp_pressure(
u8 v_value_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* write pressure over sampling */
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, BMP280_ONE_U8X);

				p_bmp280->oversamp_pressure = v_value_u8;
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP280_SLEEP_MODE
 *	0x01 and 0x02    | BMP280_FORCED_MODE
 *	0x03             | BMP280_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_power_mode(u8 *v_power_mode_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read the power mode*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
			&v_mode_u8, BMP280_ONE_U8X);
			*v_power_mode_u8 = BMP280_GET_BITSLICE(v_mode_u8,
			BMP280_CTRL_MEAS_REG_POWER_MODE);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP280_SLEEP_MODE
 *	0x01 and 0x02    | BMP280_FORCED_MODE
 *	0x03             | BMP280_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_power_mode(u8 v_power_mode_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			if (v_power_mode_u8 < BMP280_FOUR_U8X) {
				/* write the power mode*/
				v_mode_u8 = (p_bmp280->oversamp_temperature <<
				SHIFT_LEFT_5_POSITION) +
				(p_bmp280->oversamp_pressure <<
				SHIFT_LEFT_2_POSITION) + v_power_mode_u8;
				com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
				&v_mode_u8, BMP280_ONE_U8X);
			} else {
			com_rslt = E_BMP280_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register
 * the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp280_set_softreset().
 *
 * @note Usage Hint : bmp280_set_softreset()
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_soft_rst()
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_SOFT_RESET_CODE;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* write soft reset */
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			BMP280_RST_REG, &v_data_u8, BMP280_ONE_U8X);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to get the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_spi3(u8 *v_enable_disable_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			*v_enable_disable_u8 = BMP280_GET_BITSLICE(
			v_data_u8,
			BMP280_CONFIG_REG_SPI3_ENABLE);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_spi3(u8 v_enable_disable_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, BMP280_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CONFIG_REG_SPI3_ENABLE,
				v_enable_disable_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_SPI3_ENABLE__REG,
				&v_data_u8, BMP280_ONE_U8X);
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP280_FILTER_COEFF_OFF
 *	0x01        | BMP280_FILTER_COEFF_2
 *	0x02        | BMP280_FILTER_COEFF_4
 *	0x03        | BMP280_FILTER_COEFF_8
 *	0x04        | BMP280_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_filter(u8 *v_value_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read filter*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8, BMP280_ONE_U8X);
			*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CONFIG_REG_FILTER);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP280_FILTER_COEFF_OFF
 *	0x01        | BMP280_FILTER_COEFF_2
 *	0x02        | BMP280_FILTER_COEFF_4
 *	0x03        | BMP280_FILTER_COEFF_8
 *	0x04        | BMP280_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(u8 v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* write filter*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_FILTER__REG,
			&v_data_u8, BMP280_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMP280_SET_BITSLICE(
				v_data_u8,
				BMP280_CONFIG_REG_FILTER, v_value_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_FILTER__REG,
				&v_data_u8, BMP280_ONE_U8X);
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP280_STANDBYTIME_1_MS
 *    0x01    | BMP280_STANDBYTIME_63_MS
 *    0x02    | BMP280_STANDBYTIME_125_MS
 *    0x03    | BMP280_STANDBYTIME_250_MS
 *    0x04    | BMP280_STANDBYTIME_500_MS
 *    0x05    | BMP280_STANDBYTIME_1000_MS
 *    0x06    | BMP280_STANDBYTIME_2000_MS
 *    0x07    | BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_standby_durn(u8 *v_standby_durn_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read the standby duration*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_STANDBY_DURN__REG,
			&v_data_u8, BMP280_ONE_U8X);
			*v_standby_durn_u8 = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CONFIG_REG_STANDBY_DURN);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor
 *	in the register 0xF5 bit 5 to 7
 *	@note Normal mode comprises an
 *	automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined
 *	by the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 *	@note bme280_set_standby_durN(BME280_STANDBYTIME_125_MS)
 *
 *
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP280_STANDBYTIME_1_MS
 *    0x01    | BMP280_STANDBYTIME_63_MS
 *    0x02    | BMP280_STANDBYTIME_125_MS
 *    0x03    | BMP280_STANDBYTIME_250_MS
 *    0x04    | BMP280_STANDBYTIME_500_MS
 *    0x05    | BMP280_STANDBYTIME_1000_MS
 *    0x06    | BMP280_STANDBYTIME_2000_MS
 *    0x07    | BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standby_durn(u8 v_standby_durn_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* write the standby duration*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			BMP280_CONFIG_REG_STANDBY_DURN__REG,
			&v_data_u8, BMP280_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8 =
				BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CONFIG_REG_STANDBY_DURN,
				v_standby_durn_u8);
				com_rslt +=
				p_bmp280->BMP280_BUS_WRITE_FUNC(
				p_bmp280->dev_addr,
				BMP280_CONFIG_REG_STANDBY_DURN__REG,
				&v_data_u8, BMP280_ONE_U8X);
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  @param v_work_mode_u8 : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | BMP280_ULTRA_LOW_POWER_MODE
 *    1         | BMP280_LOW_POWER_MODE
 *    2         | BMP280_STANDARD_RESOLUTION_MODE
 *    3         | BMP280_HIGH_RESOLUTION_MODE
 *    4         | BMP280_ULTRA_HIGH_RESOLUTION_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_work_mode(u8 v_work_mode_u8)
{
/* variable used to return communication result*/
BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8 = BMP280_ZERO_U8X;
/* check the p_bmp280 struct pointer as NULL*/
if (p_bmp280 == BMP280_NULL) {
	return  E_BMP280_NULL_PTR;
} else {
	if (v_work_mode_u8 <= BMP280_FOUR_U8X) {
		com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
		p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
		&v_data_u8, BMP280_ONE_U8X);
		if (com_rslt == SUCCESS) {
			switch (v_work_mode_u8) {
			/* write work mode*/
			case BMP280_ULTRA_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_STANDARD_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			}
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
			p_bmp280->oversamp_temperature);
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
			p_bmp280->oversamp_pressure);
			com_rslt += p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
			&v_data_u8, BMP280_ONE_U8X);
		}
	} else {
	com_rslt = E_BMP280_OUT_OF_RANGE;
	}
}
return com_rslt;
}
/*!
 *	@brief This API used to read both
 *	uncompensated pressure and temperature in forced mode
 *
 *
 *  @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 *  @param  v_uncomp_temperature_s32: The value of uncompensated temperature
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP280_RETURN_FUNCTION_TYPE
bmp280_get_forced_uncomp_pressure_temperature(s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_ZERO_U8X;
	u8 v_waittime_u8 = BMP280_ZERO_U8X;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			/* read pressure and temperature*/
			v_data_u8 = (p_bmp280->oversamp_temperature
			<< SHIFT_LEFT_5_POSITION) +
			(p_bmp280->oversamp_pressure << SHIFT_LEFT_2_POSITION) +
			BMP280_FORCED_MODE;
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,	BMP280_CTRL_MEAS_REG,
			&v_data_u8, BMP280_ONE_U8X);
			bmp280_compute_wait_time(&v_waittime_u8);
			p_bmp280->delay_msec(v_waittime_u8);
			com_rslt += bmp280_read_uncomp_pressure_temperature(
			v_uncomp_pressure_s32, v_uncomp_temperature_s32);
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
			p_bmp280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bmp280 struct pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		return  E_BMP280_NULL_PTR;
		} else {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
			p_bmp280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
#ifdef BMP280_ENABLE_FLOAT
/*!
 * @brief This API used to read
 * actual temperature from uncompensated temperature
 * @note Returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return
 *	Actual temperature in floating point
 *
*/
double bmp280_compensate_T_double(s32 v_uncomp_temperature_s32)
{
	double v_x1_u32r = BMP280_ZERO_U8X;
	double v_x2_u32r = BMP280_ZERO_U8X;
	double temperature = BMP280_ZERO_U8X;

	v_x1_u32r  = (((double)v_uncomp_temperature_s32)
	/ BMP280_FLOAT_TRUE_TEMP_1_6_3_8_4_DATA -
	((double)p_bmp280->calib_param.dig_T1)
	/ BMP280_FLOAT_TRUE_TEMP_1_0_2_4_DATA) *
	((double)p_bmp280->calib_param.dig_T2);
	v_x2_u32r  = ((((double)v_uncomp_temperature_s32)
	/ BMP280_FLOAT_TRUE_TEMP_1_3_1_0_7_2_DATA -
	((double)p_bmp280->calib_param.dig_T1)
	/ BMP280_FLOAT_TRUE_TEMP_8_1_9_2_DATA) *
	(((double)v_uncomp_temperature_s32)
	/ BMP280_FLOAT_TRUE_TEMP_1_3_1_0_7_2_DATA -
	((double)p_bmp280->calib_param.dig_T1)
	/ BMP280_FLOAT_TRUE_TEMP_8_1_9_2_DATA)) *
	((double)p_bmp280->calib_param.dig_T3);
	p_bmp280->calib_param.t_fine = (s32)(v_x1_u32r + v_x2_u32r);
	temperature  = (v_x1_u32r + v_x2_u32r)
	/ BMP280_FLOAT_TRUE_TEMP_5_1_2_0_DATA;


	return temperature;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns pressure in Pa as double.
 *	@note Output value of "96386.2"
 *	equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return
 *	Actual pressure in floating point
 *
*/
double bmp280_compensate_P_double(s32 v_uncomp_pressure_s32)
{
	double v_x1_u32r = BMP280_ZERO_U8X;
	double v_x2_u32r = BMP280_ZERO_U8X;
	double pressure = BMP280_ZERO_U8X;

	v_x1_u32r = ((double)p_bmp280->calib_param.t_fine /
	BMP280_FLAOT_TRUE_PRESSURE_2_DATA) -
	BMP280_FLAOT_TRUE_PRESSURE_6_4_0_0_0_DATA;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((double)p_bmp280->calib_param.dig_P6) /
	BMP280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((double)p_bmp280->calib_param.dig_P5)
	* BMP280_FLAOT_TRUE_PRESSURE_2_DATA;
	v_x2_u32r = (v_x2_u32r / BMP280_FLAOT_TRUE_PRESSURE_4_DATA) +
	(((double)p_bmp280->calib_param.dig_P4)
	* BMP280_FLAOT_TRUE_PRESSURE_6_5_5_3_6_DATA);
	v_x1_u32r = (((double)p_bmp280->calib_param.dig_P3) *
	v_x1_u32r * v_x1_u32r
	/ BMP280_FLAOT_TRUE_PRESSURE_5_2_4_2_8_8_DATA +
	((double)p_bmp280->calib_param.dig_P2) * v_x1_u32r)
	/ BMP280_FLAOT_TRUE_PRESSURE_5_2_4_2_8_8_DATA;
	v_x1_u32r = (BMP280_FLAOT_TRUE_PRESSURE_1_DATA + v_x1_u32r
	/ BMP280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA) *
	((double)p_bmp280->calib_param.dig_P1);
	pressure = BMP280_FLAOT_TRUE_PRESSURE_1_0_4_8_5_7_6_DATA
	- (double)v_uncomp_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != BMP280_FLAOT_TRUE_PRESSURE_0_DATA)
		pressure = (pressure - (v_x2_u32r
		/ BMP280_FLAOT_TRUE_PRESSURE_4_0_9_6_DATA)) *
		BMP280_FLAOT_TRUE_PRESSURE_6_2_5_0_DATA / v_x1_u32r;
	else
		return BMP280_ZERO_U8X;
	v_x1_u32r = ((double)p_bmp280->calib_param.dig_P9) *
	pressure * pressure /
	BMP280_FLAOT_TRUE_PRESSURE_2_1_4_7_4_8_3_6_4_8_DATA;
	v_x2_u32r = pressure * ((double)p_bmp280->calib_param.dig_P8)
	/ BMP280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA;
	pressure = pressure + (v_x1_u32r + v_x2_u32r +
	((double)p_bmp280->calib_param.dig_P7))
	/ BMP280_FLAOT_TRUE_PRESSURE_1_6_DATA;

	return pressure;
}
#endif
#if defined(BMP280_ENABLE_INT64) && defined(BMP280_64BITSUPPORT_PRESENT)
/*!
 * @brief This API used to read actual pressure from uncompensated pressure
 * @note returns the value in Pa as unsigned 32 bit
 * integer in Q24.8 format (24 integer bits and
 * 8 fractional bits). Output value of "24674867"
 * represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return actual pressure as 64bit output
 *
*/
u32 bmp280_compensate_P_int64(s32 v_uncomp_pressure_s32)
{
	s64 v_x1_s64r = BMP280_ZERO_U8X;
	s64 v_x2_s64r = BMP280_ZERO_U8X;
	s64 pressure = BMP280_ZERO_U8X;
	v_x1_s64r = ((s64)p_bmp280->calib_param.t_fine) -
	BMP280_TRUE_PRESSURE_1_2_8_0_0_0_DATA;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P5)
	<< SHIFT_LEFT_17_POSITION);
	v_x2_s64r = v_x2_s64r +
	(((s64)p_bmp280->calib_param.dig_P4)
	<< SHIFT_LEFT_35_POSITION);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(s64)p_bmp280->calib_param.dig_P3)
	>> SHIFT_RIGHT_8_POSITION) +
	((v_x1_s64r * (s64)p_bmp280->calib_param.dig_P2)
	<< SHIFT_LEFT_12_POSITION);
	v_x1_s64r = (((((s64)BMP280_TRUE_PRESSURE_1_DATA)
	<< SHIFT_LEFT_47_POSITION) + v_x1_s64r)) *
	((s64)p_bmp280->calib_param.dig_P1)
	>> SHIFT_RIGHT_33_POSITION;
	pressure = BMP280_TRUE_PRESSURE_1_0_4_8_5_7_6_DATA
	- v_uncomp_pressure_s32;
	if (v_x1_s64r != BMP280_ZERO_U8X)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure
			<< SHIFT_LEFT_31_POSITION) - v_x2_s64r)
			* BMP280_TRUE_PRESSURE_3_1_2_5_DATA), v_x1_s64r);
		#else
			pressure = (((pressure
			<< SHIFT_LEFT_31_POSITION) - v_x2_s64r)
			* BMP280_TRUE_PRESSURE_3_1_2_5_DATA) / v_x1_s64r;
		#endif
	else
		return BMP280_ZERO_U8X;
	v_x1_s64r = (((s64)p_bmp280->calib_param.dig_P9) *
	(pressure >> SHIFT_RIGHT_13_POSITION) *
	(pressure >> SHIFT_RIGHT_13_POSITION))
	>> SHIFT_RIGHT_25_POSITION;
	v_x2_s64r = (((s64)p_bmp280->calib_param.dig_P8) *
	pressure) >> SHIFT_RIGHT_19_POSITION;
	pressure = ((pressure + v_x1_s64r + v_x2_s64r)
	>> SHIFT_RIGHT_8_POSITION) +
	(((s64)p_bmp280->calib_param.dig_P7)
	<< SHIFT_LEFT_4_POSITION);
	return (u32)pressure;
}
#endif
/*!
 * @brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  @param v_delaytime_u8r: The value of delay time
 *
 *
 *  @return 0
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_compute_wait_time(u8
*v_delaytime_u8r)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX *
	(((BMP280_ONE_U8X << p_bmp280->oversamp_temperature)
	>> SHIFT_RIGHT_1_POSITION) +
	((BMP280_ONE_U8X << p_bmp280->oversamp_pressure)
	>> SHIFT_RIGHT_1_POSITION)) +
	(p_bmp280->oversamp_pressure ? T_SETUP_PRESSURE_MAX : BMP280_ZERO_U8X)
	+ BMP280_FIVETEEN_U8X)
	/ BMP280_SIXTEEN_U8X;
	return com_rslt;
}
