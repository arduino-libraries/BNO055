/*
 ***************************************************************************
 *
 *  bno055.c - part of sample SW for using BNO055 with Arduino
 * 
 * Usage:        BNO055 Sensor Driver Source File
 * 
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 * Copyright (C) 2014 Bosch Sensortec GmbH
 *
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **************************************************************************/
/*	Date: 2014/01/07
 *	 Revision: 1.2
 *
 */


/****************************************************************************/
/*! file <BNO055 >
    brief <Sensor driver for BNO055> */
#include "BNO055.h"

/* user defined code to be added here ... */
static struct bno055_t *p_bno055;
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief
 *        This function initialises the structure pointer
 *        and assigns the I2C address.
 *
 *
 *
 *
 *
 *  \param  bno055_t *bno055 structure pointer.
 *
 *
 *
 *  \return communication results.
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
	{
		BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
		unsigned char a_data_u8r = BNO055_Zero_U8X;
		unsigned char a_SWID_u8r[2] = {0, 0};

		p_bno055 = bno055;
		p_bno055->dev_addr = BNO055_I2C_ADDR;

		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_CHIP_ID__REG, &a_data_u8r, 1);
		p_bno055->chip_id = a_data_u8r;
		
		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_ACC_REV_ID__REG, &a_data_u8r, 1);
		p_bno055->accel_revision_id = a_data_u8r;

		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_MAG_REV_ID__REG, &a_data_u8r, 1);
		p_bno055->mag_revision_id = a_data_u8r;

		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_GYR_REV_ID__REG, &a_data_u8r, 1);
		p_bno055->gyro_revision_id = a_data_u8r;

		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_BL_Rev_ID__REG, &a_data_u8r, 1);
		p_bno055->bootloader_revision_id = a_data_u8r;

		comres = p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
		BNO055_SW_REV_ID_LSB__REG, a_SWID_u8r, 2);
		a_SWID_u8r[0] = BNO055_GET_BITSLICE(a_SWID_u8r[0],
		BNO055_SW_REV_ID_LSB);
		p_bno055->sw_revision_id = (BNO055_U16)
		((((BNO055_U16)((signed char)a_SWID_u8r[1])) <<
		BNO055_SHIFT_8_POSITION) | (a_SWID_u8r[0]));

		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_Page_ID__REG, &a_data_u8r, 1);
		p_bno055->page_id = a_data_u8r;

		return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                   the data is written in the corresponding register address
 *
 *
 *
 *
 *  \param unsigned char addr, unsigned char data, unsigned char len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
 *
 *
 *
 *  \return communication results.
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(unsigned char addr,
unsigned char *data, unsigned char len) {
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr, addr, data, len);
		}
		return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads the data from
 *                        the given register address
 *
 *
 *
 *
 *  \param unsigned char addr, unsigned char *data, unsigned char len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_register(unsigned char addr,
unsigned char *data, unsigned char len)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr, addr, data, len);
		}
		return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads chip id
 *                          from location 00h
 *
 *
 *
 *
 *  \param unsigned char *chip_id   : Pointer holding the chip id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(unsigned char *chip_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_CHIP_ID__REG, &a_data_u8r, 1);
	*chip_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads software revision id
 *                          from location 04h and 05h
 *
 *
 *
 *
 *  \param BNO055_U16 *sw_id   : Pointer holding the SW revision id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_revision_id(BNO055_U16 *sw_id)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_SW_REV_ID_LSB__REG, a_data_u8r, 2);
		a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
		BNO055_SW_REV_ID_LSB);
		*sw_id = (BNO055_U16)
		((((BNO055_U16)((signed char)a_data_u8r[1])) <<
		BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads page id
 *                          from location 07h
 *
 *
 *
 *
 *  \param unsigned char *page_id   : Pointer holding the page id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(unsigned char *pg_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_Page_ID__REG, &a_data_u8r, 1);
	*pg_id = a_data_u8r;
	p_bno055->page_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API writes the page id register 07h
 *
 *
 *
 *
 *
 *  \param unsigned char page_id
 *         PAGE_ZERO -> 0X00
 *          PAGE_ONE -> 0X01
 *
 *  \return Communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(unsigned char page_id)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char pg_id = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_Page_ID__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_Page_ID, page_id);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_Page_ID__REG, &v_data_u8r, 1);
			bno055_read_page_id(&pg_id);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads accel revision id
 *                          from location 01h
 *
 *
 *
 *
 *  \param unsigned char *acc_rev_id   : Pointer holding the accel revision id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_revision_id(
unsigned char *acc_rev_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_ACC_REV_ID__REG, &a_data_u8r, 1);
	*acc_rev_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads mag revision id
 *                          from location 02h
 *
 *
 *
 *
 *  \param unsigned char *mag_rev_id   : Pointer holding the mag revision id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_revision_id(
unsigned char *mag_rev_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_MAG_REV_ID__REG, &a_data_u8r, 1);
	*mag_rev_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads gyro revision id
 *                          from location 03h
 *
 *
 *
 *
 *  \param unsigned char *gyr_rev_id   : Pointer holding the gyro revision id
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_revision_id(
unsigned char *gyr_rev_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_GYR_REV_ID__REG, &a_data_u8r, 1);
	*gyr_rev_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads boot loader revision id
 *                          from location 06h
 *
 *
 *
 *
 *  \param unsigned char *bl_rev_id   : Pointer holding
 *                                      the boot loader revision id
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_bootloader_revision_id(
unsigned char *bl_rev_id)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	comres = p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_BL_Rev_ID__REG, &a_data_u8r, 1);
	*bl_rev_id = a_data_u8r;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads acceleration data X values
 *                          from location 08h and 0dh
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(BNO055_S16 *acc_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_DATA_X_MSB_VALUEX);
			*acc_x = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values
 *                          from location 08h and 0dh
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(BNO055_S16 *acc_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_DATA_Y_MSB_VALUEY);
			*acc_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads acceleration data Z values
 *                          from location 08h and 0dh
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(BNO055_S16 *acc_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_DATA_Z_MSB_VALUEZ);
			*acc_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z of accelerometer
 *							from location 08h to 0Dh
 *
 *
 *
 *
 *  \param
 *      bno055_accel *acc   :  Pointer holding the bno055_accel
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(struct bno055_accel *acc)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_DATA_X_LSB_VALUEX__REG, a_data_u8r, 7);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_DATA_X_MSB_VALUEX);
			acc->x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_ACC_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_ACC_DATA_Y_MSB_VALUEY);
			acc->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_ACC_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_ACC_DATA_Z_MSB_VALUEZ);
			acc->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
 /*****************************************************************************
 * Description: *//**\brief This API reads magnetometer data X values
 *                          from location 0Eh and 0Fh
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(BNO055_S16 *mag_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			*mag_x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads magnetometer data Y values
 *                          from location 10h and 11h
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(BNO055_S16 *mag_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			*mag_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads magnetometer data Z values
 *                          from location 12h and 13h
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(BNO055_S16 *mag_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			*mag_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z of magnetometer
 *							from location 0Eh to 13h
 *
 *
 *
 *
 *  \param
 *      bno055_mag *mag   :  Pointer holding the bno055_mag
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag *mag)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG, a_data_u8r, 7);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			mag->x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			mag->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			mag->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
 /*****************************************************************************
 * Description: *//**\brief This API reads gyroscope data X values
 *                          from location 14h and 15h
 *
 *
 *
 *
 *  \param BNO055_S16  *gyr_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(BNO055_S16 *gyr_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_DATA_X_MSB_VALUEX);
			*gyr_x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads gyroscope data Y values
 *                          from location 16h and 17h
 *
 *
 *
 *
 *  \param BNO055_S16  *gyr_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(BNO055_S16 *gyr_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_DATA_Y_MSB_VALUEY);
			*gyr_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads gyroscope data Z values
 *                          from location 18h and 19h
 *
 *
 *
 *
 *  \param short  *gyr_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(BNO055_S16 *gyr_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_DATA_Z_MSB_VALUEZ);
			*gyr_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z of gyroscope
 *							from location 14h to 19h
 *
 *
 *
 *
 *  \param
 *      bno055_gyro *gyr   :  Pointer holding the bno055_gyro
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro *gyr)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_DATA_X_LSB_VALUEX__REG, a_data_u8r, 7);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_DATA_X_MSB_VALUEX);
			gyr->x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_GYR_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_GYR_DATA_Y_MSB_VALUEY);
			gyr->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_GYR_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_GYR_DATA_Z_MSB_VALUEZ);
			gyr->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads Euler data H values
 *                          from location 1Ah and 1Bh
 *
 *
 *
 *
 *  \param BNO055_S16  *eul_h   : Pointer holding the H-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(BNO055_S16 *eul_h)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EUL_HEADING_LSB_VALUEH__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE
			(a_data_u8r[0],
			BNO055_EUL_HEADING_LSB_VALUEH);
			a_data_u8r[1] = BNO055_GET_BITSLICE
			(a_data_u8r[1],
			BNO055_EUL_HEADING_MSB_VALUEH);
			*eul_h = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads Euler data R values
 *                          from location 1Ch and 1Dh
 *
 *
 *
 *
 *  \param BNO055_S16  *eul_r   : Pointer holding the R-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(BNO055_S16 *eul_r)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EUL_ROLL_LSB_VALUER__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EUL_ROLL_LSB_VALUER);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EUL_ROLL_MSB_VALUER);
			*eul_r = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Euler data P values
 *                          from location 1Eh and 1Fh
 *
 *
 *
 *
 *  \param BNO055_S16  *eul_p   : Pointer holding the P-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(BNO055_S16 *eul_p)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EUL_PITCH_LSB_VALUEP__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EUL_PITCH_LSB_VALUEP);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EUL_PITCH_MSB_VALUEP);
			*eul_p = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data H,R,P of Euler
 *							from location 1Ah to 1Fh
 *
 *
 *
 *
 *  \param
 *      bno055_euler *eul   :  Pointer holding the bno055_euler
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(struct bno055_euler *eul)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EUL_HEADING_LSB_VALUEH__REG, a_data_u8r, 7);
			/* Data H*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EUL_HEADING_LSB_VALUEH);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EUL_HEADING_MSB_VALUEH);
			eul->h = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data R*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_EUL_ROLL_LSB_VALUER);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_EUL_ROLL_MSB_VALUER);
			eul->r = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data P*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_EUL_PITCH_LSB_VALUEP);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_EUL_PITCH_MSB_VALUEP);
			eul->p = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
 /*****************************************************************************
 * Description: *//**\brief This API reads quaternion data W values
 *                          from location 20h and 21h
 *
 *
 *
 *
 *  \param BNO055_S16  *quan_w   : Pointer holding the W-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(BNO055_S16 *quan_w)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUA_DATA_W_LSB_VALUEW__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUA_DATA_W_LSB_VALUEW);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUA_DATA_W_MSB_VALUEW);
			*quan_w = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
 /*****************************************************************************
 * Description: *//**\brief This API reads quaternion data X values
 *                          from location 22h and 23h
 *
 *
 *
 *
 *  \param BNO055_S16  *quan_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(BNO055_S16 *quan_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUA_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUA_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUA_DATA_X_MSB_VALUEX);
			*quan_x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads quaternion data Y values
 *                          from location 24h and 25h
 *
 *
 *
 *
 *  \param BNO055_S16  *quan_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(BNO055_S16 *quan_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUA_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE
			(a_data_u8r[0],
			BNO055_QUA_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE
			(a_data_u8r[1],
			BNO055_QUA_DATA_Y_MSB_VALUEY);
			*quan_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads quaternion data Z values
 *                          from location 26h and 27h
 *
 *
 *
 *
 *  \param BNO055_S16  *quan_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(BNO055_S16 *quan_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUA_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUA_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUA_DATA_Z_MSB_VALUEZ);
			*quan_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data W,X,Y,Z of quaternion
 *							from location 20h to 27h
 *
 *
 *
 *
 *  \param
 *      bno055_quaternion *qur   :  Pointer holding the bno055_quaternion
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(
struct bno055_quaternion *qur)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUA_DATA_W_LSB_VALUEW__REG, a_data_u8r, 7);
			/* Data W*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUA_DATA_W_LSB_VALUEW);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUA_DATA_W_MSB_VALUEW);
			qur->w = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data X*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_QUA_DATA_X_LSB_VALUEX);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_QUA_DATA_X_MSB_VALUEX);
			qur->x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Y*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_QUA_DATA_Y_LSB_VALUEY);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_QUA_DATA_Y_MSB_VALUEY);
			qur->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
			/* Data Z*/
			a_data_u8r[6] = BNO055_GET_BITSLICE(a_data_u8r[6],
			BNO055_QUA_DATA_Z_LSB_VALUEZ);
			a_data_u8r[7] = BNO055_GET_BITSLICE(a_data_u8r[7],
			BNO055_QUA_DATA_Z_MSB_VALUEZ);
			qur->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[7])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[6]));
		} else {
		return ERROR1;
		}
	}
		return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads linear acceleration data X values
 *                          from location 28h and 29h
 *
 *
 *
 *
 *  \param BNO055_S16  *lia_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(BNO055_S16 *lia_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LIA_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LIA_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LIA_DATA_X_MSB_VALUEX);
			*lia_x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads linear acceleration data Y values
 *                          from location 2Ah and 2Bh
 *
 *
 *
 *
 *  \param BNO055_S16  *lia_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(BNO055_S16 *lia_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LIA_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LIA_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LIA_DATA_Y_MSB_VALUEY);
			*lia_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads linear acceleration data Z values
 *                          from location 2Ch and 2Dh
 *
 *
 *
 *
 *  \param BNO055_S16  *lia_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(BNO055_S16 *lia_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LIA_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LIA_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LIA_DATA_Z_MSB_VALUEZ);
			*lia_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z of linear acceleration
 *							from location 28h to 2Dh
 *
 *
 *
 *
 *  \param
 *      bno055_linear_accel *lia   :  Pointer holding the bno055_linear_accel
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(
struct bno055_linear_accel *lia)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LIA_DATA_X_LSB_VALUEX__REG, a_data_u8r, 7);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LIA_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LIA_DATA_X_MSB_VALUEX);
			lia->x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_LIA_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_LIA_DATA_Y_MSB_VALUEY);
			lia->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_LIA_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_LIA_DATA_Z_MSB_VALUEZ);
			lia->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads gravity data X values
 *                          from location 2Eh and 2Fh
 *
 *
 *
 *
 *  \param BNO055_S16  *grv_x   : Pointer holding the X-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(BNO055_S16 *grv_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRV_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRV_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRV_DATA_X_MSB_VALUEX);
			*grv_x = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief This API reads gravity data Y values
 *                          from location 30h and 31h
 *
 *
 *
 *
 *  \param BNO055_S16  *grv_y   : Pointer holding the Y-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(BNO055_S16 *grv_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRV_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRV_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRV_DATA_Y_MSB_VALUEY);
			*grv_y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads gravity data Z values
 *                          from location 32h and 33h
 *
 *
 *
 *
 *  \param BNO055_S16  *grv_z   : Pointer holding the Z-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(BNO055_S16 *grv_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRV_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRV_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRV_DATA_Z_MSB_VALUEZ);
			*grv_z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 * Description: *//**\brief Reads data X,Y,Z of gravity
 *							from location 2Eh to 33h
 *
 *
 *
 *
 *  \param
 *      bno055_linear_accel *grvt   :  Pointer holding the bno055_linear_accel
 *
 *
 *  \return
 *      result of communication routines
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(
struct bno055_gravity *grvt)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[7] = {0, 0, 0, 0, 0, 0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRV_DATA_X_LSB_VALUEX__REG, a_data_u8r, 7);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRV_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRV_DATA_X_MSB_VALUEX);
			grvt->x = (BNO055_S16)(((BNO055_S16)
			((signed char)a_data_u8r[1]) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_GRV_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_GRV_DATA_Y_MSB_VALUEY);
			grvt->y = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_GRV_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_GRV_DATA_Z_MSB_VALUEZ);
			grvt->z = (BNO055_S16)((((BNO055_S16)
			((signed char)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads temperature data
 *                          from location 34h
 *
 *
 *
 *
 *  \param BNO055_S16  *temp   : Pointer holding the temperature-DATA
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_temperature_data(BNO055_S16 *temp)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP__REG, &a_data_u8r, 1);
			*temp = a_data_u8r;
		} else {
		return ERROR1;

		}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads mag calibration status register byte from 35h
 *
 *
 *
 *
 *  \param
 *      unsigned char *mag_calib : Pointer holding the mag_calib status register
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_magcalib_status(
unsigned char *mag_calib)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_CALIB_STAT__REG, &v_data_u8r, 1);
			*mag_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_MAG_CALIB_STAT);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel calibration status register byte from 35h
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_calib : Pointer holding
 *                     the accel_calib status register
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accelcalib_status(
unsigned char *acc_calib)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_CALIB_STAT__REG, &v_data_u8r, 1);
			*acc_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACC_CALIB_STAT);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro calibration status register byte from 35h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyr_calib : Pointer holding
 *          the gyro_calib status register
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyrocalib_status(
unsigned char *gyr_calib)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_CALIB_STAT__REG, &v_data_u8r, 1);
			*gyr_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_CALIB_STAT);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/**************************************************************************
 * Description: *//**\brief Reads system calibration status
 *								register byte from 35h
 *
 *
 *
 *  \param
 *      unsigned char *sys_calib : Pointer holding the sys_calib
 *				       status register
 *
 *  \return
 *      Result of bus communication function
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_syscalib_status(
unsigned char *sys_calib)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_CALIB_STAT__REG, &v_data_u8r, 1);
			*sys_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SYS_CALIB_STAT);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads ST result of
 *                          accelerometer register byte from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *st_acc : Pointer holding the
 *                              ST result of accelerometer
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_accel(
unsigned char *st_acc)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ST_ACC__REG, &v_data_u8r, 1);
			*st_acc =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ST_ACC);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads ST result of
 *                          magnetometer register byte from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *st_mag : Pointer holding the
 *                              ST result of magnetometer
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_mag(
unsigned char *st_mag)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ST_MAG__REG, &v_data_u8r, 1);
			*st_mag =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ST_MAG);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads ST result of
 *                          gyroscope register byte from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *st_gyr : Pointer holding the
 *                              ST result of gyroscope
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_gyro(
unsigned char *st_gyr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ST_GYR__REG, &v_data_u8r, 1);
			*st_gyr =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ST_GYR);
		} else {

		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /****************************************************************************
 * Description: *//**\brief Reads ST result of
 *                          MCU register byte from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *st_mcu : Pointer holding the
 *                              ST result of MCU
 *
 *
 *  \return
 *      Result of bus communication function
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_mcu(
unsigned char *st_mcu)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ST_MCU__REG, &v_data_u8r, 1);
			*st_mcu =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ST_MCU);
		} else {
		return ERROR1;

	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads gyro any motion interrupt status
 *                          from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyr_anymotion : Pointer holding the
 *                              gyro any motion interrupt status
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_gyro_anymotion(
unsigned char *gyr_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_GYRO_AM__REG, &v_data_u8r, 1);
			*gyr_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_GYRO_AM);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads gyro high rate interrupt status
 *                          from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyr_highrate : Pointer holding the
 *                             high rate interrupt status
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_gyro_highrate(
unsigned char *gyr_highrate)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_GYRO_HIGH_RATE__REG, &v_data_u8r, 1);
			*gyr_highrate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_GYRO_HIGH_RATE);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads accel high g interrupt status
 *                          from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_highg : Pointer holding the
 *                             accel high g interrupt status
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_highg(
unsigned char *acc_highg)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACC_HIGH_G__REG, &v_data_u8r, 1);
			*acc_highg =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_ACC_HIGH_G);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads accel any motion interrupt status
 *                          from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_anymotion : Pointer holding the
 *                             accel any motion interrupt status
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_anymotion(
unsigned char *acc_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACC_AM__REG, &v_data_u8r, 1);
			*acc_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_INT_STAT_ACC_AM);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads accel no motion interrupt status
 *                          from 36h
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_nomotion : Pointer holding the
 *                             accel no motion interrupt status
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_nomotion(
unsigned char *acc_nomotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACC_NM__REG, &v_data_u8r, 1);
			*acc_nomotion =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_INT_STAT_ACC_NM);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads system status code
 *                          from 39h
 *
 *
 *
 *
 *  \param
 *      unsigned char *sys_status : Pointer holding the
 *                             system status code
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_system_status_code(
unsigned char *sys_status)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYSTEM_STATUS_CODE__REG, &v_data_u8r, 1);
			*sys_status =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYSTEM_STATUS_CODE);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

 /*****************************************************************************
 * Description: *//**\brief Reads system error code
 *                          from 3Ah
 *
 *
 *
 *
 *  \param
 *      unsigned char *sys_error : Pointer holding the
 *                             system error code
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_system_error_code(
unsigned char *sys_error)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYSTEM_ERROR_CODE__REG, &v_data_u8r, 1);
			*sys_error =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYSTEM_ERROR_CODE);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel unit
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_unit : Pointer holding the
 *                                accel unit
 *      0 m/s2
 *		1 mG
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(
unsigned char *acc_unit)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_UNIT__REG, &v_data_u8r, 1);
			*acc_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACC_UNIT);
		} else {
		return ERROR1;
	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel unit register 3Bh
 * (0th bit)
 *
 *
 *
 *
 *  \param unsigned char acc_unit
 *
 *      Accel unit
 *      0 m/s2
 *		1 mG
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(
unsigned char acc_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_UNIT__REG, &v_data_u8r, 1);
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACC_UNIT, acc_unit);
				comres = p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_UNIT__REG, &v_data_u8r, 1);
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro unit
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyr_unit : Pointer holding the
 *                                gyro unit
 *      0 Dps
 *		1 Rps
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(
unsigned char *gyr_unit)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_UNIT__REG, &v_data_u8r, 1);
			*gyr_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_UNIT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro unit register 3Bh
 * (1st bit)
 *
 *
 *
 *
 *  \param unsigned char gyr_unit
 *
 *      Gyro unit
 *      0 Dps
 *		1 Rps
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(unsigned char gyr_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_UNIT__REG, &v_data_u8r, 1);
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYR_UNIT, gyr_unit);
				comres = p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_UNIT__REG, &v_data_u8r, 1);
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief Reads Euler unit
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *eul_unit : Pointer holding the
 *                                Euler unit
 *      0 Degrees
 *		1 Radians
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(
unsigned char *eul_unit)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EUL_UNIT__REG, &v_data_u8r, 1);
			*eul_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_EUL_UNIT);
		} else {
		return ERROR1;

	}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Euler unit register 3Bh
 * (2nd bit)
 *
 *
 *
 *
 *  \param unsigned char eul_unit
 *
 *      Euler unit
 *      0 Degrees
 *		1 Radians
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(unsigned char eul_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
	status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_EUL_UNIT__REG, &v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_EUL_UNIT, eul_unit);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_EUL_UNIT__REG, &v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads Tilt unit
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *tilt_unit : Pointer holding the
 *                                Tilt unit
 *      0 Degrees
 *		1 Radians
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(
unsigned char *tilt_unit)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TILT_UNIT__REG, &v_data_u8r, 1);
			*tilt_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TILT_UNIT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Tilt unit register 3Bh
 * (2nd bit)
 *
 *
 *
 *
 *  \param unsigned char tilt_unit
 *
 *      Tilt unit
 *      0 Degrees
 *		1 Radians
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(unsigned char tilt_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
	status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_TILT_UNIT__REG, &v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TILT_UNIT, tilt_unit);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TILT_UNIT__REG, &v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads temperature unit
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *temp_unit : Pointer holding the
 *                                temperature unit
 *      0 Deg Centigrade
 *		1 Deg Fahrenheit
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_temperature_unit(
unsigned char *temp_unit)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_UNIT__REG, &v_data_u8r, 1);
			*temp_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TEMP_UNIT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the temperature unit register 3Bh
 * (2nd bit)
 *
 *
 *
 *
 *  \param unsigned char temp_unit
 *
 *      Temperature unit
 *      0 Deg Centigrade
 *		1 Deg Fahrenheit
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_temperature_unit(
unsigned char temp_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_UNIT__REG, &v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_UNIT, temp_unit);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_UNIT__REG, &v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description:*//**\brief Reads data out put format of android and windows OS
 *                          register byte from 3Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *dof : Pointer holding the
 *                              data out put format of android and windows OS
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(
unsigned char *dof)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_OUTPUT_FORMAT__REG, &v_data_u8r, 1);
			*dof =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_DATA_OUTPUT_FORMAT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data out put format of
 *                                    android and windows OS 3Bh
 * (7th bit)
 *
 *
 *
 *
 *  \param unsigned char dof
 *
 *      Data output format
 *      0 Windows
 *		1 Android
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(unsigned char dof)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_DATA_OUTPUT_FORMAT__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_DATA_OUTPUT_FORMAT, dof);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_DATA_OUTPUT_FORMAT__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of accel
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *acc_datasel : Pointer holding the
 *                                data select of accel
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_data_select(
unsigned char *acc_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_ACC__REG, &v_data_u8r, 1);
			*acc_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_ACC);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of accel
 *                                    from 3Ch
 * (0th bit)
 *
 *
 *
 *
 *  \param unsigned char acc_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_data_select(
unsigned char acc_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_ACC__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_ACC, acc_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_ACC__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Mag
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *mag_datasel : Pointer holding the
 *                                data select of Mag
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_select(
unsigned char *mag_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_MAG__REG, &v_data_u8r, 1);
			*mag_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_MAG);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of Mag
 *                                    from 3Ch
 * (1st bit)
 *
 *
 *
 *
 *  \param unsigned char mag_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_select(
unsigned char mag_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_MAG__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_MAG, mag_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_MAG__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Gyro
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_datasel : Pointer holding the
 *                                data select of Gyro
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_data_select(
unsigned char *gyro_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GYR__REG, &v_data_u8r, 1);
			*gyro_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_GYR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of Gyro
 *                                    from 3Ch
 * (2nd bit)
 *
 *
 *
 *
 *  \param unsigned char gyro_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_data_select(
unsigned char gyro_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GYR__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_GYR, gyro_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GYR__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Euler
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *eul_datasel : Pointer holding the
 *                                data select of Euler
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_data_select(
unsigned char *eul_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_EUL__REG, &v_data_u8r, 1);
			*eul_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_EUL);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of Euler
 *                                    from 3Ch
 * (3rd bit)
 *
 *
 *
 *
 *  \param unsigned char eul_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_data_select(
unsigned char eul_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_EUL__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_EUL, eul_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_EUL__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Quaternion
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *qur_datasel : Pointer holding the
 *                                data select of Quaternion
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_quaternion_data_select(
unsigned char *qur_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_QUA__REG, &v_data_u8r, 1);
			*qur_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_QUA);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of Quaternion
 *                                    from 3Ch
 * (4th bit)
 *
 *
 *
 *
 *  \param unsigned char qur_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_quaternion_data_select(
unsigned char qur_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_QUA__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_QUA, qur_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_QUA__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Linear acceleration
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *lia_datasel : Pointer holding the
 *                                data select of Linear acceleration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_linear_accel_data_select(
unsigned char *lia_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_LINEAR_ACC__REG, &v_data_u8r, 1);
			*lia_datasel =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_LINEAR_ACC);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of
 *                                  Linear acceleration  from 3Ch
 * (5th bit)
 *
 *
 *
 *
 *  \param unsigned char lia_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_linear_accel_data_select(
unsigned char lia_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_LINEAR_ACC__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_LINEAR_ACC, lia_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_LINEAR_ACC__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of Gravity
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *grv_datasel : Pointer holding the
 *                                data select of Gravity
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gravity_data_select(
unsigned char *grv_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GRV__REG, &v_data_u8r, 1);
			*grv_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_GRV);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of
 *                                  Gravity from 3Ch
 * (6th bit)
 *
 *
 *
 *
 *  \param unsigned char grv_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gravity_data_select(
unsigned char grv_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GRV__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_GRV, grv_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_GRV__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data select of temperature
 *                          register byte from 3Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *temp_datasel : Pointer holding the
 *                                data select of temperature
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_temperature_data_select(
unsigned char *temp_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_TEMP__REG, &v_data_u8r, 1);
			*temp_datasel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_DATA_SEL_TEMP);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data select of
 *                                  temperature from 3Ch
 * (7th bit)
 *
 *
 *
 *
 *  \param unsigned char temp_datasel
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_temperature_data_select(
unsigned char temp_datasel)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_TEMP__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_DATA_SEL_TEMP, temp_datasel);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_SEL_TEMP__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads data output rate
 *                          register byte from 3Dh
 *
 *
 *  \param
 *      unsigned char *data_out_rate : Pointer holding the
 *                                data output rate
 *
 *
 *
 * data_out_rate
 *		FASTEST_MODE_1 - 0x00
 *		FASTEST_MODE_2 - 0x01
 *		GAME_MODE	   - 0x02
 *		UI_MODE		   - 0x03
 *		NORMALA_MODE   - 0x04
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_rate(
unsigned char *data_out_rate)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_OUTPUT_DATA_RATE__REG, &v_data_u8r, 1);
			*data_out_rate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_OUTPUT_DATA_RATE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the data out put rate in
 *                                    3Dh register
 *
 *
 *  \param unsigned char data_out_rate
 *
 *
 * data_out_rate
 *		FASTEST_MODE_1 - 0x00
 *		FASTEST_MODE_2 - 0x01
 *		GAME_MODE	   - 0x02
 *		UI_MODE		   - 0x03
 *		NORMALA_MODE   - 0x04
 *
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_rate(
unsigned char data_out_rate)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_OUTPUT_DATA_RATE__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OUTPUT_DATA_RATE, data_out_rate);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_OUTPUT_DATA_RATE__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads operation mode
 *                          register byte from 3Dh(0 to 3 bit)
 *
 *
 *
 *
 *  \param
 *      unsigned char *op_mode : Pointer holding the
 *                                operation mode
 *Operation mode:
 *SLEEP MODE
 *0x00 - OPERATION_MODE_CONFIG
 *SENSOR MODE
 *0x01 - OPERATION_MODE_ACCONLY
 *0x02 - OPERATION_MODE_MAGONLY
 *0x03 - OPERATION_MODE_GYRONLY
 *0x04 - OPERATION_MODE_ACCMAG
 *0x05 - OPERATION_MODE_ACCGYRO
 *0x06 - OPERATION_MODE_MAGGYRO
 *0x07 - OPERATION_MODE_AMG
 *FUSION MODE
 *0x08 - OPERATION_MODE_IMUPLUS
 *0x09 - OPERATION_MODE_COMPASS
 *0x0A - OPERATION_MODE_M4G
 *0x0B - OPERATION_MODE_NDOF_FMC_OFF
 *0x0C - OPERATION_MODE_NDOF
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(
unsigned char *op_mode)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
			*op_mode =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_OPERATION_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the operation mode
 *                                    in 3Bh(0th bit to 3rd bit)
 *
 *
 *
 *
 *
 *  \param unsigned char opr_mode
 *
 *Operation mode:
 *SLEEP MODE
 *0x00 - OPERATION_MODE_CONFIG
 *SENSOR MODE
 *0x01 - OPERATION_MODE_ACCONLY
 *0x02 - OPERATION_MODE_MAGONLY
 *0x03 - OPERATION_MODE_GYRONLY
 *0x04 - OPERATION_MODE_ACCMAG
 *0x05 - OPERATION_MODE_ACCGYRO
 *0x06 - OPERATION_MODE_MAGGYRO
 *0x07 - OPERATION_MODE_AMG
 *FUSION MODE
 *0x08 - OPERATION_MODE_IMUPLUS
 *0x09 - OPERATION_MODE_COMPASS
 *0x0A - OPERATION_MODE_M4G
 *0x0B - OPERATION_MODE_NDOF_FMC_OFF
 *0x0C - OPERATION_MODE_NDOF
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(unsigned char opr_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode == OPERATION_MODE_CONFIG) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_OPERATION_MODE, opr_mode);
				comres = p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
			} else {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_OPERATION_MODE, OPERATION_MODE_CONFIG);
				bno055_write_register(
				BNO055_OPERATION_MODE__REG,
				&v_data_u8r, 1);
				if (opr_mode !=
				OPERATION_MODE_CONFIG) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_OPERATION_MODE, opr_mode);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, 1);
				}
			}
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads Power mode
 *                          register byte from 3Eh
 *
 *
 *
 *
 *  \param
 *      unsigned char *pwm : Pointer holding the
 *                              power mode
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_powermode(
unsigned char *pwm)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_POWER_MODE__REG, &v_data_u8r, 1);
			*pwm =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_POWER_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the power mode register 3Eh
 * (0 to 1 bits)
 *
 *
 *
 *
 *  \param unsigned char power mode
 *
 *      power mode[0....7]
 *      0x00 - Normal mode
 *		0x01 - Low Power mode
 *		0x02 - Suspend mode
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_powermode(unsigned char powermode)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_POWER_MODE__REG, &v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_POWER_MODE, powermode);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_POWER_MODE__REG, &v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads reset interrupt
 *                          register byte from 3Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *rst_int : Pointer holding the
 *                              reset interrupt
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_reset_int(
unsigned char *rst_int)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_INT__REG, &v_data_u8r, 1);
			*rst_int =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_RST_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the reset interrupt register 3Fh
 * (6th bit)
 *
 *
 *
 *
 *  \param unsigned char rst_int
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_reset_int(unsigned char rst_int)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_INT__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_RST_INT, rst_int);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_INT__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads reset system
 *                          register byte from 3Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *rst_sys : Pointer holding the
 *                             reset system
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_reset_sys(
unsigned char *rst_sys)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_SYS__REG, &v_data_u8r, 1);
			*rst_sys =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_RST_SYS);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief This API sets the reset system register 3Fh
 * (5th bit)
 *
 *
 *
 *
 *  \param unsigned char rst_sys
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_reset_sys(unsigned char rst_sys)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_SYS__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_RST_SYS, rst_sys);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_RST_SYS__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads self test
 *                          register byte from 3Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *self_test : Pointer holding the
 *                              Self test
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(
unsigned char *self_test)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELF_TEST__REG, &v_data_u8r, 1);
			*self_test =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELF_TEST);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Self Test register 3Fh
 * (0 bit)
 *
 *
 *
 *
 *  \param unsigned char self_test
 *
 *      self_test[0bit]
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(unsigned char self_test)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SELF_TEST__REG, &v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_SELF_TEST, self_test);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SELF_TEST__REG, &v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads temperature source
 *                          register byte from 40h
 *
 *
 *
 *
 *  \param
 *      unsigned char *temp_sour : Pointer holding the
 *                              temperature source
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(
unsigned char *temp_sour)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_SOURCE__REG, &v_data_u8r, 1);
			*temp_sour =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TEMP_SOURCE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the temperature source register 40h
 * (0 & 1st bit)
 *
 *
 *
 *
 *  \param unsigned char temp_sour
 *
 *      temp_sour[0 to 1 bit]
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(unsigned char temp_sour)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_SOURCE__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_SOURCE, temp_sour);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_SOURCE__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
			} else {
			return ERROR1;
			}
		}
		if (prev_opmode != OPERATION_MODE_CONFIG)
			/* set the operation mode
			of previous operation mode*/
			bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads Remapped X axis value
 *                          register byte from 41h
 *
 *
 *
 *
 *  \param
 *      unsigned char *remap_axis : Pointer holding the
 *                              Remapped axis value
 *remap_axis - 0X21
 *REMAP_X_Y -> This case the axis remapped as Z=Z;X=Y;Y=X
 *
 *remap_axis - 0X18
 *REMAP_Y_Z -> This case the axis remapped as X=X;Y=Z;Z=Y
 *
 *remap_axis - 0X06
 *REMAP_Z_X -> This case the axis remapped as Y=Y;X=Z;Z=X
 *
 *remap_axis - 0X12
 *REMAP_X_Y_Z_TYPE0 -> This case the axis remapped as X=Z;Y=X;Z=Y
 *
 *remap_axis - 0X09
 *REMAP_X_Y_Z_TYPE1 -> This case the axis remapped as X=Y;Y=Z;Z=X
 *
 *remap_axis - 0X24
 *DEFAULT_AXIS -> This case is the default axis settings X=X;Y=Y;Z=Z
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(
unsigned char *remap_axis)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_AXIS_VALUE__REG, &v_data_u8r, 1);
			*remap_axis =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_AXIS_VALUE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Remapped X,Y and Z axis value
 * in the 41h
 * (0 to 5th bit)
 *
 *
 *
 *
 *  \param unsigned char remap_axis
 *
 *remap_axis - 0X21
 *REMAP_X_Y -> This case the axis remapped as Z=Z;X=Y;Y=X
 *
 *remap_axis - 0X18
 *REMAP_Y_Z -> This case the axis remapped as X=X;Y=Z;Z=Y
 *
 *remap_axis - 0X06
 *REMAP_Z_X -> This case the axis remapped as Y=Y;X=Z;Z=X
 *
 *remap_axis - 0X12
 *REMAP_X_Y_Z_TYPE0 -> This case the axis remapped as X=Z;Y=X;Z=Y
 *
 *remap_axis - 0X09
 *REMAP_X_Y_Z_TYPE1 -> This case the axis remapped as X=Y;Y=Z;Z=X
 *
 *remap_axis - 0X24
 *DEFAULT_AXIS -> This case is the default axis settings X=X;Y=Y;Z=Z
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(
unsigned char remap_axis)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					switch (remap_axis) {
					case REMAP_X_Y:
					case REMAP_Y_Z:
					case REMAP_Z_X:
					case REMAP_X_Y_Z_TYPE0:
					case REMAP_X_Y_Z_TYPE1:
					case DEFAULT_AXIS:
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_REMAP_AXIS_VALUE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_REMAP_AXIS_VALUE,
						remap_axis);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_REMAP_AXIS_VALUE__REG,
						&v_data_u8r, 1);
					break;
					default:
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_REMAP_AXIS_VALUE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_REMAP_AXIS_VALUE,
						DEFAULT_AXIS);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_REMAP_AXIS_VALUE__REG,
						&v_data_u8r, 1);
					break;
					}
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads Remapped X axis Sign
 *                          register byte from 42h
 *
 *
 *
 *
 *  \param
 *      unsigned char *remap_sign_x : Pointer holding the
 *                              Remapped X axis Sign
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_x_remap_sign(
unsigned char *remap_sign_x)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_X_SIGN__REG, &v_data_u8r, 1);
			*remap_sign_x =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_REMAP_X_SIGN);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Remapped X axis sign 42h
 * (2nd bit)
 *
 *
 *
 *
 *  \param unsigned char remap_sign_x
 *
 *      remap_sign_x[2nd bit]
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_x_remap_sign(
unsigned char remap_sign_x)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_X_SIGN__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_X_SIGN, remap_sign_x);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_X_SIGN__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads Remapped Y axis Sign
 *                          register byte from 41h
 *
 *
 *
 *
 *  \param
 *      unsigned char *remap_sign_y : Pointer holding the
 *                              Remapped Y axis Sign
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_y_remap_sign(
unsigned char *remap_sign_y)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Y_SIGN__REG, &v_data_u8r, 1);
			*remap_sign_y =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_REMAP_Y_SIGN);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Remapped Y axis Sign 42h
 * (1st)
 *
 *
 *
 *
 *  \param unsigned char remap_sign_y
 *
 *      remap_sign_y[1st bit]
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_y_remap_sign(
unsigned char remap_sign_y)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Y_SIGN__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Y_SIGN, remap_sign_y);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Y_SIGN__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
 /*****************************************************************************
 * Description: *//**\brief Reads Remapped Z axis Sign
 *                          register byte from 42h
 *
 *
 *
 *
 *  \param
 *      unsigned char *remap_sign_z : Pointer holding the
 *                              Remapped Z axis Sign
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_z_remap_sign(
unsigned char *remap_sign_z)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Z_SIGN__REG, &v_data_u8r, 1);
			*remap_sign_z =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_REMAP_Z_SIGN);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the Remapped Z axis Sign 42h
 * (0th bit)
 *
 *
 *
 *
 *  \param unsigned char remap_sign_z
 *
 *      remap_sign_z[4 to 5 bit]
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_z_remap_sign(
unsigned char remap_sign_z)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Z_SIGN__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Z_SIGN,
					remap_sign_z);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Z_SIGN__REG,
					&v_data_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 0
 *                          from location 43hh and 44h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_zero   : Pointer holding the
 *                            soft iron calibration matrix 0
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_zero(
BNO055_S16 *sic_matrix_zero)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_0_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_0_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_0_MSB);
			*sic_matrix_zero = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 0
 *                          from location 43hh and 44h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_zero -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_zero(
BNO055_S16 sic_matrix_zero)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_zero & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_0_LSB, v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_zero >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_0_MSB,
					v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
		}
		if (prev_opmode != OPERATION_MODE_CONFIG)
			/* set the operation mode
			of previous operation mode*/
			bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 1
 *                          from location 45hh and 46h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_one   : Pointer holding the
 *                            soft iron calibration matrix 1
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_one(
BNO055_S16 *sic_matrix_one)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_1_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_1_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_1_MSB);
			*sic_matrix_one = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 1
 *                          from location 45hh and 46h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_one -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_one(
BNO055_S16 sic_matrix_one)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_one & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_1_LSB,
					v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_LSB__REG,
					&v_data2_u8r, 1);

					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_one >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_1_MSB,
					v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 2
 *                          from location 47hh and 48h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_two   : Pointer holding the
 *                            soft iron calibration matrix 2
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_two(
BNO055_S16 *sic_matrix_two)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_2_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_2_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_2_MSB);
			*sic_matrix_two = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 2
 *                          from location 47h and 48h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_two -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_two(
BNO055_S16 sic_matrix_two)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_two & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_2_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_two >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_2_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 3
 *                          from location 49h and 4Ah
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_three   : Pointer holding the
 *                            soft iron calibration matrix 3
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_three(
BNO055_S16 *sic_matrix_three)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_3_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_3_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_3_MSB);
			*sic_matrix_three = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 3
 *                          from location 49h and 4Ah
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_three -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_three(
BNO055_S16 sic_matrix_three)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_three & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_3_LSB, v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_three >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_3_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 4
 *                          from location 4Bh and 4Ch
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_four   : Pointer holding the
 *                            soft iron calibration matrix 4
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_four(
BNO055_S16 *sic_matrix_four)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_4_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_4_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_4_MSB);
			*sic_matrix_four = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 4
 *                          from location 4Bh and 4Ch
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_four -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_four(
BNO055_S16 sic_matrix_four)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_four & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_4_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_four >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_4_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 5
 *                          from location 4Dh and 4Eh
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_five   : Pointer holding the
 *                            soft iron calibration matrix 5
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_five(
BNO055_S16 *sic_matrix_five)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_5_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_5_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_5_MSB);
			*sic_matrix_five = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 5
 *                          from location 4Dh and 4Eh
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_five -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_five(
BNO055_S16 sic_matrix_five)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_five & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_5_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_five >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_5_MSB, v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 6
 *                          from location 4Fh and 50h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_six   : Pointer holding the
 *                            soft iron calibration matrix 6
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_six(
BNO055_S16 *sic_matrix_six)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_6_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_6_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_6_MSB);
			*sic_matrix_six = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 6
 *                          from location 4Fh and 50h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_six -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_six(
BNO055_S16 sic_matrix_six)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_six & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_6_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_six >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_6_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 7
 *                          from location 51h and 52h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_seven   : Pointer holding the
 *                            soft iron calibration matrix 7
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_seven(
BNO055_S16 *sic_matrix_seven)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_7_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_7_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_7_MSB);
			*sic_matrix_seven = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 7
 *                          from location 51h and 52h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_seven -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_seven(
BNO055_S16 sic_matrix_seven)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_seven & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_7_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_seven >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_7_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads soft iron calibration matrix 8
 *                          from location 53h and 54h
 *
 *
 *
 *
 *  \param BNO055_S16  *sic_matrix_eight   : Pointer holding the
 *                            soft iron calibration matrix 8
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_eight(
BNO055_S16 *sic_matrix_eight)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_8_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SIC_MATRIX_8_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_SIC_MATRIX_8_MSB);
			*sic_matrix_eight = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write soft iron calibration matrix 8
 *                          from location 53h and 54h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   sic_matrix_eight -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_eight(
BNO055_S16 sic_matrix_eight)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_eight & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_8_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(sic_matrix_eight >>
					BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_SIC_MATRIX_8_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Accel offset of X axis
 *                          from location 55h and 56h
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_off_x   : Pointer holding the
 *                             Accel offset of X axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_x_axis(
BNO055_S16 *acc_off_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_OFFSET_X_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_OFFSET_X_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_OFFSET_X_MSB);
			*acc_off_x = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Accel offset of X axis
 *                          from location 55h and 56h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   acc_off_x -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_x_axis(
BNO055_S16 acc_off_x)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_x & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_X_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_x >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_X_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Accel offset of Y axis
 *                          from location 57h and 58h
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_off_y   : Pointer holding the
 *                             Accel offset of Y axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_y_axis(
BNO055_S16 *acc_off_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_OFFSET_Y_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_OFFSET_Y_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_OFFSET_Y_MSB);
			*acc_off_y = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Accel offset of Y axis
 *                          from location 57h and 58h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   acc_off_y -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_y_axis(
BNO055_S16 acc_off_y)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_y & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_Y_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_y >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_Y_MSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Accel offset of Z axis
 *                          from location 59h and 5ah
 *
 *
 *
 *
 *  \param BNO055_S16  *acc_off_z   : Pointer holding the
 *                             Accel offset of Z axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_z_axis(
BNO055_S16 *acc_off_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_OFFSET_Z_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACC_OFFSET_Z_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACC_OFFSET_Z_MSB);
			*acc_off_z = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Accel offset of Z axis
 *                          from location 59h and 5Ah
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   acc_off_z -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_z_axis(
BNO055_S16 acc_off_z)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_z & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_Z_LSB,
					v_data1_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(acc_off_z >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_ACC_OFFSET_Z_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Mag offset of X axis
 *                          from location 5Bh and 5Ch
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_off_x   : Pointer holding the
 *                             Mag offset of X axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_x_axis(
BNO055_S16 *mag_off_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OFFSET_X_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_OFFSET_X_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_OFFSET_X_MSB);
			*mag_off_x = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Mag offset of X axis
 *                          from location 5Bh and 5Ch
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   mag_off_x -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_x_axis(
BNO055_S16 mag_off_x)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_x & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_X_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_x >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_X_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief This API reads Mag offset of Y axis
 *                          from location 5Dh and 5Eh
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_off_y   : Pointer holding the
 *                             Mag offset of Y axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_y_axis(
BNO055_S16 *mag_off_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OFFSET_Y_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_OFFSET_Y_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_OFFSET_Y_MSB);
			*mag_off_y = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Mag offset of Y axis
 *                          from location 5Dh and 5Eh
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   mag_off_y -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_y_axis(
BNO055_S16 mag_off_y)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_y & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_Y_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_y >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_Y_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Mag offset of Z axis
 *                          from location 5Fh and 60h
 *
 *
 *
 *
 *  \param BNO055_S16  *mag_off_z   : Pointer holding the
 *                             Mag offset of Z axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_z_axis(
BNO055_S16 *mag_off_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OFFSET_Z_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_OFFSET_Z_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_OFFSET_Z_MSB);
			*mag_off_z = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Mag offset of Z axis
 *                          from location 5Fh and 60h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   mag_off_z -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_z_axis(
BNO055_S16 mag_off_z)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_z & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_Z_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(mag_off_z >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_MAG_OFFSET_Z_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Gyro offset of X axis
 *                          from location 61h and 62h
 *
 *
 *
 *
 *  \param BNO055_S16  *gyr_off_x   : Pointer holding the
 *                             Gyro offset of Z axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_x_axis(
BNO055_S16 *gyr_off_x)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_OFFSET_X_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_OFFSET_X_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_OFFSET_X_MSB);
			*gyr_off_x = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Gyro offset of X axis
 *                          from location 61h and 62h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   gyr_off_x -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_x_axis(
BNO055_S16 gyr_off_x)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_x & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_X_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_x >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_X_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Gyro offset of Y axis
 *                          from location 63h and 64h
 *
 *
 *
 *
 *  \param BNO055_S16  *gyr_off_y   : Pointer holding the
 *                             Gyro offset of Y axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_y_axis(
BNO055_S16 *gyr_off_y)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_OFFSET_Y_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_OFFSET_Y_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_OFFSET_Y_MSB);
			*gyr_off_y = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Gyro offset of Y axis
 *                          from location 63h and 64h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   gyr_off_y -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_y_axis(
BNO055_S16 gyr_off_y)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_y & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_Y_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);

					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_y >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_Y_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API reads Gyro offset of Z axis
 *                          from location 65h and 66h
 *
 *
 *
 *
 *  \param BNO055_S16  *gyr_off_z   : Pointer holding the
 *                             Gyro offset of Z axis
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_z_axis(
BNO055_S16 *gyr_off_z)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_OFFSET_Z_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYR_OFFSET_Z_LSB);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYR_OFFSET_Z_MSB);
			*gyr_off_z = (BNO055_S16)((((BNO055_S16)
			(signed char)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef
#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API write Gyro offset of Z axis
 *                          from location 65h and 66h
 *
 *
 *
 *
 *  \param unsigned char
 *
 *                   gyr_off_z -> Any valid value
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_z_axis(
BNO055_S16 gyr_off_z)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data1_u8r = BNO055_Zero_U8X;
unsigned char v_data2_u8r = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
	} else {
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_z & 0x00FF));
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_Z_LSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);

					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					v_data1_u8r = ((signed char)
					(gyr_off_z >> BNO055_SHIFT_8_POSITION)
					& 0x00FF);
					v_data2_u8r =
					BNO055_SET_BITSLICE(v_data2_u8r,
					BNO055_GYR_OFFSET_Z_MSB, v_data1_u8r);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
				} else {
				return ERROR1;
				}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
* PAGE 1
***************************************************************************/
/***************************************************************************
 * Description: *//**\brief Reads Accel range
 *                          register byte from page1 08h
 *
 *
 *  \param
 *      unsigned char *accel_range : Pointer holding the
 *                                Accel range
 *
 *
 *
 *accel_range
 *ACCEL_RANGE_2G - 0x00
 *ACCEL_RANGE_4G - 0x01
 *ACCEL_RANGE_8G - 0x02
 *ACCEL_RANGE_16G- 0x03
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(
unsigned char *accel_range)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_ACC_RANGE__REG, &v_data_u8r, 1);
			*accel_range =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_ACC_RANGE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel range in
 *                                    page1 08h register
 *
 *
 *  \param unsigned char accel_range
 *
 *
 *accel_range
 *ACCEL_RANGE_2G - 0x00
 *ACCEL_RANGE_4G - 0x01
 *ACCEL_RANGE_8G - 0x02
 *ACCEL_RANGE_16G- 0x03
 *
 *
 *
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(
unsigned char accel_range)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if (accel_range < BNO055_Five_U8X) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_ACC_RANGE__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_CONFIG_ACC_RANGE,
					accel_range);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_ACC_RANGE__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	bno055_set_operation_mode
	(prev_opmode);
return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the bandwidth of the accel
 *
 *
 *
 *
 *  \param  unsigned char * accel_bw : Pointer holding the bandwidth
 *                accel_bw ->

 *						0x00 -> 7.81HZ
 *                      0x01 -> 15.63HZ
 *                      0x02 -> 31.25HZ
 *                      0X03 -> 62.50HZ
 *                      0X04 -> 125HZ
 *                      0X05 -> 250HZ
 *                      0X06 -> 500HZ
 *                      0X07 -> 1000HZ
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bandwidth(
unsigned char *accel_bw)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_ACC_BW__REG, &v_data_u8r, 1);
			*accel_bw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_ACC_BW);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set Bandwidth of the accel
 *
 *
 *
 *
 *  \param unsigned char accel_bandwidth: The value of Bandwidth
 *              accel_bandwidth ->
 *						0x00 -> 7.81HZ
 *                      0x01 -> 15.63HZ
 *                      0x02 -> 31.25HZ
 *                      0X03 -> 62.50HZ
 *                      0X04 -> 125HZ
 *                      0X05 -> 250HZ
 *                      0X06 -> 500HZ
 *                      0X07 -> 1000HZ
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bandwidth(
unsigned char accel_bw)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if (accel_bw < BNO055_Eight_U8X) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_ACC_BW__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r, BNO055_CONFIG_ACC_BW,
					accel_bw);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_ACC_BW__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endi
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the power mode of the accel
 *
 *
 *
 *
 *  \param  unsigned char * accel_pw : Pointer holding
 *                  the power mode of the accel
 *                accel_pw ->
 *						0x00 -> Normal
 *                      0x01 -> Suspend
 *                      0x02 -> Lowpower1
 *                      0X03 -> standby
 *                      0X04 -> Lowpower2
 *                      0X05 -> Deep suspend
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_powermode(
unsigned char *accel_pw)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_ACC_PWR_MODE__REG, &v_data_u8r, 1);
			*accel_pw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_ACC_PWR_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set power mode of the accel
 *
 *
 *
 *
 *  \param unsigned char accel_bandwidth: The value of power mode of the accel
 *              accel_pm ->
*						0x00 -> Normal
 *                      0x01 -> Suspend
 *                      0x02 -> Lowpower1
 *                      0X03 -> standby
 *                      0X04 -> Lowpower2
 *                      0X05 -> Deep suspend
 *
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_powermode(
unsigned char accel_pm)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (accel_pm < BNO055_Seven_U8X) {
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_ACC_PWR_MODE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_CONFIG_ACC_PWR_MODE,
						accel_pm);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_ACC_PWR_MODE__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the data output rate of the mag
 *
 *
 *
 *
 *  \param  unsigned char * mag_data_outrate : Pointer holding
 *                      the data output rate of mag
 *                 ->  mag_data_outrate
 *						0x00 -> MAG_DATA_OUTRATE_2Hz
 *                      0x01 -> MAG_DATA_OUTRATE_6Hz
 *                      0x02 -> MAG_DATA_OUTRATE_8Hz
 *                      0X03 -> MAG_DATA_OUTRATE_10Hz
 *                      0X04 -> MAG_DATA_OUTRATE_15Hz
 *                      0X05 -> MAG_DATA_OUTRATE_20Hz
 *                      0X06 -> MAG_DATA_OUTRATE_25Hz
 *                      0X07 -> MAG_DATA_OUTRATE_30Hz
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_outputrate(
unsigned char *mag_data_outrate)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__REG,
			&v_data_u8r, 1);
			*mag_data_outrate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_MAG_DATA_OUTPUT_RATE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the data output rate of
 *                                                mag in the register 09h
 *
 *
 *
 *
 *  \param unsigned char mag_data_outrate: The value of data output rate of mag
 *              mag_data_outrate ->
 *						0x00 -> MAG_DATA_OUTRATE_2Hz
 *                      0x01 -> MAG_DATA_OUTRATE_6Hz
 *                      0x02 -> MAG_DATA_OUTRATE_8Hz
 *                      0X03 -> MAG_DATA_OUTRATE_10Hz
 *                      0X04 -> MAG_DATA_OUTRATE_15Hz
 *                      0X05 -> MAG_DATA_OUTRATE_20Hz
 *                      0X06 -> MAG_DATA_OUTRATE_25Hz
 *                      0X07 -> MAG_DATA_OUTRATE_30Hz
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_outrate(
unsigned char mag_data_outrate)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if (mag_data_outrate
					< BNO055_Eight_U8X) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_CONFIG_MAG_DATA_OUTPUT_RATE,
					mag_data_outrate);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the operation mode of the mag
 *
 *
 *
 *
 *  \param  unsigned char * mag_operation_mode:
 *                     Pointer holding the operation mode of Mag
 *                mag_operation_mode ->
 *						0x00 -> MAG_OPR_MODE_LOWPOWER
 *                      0x01 -> MAG_OPR_MODE_REGULAR
 *                      0x02 -> MAG_OPR_MODE_ENHANCED_REGULAR
 *                      0X03 -> MAG_OPR_MODE_HIGH_ACCURACY
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(
unsigned char *mag_operation_mode)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_MAG_OPR_MODE__REG, &v_data_u8r, 1);
			*mag_operation_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_MAG_OPR_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set operation mode of Mag
 *
 *
 *
 *
 *  \param unsigned char accel_bandwidth: The value of operation mode of Mag
 *              mag_operation_mode ->
*						0x00 -> MAG_OPR_MODE_LOWPOWER
 *                      0x01 -> MAG_OPR_MODE_REGULAR
 *                      0x02 -> MAG_OPR_MODE_ENHANCED_REGULAR
 *                      0X03 -> MAG_OPR_MODE_HIGH_ACCURACY
 *
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(
unsigned char mag_operation_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (mag_operation_mode
						< BNO055_Five_U8X) {
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_MAG_OPR_MODE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_CONFIG_MAG_OPR_MODE,
						mag_operation_mode);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_MAG_OPR_MODE__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the power mode of the mag
 *
 *
 *
 *
 *  \param  unsigned char * mag_pw : Pointer holding
 *                  the power mode of the mag
 *                mag_pw ->
 *						0x00 -> MAG_POWER_MODE_NORMAL
 *                      0x01 -> MAG_POWER_MODE_SLEEP
 *                      0x02 -> MAG_POWER_MODE_SUSPEND
 *                      0X03 -> MAG_POWER_MODE_FORCE_MODE
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_powermode(
unsigned char *mag_pw)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_MAG_POWER_MODE__REG, &v_data_u8r, 1);
			*mag_pw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_MAG_POWER_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set power mode of the mag
 *
 *
 *
 *
 *  \param unsigned char mag_pw: The value of power mode of the mag
 *              mag_pw ->
*						0x00 -> MAG_POWER_MODE_NORMAL
 *                      0x01 -> MAG_POWER_MODE_SLEEP
 *                      0x02 -> MAG_POWER_MODE_SUSPEND
 *                      0X03 -> MAG_POWER_MODE_FORCE_MODE
 *
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_powermode(
unsigned char mag_pw)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if (mag_pw < BNO055_Five_U8X) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_MAG_POWER_MODE__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_CONFIG_MAG_POWER_MODE, mag_pw);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_MAG_POWER_MODE__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the range of gyro
 *
 *
 *
 *
 *  \param  unsigned char * gyro_range : Pointer holding
 *                      the range of gyro
 *                 ->  gyro_range
 *						0x00 -> GYRO_RANGE_2000rps
 *                      0x01 -> GYRO_RANGE_1000rps
 *                      0x02 -> GYRO_RANGE_500rps
 *                      0X03 -> GYRO_RANGE_250rps
 *                      0X04 -> GYRO_RANGE_125rps
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(
unsigned char *gyro_range)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_GYR_RANGE__REG, &v_data_u8r, 1);
			*gyro_range =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_GYR_RANGE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the range of gyro
 *                                                 in the register 1Ah
 *
 *
 *
 *
 *  \param unsigned char gyro_range: The value of the range of gyro
 *              gyro_range ->
 *						0x00 -> GYRO_RANGE_2000rps
 *                      0x01 -> GYRO_RANGE_1000rps
 *                      0x02 -> GYRO_RANGE_500rps
 *                      0X03 -> GYRO_RANGE_250rps
 *                      0X04 -> GYRO_RANGE_125rps
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(
unsigned char gyro_range)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (gyro_range < BNO055_Five_U8X) {
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_GYR_RANGE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_CONFIG_GYR_RANGE,
						gyro_range);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_CONFIG_GYR_RANGE__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set
 *             the bandwidth of gyro
 *
 *
 *
 *
 *  \param  unsigned char * gyro_bw : Pointer holding
 *                      the bandwidth of gyro
 *                 ->  gyro_bw
 *						0x00 -> GYRO_BW_523Hz
 *                      0x01 -> GYRO_BW_230Hz
 *                      0x02 -> GYRO_BW_116Hz
 *                      0X03 -> GYRO_BW_47Hz
 *                      0X04 -> GYRO_BW_23Hz
 *                      0X05 -> GYRO_BW_12Hz
 *                      0X06 -> GYRO_BW_64Hz
 *                      0X07 -> GYRO_BW_32Hz
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bandwidth(
unsigned char *gyro_bw)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_GYR_BANDWIDTH__REG, &v_data_u8r, 1);
			*gyro_bw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_GYR_BANDWIDTH);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the range of gyro
 *                                                 in the register 1Ah
 *
 *
 *
 *
 *  \param unsigned char gyro_bw: The value of data output rate of mag
 *              gyro_bw ->
 *						0x00 -> GYRO_BW_523Hz
 *                      0x01 -> GYRO_BW_230Hz
 *                      0x02 -> GYRO_BW_116Hz
 *                      0X03 -> GYRO_BW_47Hz
 *                      0X04 -> GYRO_BW_23Hz
 *                      0X05 -> GYRO_BW_12Hz
 *                      0X06 -> GYRO_BW_64Hz
 *                      0X07 -> GYRO_BW_32Hz
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bandwidth(
unsigned char gyro_bw)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	unsigned char gyro_opmode = BNO055_Zero_U8X;
	unsigned char gyro_autosleepduration = BNO055_Zero_U8X;
	unsigned char pg_status = BNO055_Zero_U8X;
	unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if ((gyro_bw == BNO055_Zero_U8X ||
					gyro_bw > BNO055_Zero_U8X) &&
					gyro_bw < BNO055_Eight_U8X) {
					bno055_get_gyro_operation_mode
					(&gyro_opmode);
					if (gyro_opmode ==
					GYRO_OPR_MODE_ADVANCE_POWERSAVE) {
						bno055_get_gyro_autosleepdur
						(&gyro_autosleepduration);
						bno055_gyro_set_autosleepdur
						(gyro_autosleepduration,
						gyro_bw);
					}
					switch (gyro_bw) {
					case GYRO_BW_523Hz:
					gyro_bw = GYRO_BW_523Hz;
					break;
					case GYRO_BW_230Hz:
					gyro_bw = GYRO_BW_230Hz;
					break;
					case GYRO_BW_116Hz:
					gyro_bw = GYRO_BW_116Hz;
					break;
					case GYRO_BW_47Hz:
					gyro_bw = GYRO_BW_47Hz;
					break;
					case GYRO_BW_23Hz:
					gyro_bw = GYRO_BW_23Hz;
					break;
					case GYRO_BW_12Hz:
					gyro_bw = GYRO_BW_12Hz;
					break;
					case GYRO_BW_64Hz:
					gyro_bw = GYRO_BW_64Hz;
					break;
					case GYRO_BW_32Hz:
					gyro_bw = GYRO_BW_32Hz;
					break;
					default:
					break;
					}
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_GYR_BANDWIDTH__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_CONFIG_GYR_BANDWIDTH,
					gyro_bw);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_GYR_BANDWIDTH__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *             the operation mode of the gyro
 *
 *
 *
 *
 *  \param  unsigned char * gyro_operation_mode:
 *                     Pointer holding the operation mode of gyro
 *gyro_operation_mode ->
 *			   0x00 -> GYRO_OPR_MODE_NORMAL
 *             0x01 -> GYRO_OPR_MODE_FASTPOWERUP
 *             0x02 -> GYRO_OPR_MODE_DEEPSUSPEND
 *             0X03 -> GYRO_OPR_MODE_SUSPEND
 *			   0X04 -> GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_operation_mode(
unsigned char *gyro_operation_mode)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CONFIG_GYR_POWER_MODE__REG,
			&v_data_u8r, 1);
			*gyro_operation_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_CONFIG_GYR_POWER_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set operation mode of Gyro
 *
 *
 *param unsigned char gyro_operation_mode:
 *                    The value of operation mode of Gyro
 *
 *gyro_operation_mode ->
 *    0x00 -> GYRO_OPR_MODE_NORMAL
 *    0x01 -> GYRO_OPR_MODE_FASTPOWERUP
 *    0x02 -> GYRO_OPR_MODE_DEEPSUSPEND
 *    0X03 -> GYRO_OPR_MODE_SUSPEND
 *    0X04 -> GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_operation_mode(
unsigned char gyro_operation_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char gyro_autosleepduration = BNO055_Zero_U8X;
unsigned char gyro_bw = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				if ((gyro_operation_mode == BNO055_Zero_U8X ||
				gyro_operation_mode > BNO055_Zero_U8X) &&
				gyro_operation_mode < BNO055_Five_U8X) {
					switch (gyro_operation_mode) {
					case GYRO_OPR_MODE_NORMAL:
					gyro_operation_mode =
					GYRO_OPR_MODE_NORMAL;
					break;
					case GYRO_OPR_MODE_FASTPOWERUP:
					gyro_operation_mode =
					GYRO_OPR_MODE_FASTPOWERUP;
					break;
					case GYRO_OPR_MODE_DEEPSUSPEND:
					gyro_operation_mode =
					GYRO_OPR_MODE_DEEPSUSPEND;
					break;
					case GYRO_OPR_MODE_SUSPEND:
					gyro_operation_mode =
					GYRO_OPR_MODE_SUSPEND;
					break;
					case GYRO_OPR_MODE_ADVANCE_POWERSAVE:
					bno055_get_gyro_bandwidth
					(&gyro_bw);
					bno055_get_gyro_autosleepdur
					(&gyro_autosleepduration);
					bno055_gyro_set_autosleepdur
					(gyro_autosleepduration,
					gyro_bw);
					status =
					bno055_write_page_id(PAGE_ONE);
					gyro_operation_mode =
					GYRO_OPR_MODE_ADVANCE_POWERSAVE;
					break;
					default:
					break;
					}
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_GYR_POWER_MODE__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_CONFIG_GYR_POWER_MODE,
					gyro_operation_mode);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_CONFIG_GYR_POWER_MODE__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to get
 *                 the sleep timer mode status of Accel
 *
 *
 *
 *
 *  \param  unsigned char *sleep_tmr : Pointer holding the sleep_tmr
 *                  sleep_tmr -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling mode(EST)
 *
 *
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleeptmr_mode(
unsigned char *sleep_tmr)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/*SLEEP TIMER MODE */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_SLEEP_MODE__REG, &v_data_u8r, 1);
			*sleep_tmr =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACC_SLEEP_MODE);
		} else {
		return ERROR1;
		}
	}
		return comres;
	}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set
 *                the sleep timer mode status of Accel
 *
 *
 *
 *
 *  \param unsigned char sleep_tmr:	The value of sleep timer mode status
 *                  sleep_tmr -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling mode(EST)
 *
 *
 *
 *  \return communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleeptmr_mode(
unsigned char sleep_tmr)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (sleep_tmr < BNO055_Two_U8X) {
						/*SLEEP TIMER MODE*/
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACC_SLEEP_MODE__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACC_SLEEP_MODE,
						sleep_tmr);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACC_SLEEP_MODE__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/****************************************************************************
 * Description: *//**\brief This API is used to get
 *                    the sleep duration status of the Accel
 *
 *
 *
 *
 *  \param  unsigned char *sleep_dur : Pointer holding the sleep_dur time
 *                       0x05 -> BNO055_ACCEL_SLEEP_DUR_0_5MS
 *                       0x06 -> BNO055_ACCEL_SLEEP_DUR_1MS
 *                       0x07 -> BNO055_ACCEL_SLEEP_DUR_2MS
 *                       0x08 -> BNO055_ACCEL_SLEEP_DUR_4MS
 *                       0x09 -> BNO055_ACCEL_SLEEP_DUR_6MS
 *                       0x0A -> BNO055_ACCEL_SLEEP_DUR_10MS
 *                       0x0B -> BNO055_ACCEL_SLEEP_DUR_25MS
 *                       0x0C -> BNO055_ACCEL_SLEEP_DUR_50MS
 *                       0x0D -> BNO055_ACCEL_SLEEP_DUR_100MS
 *                       0x0E -> BNO055_ACCEL_SLEEP_DUR_500MS
 *                       0x0F -> BNO055_ACCEL_SLEEP_DUR_1S
 *
 *
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_dur(
unsigned char *sleep_dur)
	{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/*SLEEP TIMER MODE */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACC_SLEEP_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
	}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set Sleep Duration of
 *															Accel
 *
 *
 *
 *  \param unsigned char sleep_dur: The value of Sleep Duration time
 *        sleep_dur ->   0x05 -> BNO055_ACCEL_SLEEP_DUR_0_5MS
 *                       0x06 -> BNO055_ACCEL_SLEEP_DUR_1MS
 *                       0x07 -> BNO055_ACCEL_SLEEP_DUR_2MS
 *                       0x08 -> BNO055_ACCEL_SLEEP_DUR_4MS
 *                       0x09 -> BNO055_ACCEL_SLEEP_DUR_6MS
 *                       0x0A -> BNO055_ACCEL_SLEEP_DUR_10MS
 *                       0x0B -> BNO055_ACCEL_SLEEP_DUR_25MS
 *                       0x0C -> BNO055_ACCEL_SLEEP_DUR_50MS
 *                       0x0D -> BNO055_ACCEL_SLEEP_DUR_100MS
 *                       0x0E -> BNO055_ACCEL_SLEEP_DUR_500MS
 *                       0x0F -> BNO055_ACCEL_SLEEP_DUR_1S
 *
 *
 *  \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_dur(
unsigned char sleep_dur)
	{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (sleep_dur < BNO055_Sixteen_U8X) {
						/*SLEEP DURATION*/
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACC_SLEEP_DUR__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACC_SLEEP_DUR,
						sleep_dur);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACC_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief  This API is used to get data sleep duration of
 *													gyro
 *
 *
 *
 *  \param unsigned char *sleep_dur : Pointer holding the sleep duration
 *
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleepdur(unsigned char *sleep_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/*SLEEP TIMER MODE */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_SLEEP_DUR);
		} else {
		return ERROR1;
		}
	}
		return comres;
	}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set duration of gyro
 *
 *
 *
 *
 *  \param unsigned char sleep_dur:
 *          Value to be written passed as a parameter
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleepdur(unsigned char sleep_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
		status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					if (sleep_dur < BNO055_Eight_U8X) {
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_SLEEP_DUR__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYR_SLEEP_DUR,
						sleep_dur);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/*****************************************************************************
 * Description: *//**\brief  This API is used to get data auto sleep duration
 *									of gyro
 *
 *
 *
 *  \param unsigned char *auto_duration : Pointer holding
 *				the auto sleep duration
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_autosleepdur(
unsigned char *auto_duration)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AUTO_SLEEP_DUR__REG, &v_data_u8r, 1);
			*auto_duration =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_AUTO_SLEEP_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set auto sleep duration of
 *															gyro
 *
 *
 *
 *  \param unsigned char auto_duration:
 *          Value to be written passed as a parameter
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_autosleepdur(
unsigned char auto_duration, unsigned char bandwith)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char v_autosleepduration_u8r;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			pg_status = bno055_write_page_id(PAGE_ONE);
			if (pg_status == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_AUTO_SLEEP_DUR__REG,
				&v_data_u8r, 1);
				if (auto_duration < BNO055_Eight_U8X) {
					switch (bandwith) {
					case GYRO_BW_523Hz:
					if (auto_duration >
						BNO055_GYRO_4ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_4ms_AuSlpDur;
					break;
					case GYRO_BW_230Hz:
					if (auto_duration >
						BNO055_GYRO_4ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_4ms_AuSlpDur;
					break;
					case GYRO_BW_116Hz:
					if (auto_duration >
						BNO055_GYRO_4ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_4ms_AuSlpDur;
					break;
					case GYRO_BW_47Hz:
					if (auto_duration >
						BNO055_GYRO_5ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_5ms_AuSlpDur;
					break;
					case GYRO_BW_23Hz:
					if (auto_duration >
						BNO055_GYRO_10ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_10ms_AuSlpDur;
					break;
					case GYRO_BW_12Hz:
					if (auto_duration >
						BNO055_GYRO_20ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_20ms_AuSlpDur;
					break;
					case GYRO_BW_64Hz:
					if (auto_duration >
						BNO055_GYRO_10ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_10ms_AuSlpDur;
					break;
					case GYRO_BW_32Hz:
					if (auto_duration >
						BNO055_GYRO_20ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_20ms_AuSlpDur;
					break;
					default:
					if (auto_duration >
						BNO055_GYRO_4ms_AuSlpDur)
						v_autosleepduration_u8r =
						auto_duration;
					else
						v_autosleepduration_u8r =
						BNO055_GYRO_4ms_AuSlpDur;
					break;
					}
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYR_AUTO_SLEEP_DUR,
					v_autosleepduration_u8r);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AUTO_SLEEP_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR1;
			}
		} else {
		return ERROR1;
		}
	} else {
	return ERROR1;
	}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads magnetometer sleep mode
 *
 *
 *
 *
 *
 *  \param
 *      unsigned char *sleep_mode : Pointer holding the
 *                                magnetometer sleep mode
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(
unsigned char *sleep_mode)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_MODE__REG, &v_data_u8r, 1);
			*sleep_mode =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_MAG_SLEEP_MODE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the magnetometer sleep mode
 *
 *
 *
 *
 *
 *
 *  \param unsigned char sleep_mode
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(
unsigned char sleep_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_MODE__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_MAG_SLEEP_MODE,
					sleep_mode);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_MODE__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads magnetometer sleep duration
 *
 *
 *
 *
 *
 *  \param
 *      unsigned char *sleep_dur : Pointer holding the
 *                                magnetometer sleep mode
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_duration(
unsigned char *sleep_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_SLEEP_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the magnetometer sleep duration
 *
 *
 *
 *
 *
 *
 *  \param unsigned char sleep_dur
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_duration(
unsigned char sleep_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_DUR__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_MAG_SLEEP_DUR, sleep_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion interrupt mask
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am : Pointer holding the
 *                                gyro any motion interrupt mask
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_gyro_anymotion(
unsigned char *gyro_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INTMSK__REG, &v_data_u8r, 1);
			*gyro_am =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_AM_INTMSK);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion interrupt mask
 *
 *
 *
 *
 *  \param unsigned char gyro_am
 *
 *      gyro_am interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_gyro_anymotion(
unsigned char gyro_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INTMSK__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_GYR_AM_INTMSK, gyro_am);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INTMSK__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate interrupt mask
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr : Pointer holding the
 *                                high rate interrupt mask
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_gyro_highrate(
unsigned char *gyro_hr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INTMSK__REG,
			&v_data_u8r, 1);
			*gyro_hr =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_HIGH_RATE_INTMSK);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate interrupt mask
 *
 *
 *
 *
 *  \param unsigned char gyro_hr
 *
 *      gyro_hr interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_gyro_highrate(
unsigned char gyro_hr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INTMSK__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_GYR_HIGH_RATE_INTMSK, gyro_hr);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INTMSK__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel high g interrupt mask
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_hg : Pointer holding the
 *                                accel high g
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_high_g(
unsigned char *accel_hg)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INTMSK__REG, &v_data_u8r, 1);
			*accel_hg =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_HIGH_G_INTMSK);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel high g interrupt mask
 *
 *
 *
 *
 *  \param unsigned char accel_hg
 *
 *      accel_hg interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_high_g(
unsigned char accel_hg)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INTMSK__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_ACC_HIGH_G_INTMSK, accel_hg);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INTMSK__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel any motion interrupt mask
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_am : Pointer holding the
 *                                accel any motion
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_anymotion(
unsigned char *accel_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INTMSK__REG, &v_data_u8r, 1);
			*accel_am =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_INTMSK);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel any motion interrupt mask
 *
 *
 *
 *
 *  \param unsigned char accel_am
 *
 *      accel_am interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_anymotion(
unsigned char accel_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INTMSK__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_INTMSK, accel_am);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INTMSK__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel no motion interrupt mask
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_nm : Pointer holding the
 *                                accel no motion
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_nomotion(
unsigned char *accel_nm)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NM_INTMSK__REG, &v_data_u8r, 1);
			*accel_nm =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_NM_INTMSK);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel no motion interrupt mask
 *
 *
 *
 *
 *  \param unsigned char accel_nm
 *
 *      accel_nm interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_nomotion(
unsigned char accel_nm)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NM_INTMSK__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_ACC_NM_INTMSK, accel_nm);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NM_INTMSK__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion interrupt
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am : Pointer holding the
 *                                gyro any motion interrupt
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_anymotion(
unsigned char *gyro_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INT__REG, &v_data_u8r, 1);
			*gyro_am =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_AM_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion interrupt
 *
 *
 *
 *
 *  \param unsigned char gyro_am
 *
 *      gyro_am interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_anymotion(
unsigned char gyro_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INT__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_GYR_AM_INT, gyro_am);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_INT__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate interrupt
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr : Pointer holding the
 *                                high rate interrupt
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_highrate(
unsigned char *gyro_hr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INT__REG, &v_data_u8r, 1);
			*gyro_hr =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_HIGH_RATE_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate interrupt
 *
 *
 *
 *
 *  \param unsigned char gyro_hr
 *
 *      gyro_hr interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_highrate(
unsigned char gyro_hr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INT__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_GYR_HIGH_RATE_INT, gyro_hr);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HIGH_RATE_INT__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;

		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel high g interrupt
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_hg : Pointer holding the
 *                                accel high g
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_high_g(
unsigned char *accel_hg)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INT__REG, &v_data_u8r, 1);
			*accel_hg =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACC_HIGH_G_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel high g interrupt
 *
 *
 *
 *
 *  \param unsigned char accel_hg
 *
 *      accel_hg interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_high_g(
unsigned char accel_hg)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INT__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_ACC_HIGH_G_INT, accel_hg);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_INT__REG,
			&v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel any motion interrupt
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_am : Pointer holding the
 *                                accel any motion
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_anymotion(
unsigned char *accel_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INT__REG, &v_data_u8r, 1);
			*accel_am =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel any motion interrupt
 *
 *
 *
 *
 *  \param unsigned char accel_am
 *
 *      accel_am interrupt msk
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_anymotion(
unsigned char accel_am)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INT__REG, &v_data_u8r, 1);
			v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_INT, accel_am);
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_INT__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel no motion interrupt
 *                          register byte from 0Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_nm : Pointer holding the
 *                                accel no motion
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_nomotion(
unsigned char *accel_nm)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NM_INT__REG, &v_data_u8r, 1);
			*accel_nm =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_NM_INT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel no motion interrupt
 *
 *
 *
 *
 *  \param unsigned char accel_nm
 *
 *      accel_nm interrupt
 *      0 Disable
 *		1 Enable
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_nomotion(
unsigned char accel_nm)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
	if (status == SUCCESS) {
		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_ACC_NM_INT__REG, &v_data_u8r, 1);
		v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
		BNO055_ACC_NM_INT, accel_nm);
		comres = p_bno055->BNO055_BUS_WRITE_FUNC
		(p_bno055->dev_addr,
		BNO055_ACC_NM_INT__REG, &v_data_u8r, 1);
		} else {
		return ERROR1;
		}

	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel any motion threshold
 *                          register byte from 11h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_am_thres : Pointer holding the
 *                               accel any motion threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_threshold(
unsigned char *accel_am_thres)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_THRES__REG, &v_data_u8r, 1);
			*accel_am_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_THRES);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel any motion threshold
 *                       register 11h
 *
 *
 *
 *  \param unsigned char accel_am_thres
 *
 *      accel_am_thres interrupt
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_threshold(
unsigned char accel_am_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AM_THRES__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_AM_THRES,
					accel_am_thres);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AM_THRES__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel any motion duration
 *                          register byte from 12h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_am_dur : Pointer holding the
 *                               accel any motion duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_duration(
unsigned char *accel_am_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_AM_DUR_SET__REG, &v_data_u8r, 1);
			*accel_am_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_AM_DUR_SET);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel any motion duration
 *
 *
 *
 *
 *  \param unsigned char accel_am_dur
 *
 *      accel_am_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_duration(
unsigned char accel_am_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AM_DUR_SET__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_AM_DUR_SET,
					accel_am_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AM_DUR_SET__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the status of Any Enable
 * Channel X,Y,Z
 *
 *
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_ACCEL_AM_NM_X_AXIS -> 0
 *                       BNO055_ACCEL_AM_NM_Y_AXIS -> 1
 *                       BNO055_ACCEL_AM_NM_Z_AXIS -> 2
 *      unsigned char *data: Pointer holding the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_an_nm_axis_enable(
unsigned char channel, unsigned char *data)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_ACCEL_AM_NM_X_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_AN_MOTION_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_AN_MOTION_X_AXIS);
				break;
			case BNO055_ACCEL_AM_NM_Y_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_AN_MOTION_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_AN_MOTION_Y_AXIS);
				break;
			case BNO055_ACCEL_AM_NM_Z_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_AN_MOTION_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_AN_MOTION_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set the status of Any Enable
 * Channel X,Y,Z
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_ACCEL_AM_NM_X_AXIS -> 0
 *                       BNO055_ACCEL_AM_NM_Y_AXIS -> 1
 *                       BNO055_ACCEL_AM_NM_Z_AXIS -> 2
 *      unsigned char *data: the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_an_nm_axis_enable(
unsigned char channel, unsigned char data)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					switch (channel) {
					case BNO055_ACCEL_AM_NM_X_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_X_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACC_AN_MOTION_X_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_X_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_ACCEL_AM_NM_Y_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_Y_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACC_AN_MOTION_Y_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_Y_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_ACCEL_AM_NM_Z_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_Z_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACC_AN_MOTION_Z_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_AN_MOTION_Z_AXIS__REG,
					&v_data_u8r, 1);
					break;
					default:
					comres = E_BNO055_OUT_OF_RANGE;
					break;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/***************************************************************************
 * Description:*//**\brief This API is used to get the status of High g Enable
 * Channel X,Y,Z
 *
 *
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_ACCEL_HIGH_G_X_AXIS -> 0
 *                       BNO055_ACCEL_HIGH_G_Y_AXIS -> 1
 *                       BNO055_ACCEL_HIGH_G_Z_AXIS -> 2
 *      unsigned char *data: Pointer holding the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(
unsigned char channel, unsigned char *data)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_ACCEL_HIGH_G_X_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_HIGH_G_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_HIGH_G_X_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Y_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_HIGH_G_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_HIGH_G_Y_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Z_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACC_HIGH_G_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACC_HIGH_G_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set the status of Any Enable
 * Channel X,Y,Z
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_ACCEL_HIGH_G_X_AXIS -> 0
 *                       BNO055_ACCEL_HIGH_G_Y_AXIS -> 1
 *                       BNO055_ACCEL_HIGH_G_Z_AXIS -> 2
 *      unsigned char *data: the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(
unsigned char channel, unsigned char data)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					switch (channel) {
					case BNO055_ACCEL_HIGH_G_X_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_X_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_HIGH_G_X_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_X_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_ACCEL_HIGH_G_Y_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_Y_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_HIGH_G_Y_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_Y_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_ACCEL_HIGH_G_Z_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_Z_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_HIGH_G_Z_AXIS, data);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_Z_AXIS__REG,
					&v_data_u8r, 1);
					break;
					default:
					comres = E_BNO055_OUT_OF_RANGE;
					break;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel high g duration
 *                          register byte from 13h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_hg_dur : Pointer holding the
 *                               accel high g duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_duration(
unsigned char *accel_hg_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_DURATION__REG, &v_data_u8r, 1);
			*accel_hg_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_HIGH_G_DURATION);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel high g duration
 *
 *
 *
 *
 *  \param unsigned char accel_hg_dur
 *
 *      accel_hg_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_duration(
unsigned char accel_hg_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_DURATION__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_HIGH_G_DURATION,
					accel_hg_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_DURATION__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel high g threshold
 *                          register byte from 14h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_hg_thr : Pointer holding the
 *                               accel high g threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_threshold(
unsigned char *accel_hg_thr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_HIGH_G_THRESHOLD__REG,
			&v_data_u8r, 1);
			*accel_hg_thr =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_HIGH_G_THRESHOLD);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel high g threshold
 *                                  register byte from 14h
 *
 *
 *
 *  \param unsigned char accel_hg_thr
 *
 *      accel_hg_thr
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_threshold(
unsigned char accel_hg_thr)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_THRESHOLD__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_HIGH_G_THRESHOLD,
					accel_hg_thr);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_HIGH_G_THRESHOLD__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/

/*****************************************************************************
 * Description: *//**\brief Reads accel slow no motion threshold
 *                          register byte from 15h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_slow_no_thr : Pointer holding the
 *                               accel slow no motion threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_threshold(
unsigned char *accel_slow_no_thr)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NS_THRESHOLD__REG,
			&v_data_u8r, 1);
			*accel_slow_no_thr =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_NS_THRESHOLD);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel slow no motion threshold
 *
 *
 *
 *
 *  \param unsigned char accel_slow_no_thr
 *
 *      accel_slow_no_thr
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_threshold(
unsigned char accel_slow_no_thr)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NS_THRESHOLD__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_NS_THRESHOLD,
					accel_slow_no_thr);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NS_THRESHOLD__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel no slow motion enable
 *                          register byte from 16h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_slow_no_en : Pointer holding the
 *                                accel no slow motion enable
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(
unsigned char *accel_slow_no_en)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NM_SM_ENABLE__REG,
			&v_data_u8r, 1);
			*accel_slow_no_en =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_NM_SM_ENABLE);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel no slow motion enable
 *
 *
 *
 *
 *  \param unsigned char accel_slow_no_en
 *
 *      accel_slow_no_en
 *      0 slow motion
 *		1 no motion
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(
unsigned char accel_slow_no_en)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NM_SM_ENABLE__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_NM_SM_ENABLE,
					accel_slow_no_en);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NM_SM_ENABLE__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads accel slow no motion duration
 *                          register byte from 16h
 *
 *
 *
 *
 *  \param
 *      unsigned char *accel_slow_no_dur : Pointer holding the
 *                               accel slow no motion duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_duration(
unsigned char *accel_slow_no_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACC_NS_DURATION__REG, &v_data_u8r, 1);
			*accel_slow_no_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACC_NS_DURATION);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the accel slow no motion threshold
 *
 *
 *
 *
 *  \param unsigned char accel_slow_no_dur
 *
 *      accel_slow_no_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_duration(
unsigned char accel_slow_no_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NS_DURATION__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACC_NS_DURATION,
					accel_slow_no_dur);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACC_NS_DURATION__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the status of gyro Any
 * motion Channel X,Y,Z
 *
 *
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_GYRO_AM_X_AXIS -> 0
 *                       BNO055_GYRO_AM_Y_AXIS -> 1
 *                       BNO055_GYRO_AM_Z_AXIS -> 2
 *      unsigned char *data: Pointer holding the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_axis_enable(
unsigned char channel, unsigned char *data)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_GYRO_AM_X_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_AM_X_AXIS__REG, &v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_AM_X_AXIS);
				break;
			case BNO055_GYRO_AM_Y_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_AM_Y_AXIS__REG, &v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_AM_Y_AXIS);
				break;
			case BNO055_GYRO_AM_Z_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_AM_Z_AXIS__REG, &v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_AM_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set the status of Any Enable
 * Channel X,Y,Z
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of Any Enable channel number
 *                       channel :
 *                       BNO055_GYRO_AM_X_AXIS -> 0
 *                       BNO055_GYRO_AM_Y_AXIS -> 1
 *                       BNO055_GYRO_AM_Z_AXIS -> 2
 *      unsigned char *data: the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_axis_enable(
unsigned char channel, unsigned char  data)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					switch (channel) {
					case BNO055_GYRO_AM_X_AXIS:
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_X_AXIS__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYR_AM_X_AXIS,
						data);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_X_AXIS__REG,
						&v_data_u8r, 1);
						break;
					case BNO055_GYRO_AM_Y_AXIS:
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_Y_AXIS__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYR_AM_Y_AXIS, data);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_Y_AXIS__REG,
						&v_data_u8r, 1);
						break;
					case BNO055_GYRO_AM_Z_AXIS:
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_Z_AXIS__REG,
						&v_data_u8r, 1);
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYR_AM_Z_AXIS,
						data);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYR_AM_Z_AXIS__REG,
						&v_data_u8r, 1);
						break;
					default:
						comres = E_BNO055_OUT_OF_RANGE;
						break;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get the status of gyro high
 * rate Channel X,Y,Z
 *
 *
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of high rate Enable channel number
 *                       channel :
 *                       BNO055_GYRO_HR_X_AXIS -> 0
 *                       BNO055_GYRO_HR_Y_AXIS -> 1
 *                       BNO055_GYRO_HR_Z_AXIS -> 2
 *      unsigned char *data: Pointer holding the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *  \return
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(
unsigned char channel, unsigned char *data)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_GYRO_HR_X_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_HR_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_HR_X_AXIS);
				break;
			case BNO055_GYRO_HR_Y_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_HR_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_HR_Y_AXIS);
				break;
			case BNO055_GYRO_HR_Z_AXIS:
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_HR_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_HR_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API is used to set the status of High rate
 * Channel X,Y,Z
 *
 *
 ** *  \param Pointer holding the unsigned char channel :
 *         The value of high rate channel number
 *                       channel :
 *                       BNO055_GYRO_HR_X_AXIS -> 0
 *                       BNO055_GYRO_HR_Y_AXIS -> 1
 *                       BNO055_GYRO_HR_Z_AXIS -> 2
 *      unsigned char *data: the Any Enable value
 *                       data :
 *                       Enable  -> 1
 *                       disable -> 0
 *
 *
 *
 *
 *  \return communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(
unsigned char channel, unsigned char data)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					switch (channel) {
					case BNO055_GYRO_HR_X_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYR_HR_X_AXIS, data);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_GYRO_HR_Y_AXIS:
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE(
					v_data_u8r, BNO055_GYR_HR_Y_AXIS, data);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_AXIS__REG,
					&v_data_u8r, 1);
					break;
					case BNO055_GYRO_HR_Z_AXIS:
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_AXIS__REG,
					&v_data_u8r, 1);
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYR_HR_Z_AXIS, data);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_AXIS__REG,
					&v_data_u8r, 1);
					break;
					default:
					comres = E_BNO055_OUT_OF_RANGE;
					break;
					}
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion filter
 *                          register byte from 17h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am_filter : Pointer holding the
 *                               gyro any motion filter
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_filter(
unsigned char *gyro_am_filter)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_FILT__REG, &v_data_u8r, 1);
			*gyro_am_filter =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_AM_FILT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion filter
 *
 *
 *
 *
 *  \param unsigned char gyro_am_filter
 *
 *      gyro_am_filter
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_filter(
unsigned char gyro_am_filter)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AM_FILT__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_AM_FILT, gyro_am_filter);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AM_FILT__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate filter
 *                          register byte from 17h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_filter : Pointer holding the
 *                               gyro high rate filter
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(
unsigned char *gyro_hr_filter)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_FILT__REG, &v_data_u8r, 1);
			*gyro_hr_filter =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_FILT);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate filter
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_filter
 *
 *      gyro_hr_filter
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(
unsigned char gyro_hr_filter)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_FILT__REG,
					&v_data_u8r,
					1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_FILT, gyro_hr_filter);
					comres =
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_FILT__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate X threshold
 *                          register byte from 18h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_x_thres : Pointer holding the
 *                               gyro high rate X threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_threshold(
unsigned char *gyro_hr_x_thres)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_X_THRESH__REG, &v_data_u8r, 1);
			*gyro_hr_x_thres =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_X_THRESH);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate X axis threshold
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_x_thres
 *
 *      gyro_hr_x_thres
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_threshold(
unsigned char gyro_hr_x_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_THRESH__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_X_THRESH,
					gyro_hr_x_thres);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_THRESH__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate X hysteresis
 *                          register byte from 18h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_x_hys : Pointer holding the
 *                               gyro high rate X hysteresis
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hysteresis(
unsigned char *gyro_hr_x_hys)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_X_HYST__REG, &v_data_u8r, 1);
			*gyro_hr_x_hys =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_X_HYST);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate X axis hysteresis
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_x_hys
 *
 *      gyro_hr_x_hys
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hysteresis(
unsigned char gyro_hr_x_hys)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_HYST__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_X_HYST, gyro_hr_x_hys);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_HYST__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate X duration
 *                          register byte from 18h
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_x_dur : Pointer holding the
 *                               gyro high rate X duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_duration(
unsigned char *gyro_hr_x_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_X_DUR__REG, &v_data_u8r, 1);
			*gyro_hr_x_dur =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_X_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate X axis duration
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_x_dur
 *
 *      gyro_hr_x_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_duration(
unsigned char gyro_hr_x_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_DUR__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_X_DUR,
					gyro_hr_x_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_X_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Y threshold
 *                          register byte from 1Ah
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_y_thres : Pointer holding the
 *                               gyro high rate Y threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_threshold(
unsigned char *gyro_hr_y_thres)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Y_THRESH__REG, &v_data_u8r, 1);
			*gyro_hr_y_thres =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_Y_THRESH);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Y axis threshold
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_y_thres
 *
 *      gyro_hr_y_thres
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_threshold(
unsigned char gyro_hr_y_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_THRESH__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Y_THRESH,
					gyro_hr_y_thres);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_THRESH__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Y hysteresis
 *                          register byte from 1Ah
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_y_hys : Pointer holding the
 *                               gyro high rate Y hysteresis
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hysteresis(
unsigned char *gyro_hr_y_hys)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Y_HYST__REG, &v_data_u8r, 1);
			*gyro_hr_y_hys =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_Y_HYST);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Y axis hysteresis
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_y_hys
 *
 *      gyro_hr_y_hys
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hysteresis(
unsigned char gyro_hr_y_hys)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_HYST__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Y_HYST, gyro_hr_y_hys);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_HYST__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Y duration
 *                          register byte from 1Bh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_y_dur : Pointer holding the
 *                               gyro high rate Y duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_duration(
unsigned char *gyro_hr_y_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Y_DUR__REG, &v_data_u8r, 1);
			*gyro_hr_y_dur =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_Y_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Y axis duration
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_y_dur
 *
 *      gyro_hr_y_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_duration(
unsigned char gyro_hr_y_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_DUR__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Y_DUR, gyro_hr_y_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Y_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Z threshold
 *                          register byte from 1Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_z_thres : Pointer holding the
 *                               gyro high rate Z threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_threshold(
unsigned char *gyro_hr_z_thres)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Z_THRESH__REG, &v_data_u8r, 1);
			*gyro_hr_z_thres =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_Z_THRESH);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Z axis threshold
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_z_thres
 *
 *      gyro_hr_z_thres
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_threshold(
unsigned char gyro_hr_z_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_THRESH__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Z_THRESH,
					gyro_hr_z_thres);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_THRESH__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Z hysteresis
 *                          register byte from 1Ch
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_z_hys : Pointer holding the
 *                               gyro high rate Z hysteresis
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hysteresis(
unsigned char *gyro_hr_z_hys)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Z_HYST__REG, &v_data_u8r, 1);
			*gyro_hr_z_hys =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_HR_Z_HYST);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Z axis hysteresis
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_z_hys
 *
 *      gyro_hr_z_hys
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hysteresis(
unsigned char gyro_hr_z_hys)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_HYST__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Z_HYST,
					gyro_hr_z_hys);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_HYST__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro high rate Z duration
 *                          register byte from 1Dh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_hr_z_dur : Pointer holding the
 *                               gyro high rate Z duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_duration(
unsigned char *gyro_hr_z_dur)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_HR_Z_DUR__REG, &v_data_u8r, 1);
			*gyro_hr_z_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_HR_Z_DUR);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro high rate Z axis duration
 *
 *
 *
 *
 *  \param unsigned char gyro_hr_z_dur
 *
 *      gyro_hr_z_dur
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_duration(
unsigned char gyro_hr_z_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_DUR__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_HR_Z_DUR, gyro_hr_z_dur);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_HR_Z_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion threshold
 *                          register byte from 1Eh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am_thres : Pointer holding the
 *                               gyro any motion threshold
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_threshold(
unsigned char *gyro_am_thres)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_AM_THRES__REG, &v_data_u8r, 1);
			*gyro_am_thres =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYR_AM_THRES);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion threshold
 *
 *
 *
 *
 *  \param unsigned char gyro_am_thres
 *
 *      gyro_am_thres
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_threshold(
unsigned char gyro_am_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AM_THRES__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_AM_THRES, gyro_am_thres);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AM_THRES__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion slope samples
 *                          register byte from 1Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am_slp : Pointer holding the
 *                               gyro any motion slope samples
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_slope_samples(
unsigned char *gyro_am_slp)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYR_SLOPE_SAMPLES__REG, &v_data_u8r, 1);
			*gyro_am_slp = BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYR_SLOPE_SAMPLES);
		} else {
		return ERROR1;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion slope samples
 *
 *
 *
 *
 *  \param unsigned char gyro_am_slp
 *
 *      gyro_am_slp
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_slope_samples(
unsigned char gyro_am_slp)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_SLOPE_SAMPLES__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_SLOPE_SAMPLES, gyro_am_slp);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_SLOPE_SAMPLES__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief Reads gyro any motion Awake duration
 *                          register byte from 1Fh
 *
 *
 *
 *
 *  \param
 *      unsigned char *gyro_am_awk : Pointer holding the
 *                               gyro any motion Awake duration
 *
 *
 *  \return
 *      Result of bus communication function
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_awake_duration(
unsigned char *gyro_am_awk)
{
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
	unsigned char v_data_u8r = BNO055_Zero_U8X;
	unsigned char status = BNO055_Zero_U8X;
	if (p_bno055 == BNO055_Zero_U8X) {
		return E_NULL_PTR;
		} else {
			status = bno055_write_page_id(PAGE_ONE);
			if (status == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYR_AWAKE_DUR__REG, &v_data_u8r, 1);
				*gyro_am_awk = BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYR_AWAKE_DUR);
			} else {
			return ERROR1;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**\brief This API sets the gyro any motion awake duration
 *
 *
 *
 *
 *  \param unsigned char gyro_am_awk
 *
 *      gyro_am_awk
 *
 *
 *
 *  \return Communication results
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_awake_duration(
unsigned char gyro_am_awk)
{
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_Zero_U8X;
unsigned char v_data_u8r = BNO055_Zero_U8X;
unsigned char status = BNO055_Zero_U8X;
unsigned char pg_status = BNO055_Zero_U8X;
unsigned char prev_opmode = BNO055_Zero_U8X;
if (p_bno055 == BNO055_Zero_U8X) {
	return E_NULL_PTR;
} else {
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				pg_status = bno055_write_page_id(PAGE_ONE);
				if (pg_status == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AWAKE_DUR__REG,
					&v_data_u8r, 1);
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYR_AWAKE_DUR, gyro_am_awk);
					comres = p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYR_AWAKE_DUR__REG,
					&v_data_u8r, 1);
				} else {
				comres = ERROR1;
				}
			} else {
			return ERROR1;
			}
		} else {
		return ERROR1;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG) {
		/* set the operation mode
		of previous operation mode*/
		bno055_set_operation_mode(prev_opmode);
	} else {
	return ERROR1;
	}
	return comres;
}
