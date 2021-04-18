/*
 ***************************************************************************
 *
 *  BNO055.h - part of sample SW for using BNO055 with Arduino
 * 
 * Usage:        BNO055 Sensor Driver Header File
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
/*! \file BNO050.h
    \brief BNO055 Sensor Driver Support Header File */

#ifndef __BNO055_H__
#define __BNO055_H__

#include <limits.h> /* needed to test integer limits */


/* find correct data type for signed/unsigned 16 bit
   variables by checking max of unsigned variant */
#if USHRT_MAX == 0xFFFF
	/* 16 bit achieved with short */
	#define BNO055_U16 unsigned short
    #define BNO055_S16 signed short
#elif UINT_MAX == 0xFFFF
	/* 16 bit achieved with int */
	#define BNO055_U16 unsigned int
	#define BNO055_S16 signed int
#else
	#error BNO055_U16 and BNO055_S16 could not be defined automatically, please do so manually
#endif

/* For Enabling and Disabling the floating point API's */
#define ENABLE_FLOAT

#define BNO055_U16 unsigned short
#define BNO055_S16 signed short
#define BNO055_S32 signed int


#define BNO055_WR_FUNC_PTR int (*bus_write)(unsigned char, unsigned char , unsigned char *, unsigned char)

#define BNO055_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)bus_write(dev_addr, reg_addr, reg_data, wr_len)

#define BNO055_RD_FUNC_PTR int (*bus_read)(unsigned char, unsigned char , unsigned char *, unsigned char)

#define BNO055_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)bus_read(dev_addr, reg_addr, reg_data, r_len)



#define BNO055_DELAY_RETURN_TYPE void

#define BNO055_DELAY_PARAM_TYPES unsigned int

#define BNO055_DELAY_FUNC(delay_in_msec)\
	delay_func(delay_in_msec)


/* bno055 I2C Address */
#define BNO055_I2C_ADDR1                0x28
#define BNO055_I2C_ADDR2                0x29
#define BNO055_I2C_ADDR                 BNO055_I2C_ADDR1


/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 0x00
#define BNO055_ACC_REV_ID_ADDR              0x01
#define BNO055_MAG_REV_ID_ADDR              0x02
#define BNO055_GYR_REV_ID_ADDR              0x03
#define BNO055_SW_REV_ID_LSB_ADDR			0x04
#define BNO055_SW_REV_ID_MSB_ADDR			0x05
#define BNO055_BL_Rev_ID_ADDR				0X06
#define BNO055_Page_ID_ADDR				    0X07

/* Accel data register*/
#define BNO055_ACC_DATA_X_LSB_ADDR			0X08
#define BNO055_ACC_DATA_X_MSB_ADDR			0X09
#define BNO055_ACC_DATA_Y_LSB_ADDR			0X0A
#define BNO055_ACC_DATA_Y_MSB_ADDR			0X0B
#define BNO055_ACC_DATA_Z_LSB_ADDR			0X0C
#define BNO055_ACC_DATA_Z_MSB_ADDR			0X0D

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR			0X13

/*Gyro data registers*/
#define BNO055_GYR_DATA_X_LSB_ADDR			0X14
#define BNO055_GYR_DATA_X_MSB_ADDR			0X15
#define BNO055_GYR_DATA_Y_LSB_ADDR			0X16
#define BNO055_GYR_DATA_Y_MSB_ADDR			0X17
#define BNO055_GYR_DATA_Z_LSB_ADDR			0X18
#define BNO055_GYR_DATA_Z_MSB_ADDR			0X19

/*Euler data registers*/
#define BNO055_EUL_HEADING_LSB_ADDR			0X1A
#define BNO055_EUL_HEADING_MSB_ADDR			0X1B

#define BNO055_EUL_ROLL_LSB_ADDR			0X1C
#define BNO055_EUL_ROLL_MSB_ADDR			0X1D

#define BNO055_EUL_PITCH_LSB_ADDR			0X1E
#define BNO055_EUL_PITCH_MSB_ADDR			0X1F

/*Quaternion data registers*/
#define BNO055_QUA_DATA_W_LSB_ADDR			0X20
#define BNO055_QUA_DATA_W_MSB_ADDR			0X21
#define BNO055_QUA_DATA_X_LSB_ADDR			0X22
#define BNO055_QUA_DATA_X_MSB_ADDR			0X23
#define BNO055_QUA_DATA_Y_LSB_ADDR			0X24
#define BNO055_QUA_DATA_Y_MSB_ADDR			0X25
#define BNO055_QUA_DATA_Z_LSB_ADDR			0X26
#define BNO055_QUA_DATA_Z_MSB_ADDR			0X27

/* Linear acceleration data registers*/
#define BNO055_LIA_DATA_X_LSB_ADDR			0X28
#define BNO055_LIA_DATA_X_MSB_ADDR			0X29
#define BNO055_LIA_DATA_Y_LSB_ADDR			0X2A
#define BNO055_LIA_DATA_Y_MSB_ADDR			0X2B
#define BNO055_LIA_DATA_Z_LSB_ADDR			0X2C
#define BNO055_LIA_DATA_Z_MSB_ADDR			0X2D

/*Gravity data registers*/
#define BNO055_GRV_DATA_X_LSB_ADDR			0X2E
#define BNO055_GRV_DATA_X_MSB_ADDR			0X2F
#define BNO055_GRV_DATA_Y_LSB_ADDR			0X30
#define BNO055_GRV_DATA_Y_MSB_ADDR			0X31
#define BNO055_GRV_DATA_Z_LSB_ADDR			0X32
#define BNO055_GRV_DATA_Z_MSB_ADDR			0X33

/* Temperature data register*/

#define BNO055_TEMP_ADDR					0X34

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR				0X35
#define BNO055_ST_RESULT_ADDR				0X36
#define BNO055_INT_STA_ADDR					0X37
#define BNO055_SYS_STATUS_ADDR				0X39
#define BNO055_SYS_ERR_ADDR					0X3A

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR				0X3B
#define BNO055_DATA_SEL_ADDR				0X3C

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR				0X3D
#define BNO055_PWR_MODE_ADDR				0X3E

#define BNO055_SYS_TRIGGER_ADDR				0X3F
#define BNO055_TEMP_SOURCE_ADDR				0X40
/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR			0X41
#define BNO055_AXIS_MAP_SIGN_ADDR			0X42

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR		0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR		0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR		0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR		0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR		0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR		0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR		0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR		0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR		0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR		0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR		0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR		0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR		0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR		0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR		0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR		0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR		0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR		0X54

/* Accelerometer Offset registers*/
#define ACC_OFFSET_X_LSB_ADDR				0X55
#define ACC_OFFSET_X_MSB_ADDR				0X56
#define ACC_OFFSET_Y_LSB_ADDR				0X57
#define ACC_OFFSET_Y_MSB_ADDR				0X58
#define ACC_OFFSET_Z_LSB_ADDR				0X59
#define ACC_OFFSET_Z_MSB_ADDR				0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR				0X5B
#define MAG_OFFSET_X_MSB_ADDR				0X5C
#define MAG_OFFSET_Y_LSB_ADDR				0X5D
#define MAG_OFFSET_Y_MSB_ADDR				0X5E
#define MAG_OFFSET_Z_LSB_ADDR				0X5F
#define MAG_OFFSET_Z_MSB_ADDR				0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR				0X61
#define GYRO_OFFSET_X_MSB_ADDR				0X62
#define GYRO_OFFSET_Y_LSB_ADDR				0X63
#define GYRO_OFFSET_Y_MSB_ADDR				0X64
#define GYRO_OFFSET_Z_LSB_ADDR				0X65
#define GYRO_OFFSET_Z_MSB_ADDR				0X66

/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACC_CONFIG_ADDR						0X08
#define MAG_CONFIG_ADDR						0X09
#define GYRO_CONFIG_ADDR					0X0A
#define GYRO_MODE_CONFIG_ADDR				0X0B
#define ACC_SLEEP_CONFIG_ADDR				0X0C
#define GYR_SLEEP_CONFIG_ADDR				0X0D
#define MAG_SLEEP_CONFIG_ADDR				0x0E

/* Interrupt registers*/
#define INT_MSK_ADDR						0X0F
#define INT_ADDR							0X10
#define ACC_AM_THRES_ADDR					0X11
#define ACC_INT_SETTINGS_ADDR				0X12
#define ACC_HG_DURATION_ADDR				0X13
#define ACC_HG_THRES_ADDR					0X14
#define ACC_NM_THRES_ADDR					0X15
#define ACC_NM_SET_ADDR						0X16
#define GYR_INT_SETING_ADDR					0X17
#define GYR_HR_X_SET_ADDR					0X18
#define GYR_DUR_X_ADDR						0X19
#define GYR_HR_Y_SET_ADDR					0X1A
#define GYR_DUR_Y_ADDR						0X1B
#define GYR_HR_Z_SET_ADDR					0X1C
#define GYR_DUR_Z_ADDR						0X1D
#define GYR_AM_THRES_ADDR					0X1E
#define GYR_AM_SET_ADDR						0X1F
/* PAGE1 REGISTERS DEFINITION END*/


#define BNO055_MDELAY_DATA_TYPE           unsigned int

/*< This refers BNO055 return type as int */
#define BNO055_RETURN_FUNCTION_TYPE        int


/*BNO055-STRUCT*/
struct bno055_t {
unsigned char chip_id;
unsigned char sw_revision_id;
unsigned char page_id;
unsigned char accel_revision_id;
unsigned char mag_revision_id;
unsigned char gyro_revision_id;
unsigned char bootloader_revision_id;
unsigned char dev_addr;
BNO055_WR_FUNC_PTR;
BNO055_RD_FUNC_PTR;
void (*delay_msec)(BNO055_MDELAY_DATA_TYPE);
};

/*BNO055-Accel x,y,z*/
struct bno055_accel {
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};

/*BNO055-mag x,y,z*/
struct bno055_mag {
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};

/*BNO055-Gyro x,y,z*/
struct bno055_gyro {
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};

/*BNO055-Euler h,r,p*/
struct bno055_euler {
BNO055_S16 h;
BNO055_S16 r;
BNO055_S16 p;

};

/*BNO055-quaternion w,x,y,z*/
struct bno055_quaternion {
BNO055_S16 w;
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};

/*BNO055-Linear Accel x,y,z*/
struct bno055_linear_accel {
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};

/*BNO055-Gravity x,y,z*/
struct bno055_gravity {
BNO055_S16 x;
BNO055_S16 y;
BNO055_S16 z;

};


#define         BNO055_Zero_U8X           (unsigned char)0
#define         BNO055_Two_U8X			  (unsigned char)2
#define         BNO055_Four_U8X           (unsigned char)4
#define         BNO055_Five_U8X           (unsigned char)5
#define         BNO055_Six_U8X            (unsigned char)6
#define         BNO055_Seven_U8X          (unsigned char)7
#define         BNO055_Eleven_U8X         (unsigned char)11
#define         BNO055_Sixteen_U8X        (unsigned char)16
#define			BNO055_Eight_U8X		  (unsigned char)8


#define         BNO055_SHIFT_8_POSITION	   (unsigned char)8


/*  BNO055 API error codes */
#define E_NULL_PTR                  (char)(-127)
#define E_BNO055_OUT_OF_RANGE       (signed char)(-2)
#define	SUCCESS						(unsigned char)0
#define	ERROR1						(signed char)(-1)

/* Page ID */
#define PAGE_ZERO		0X00
#define PAGE_ONE		0X01


/* Operation mode settings*/
#define OPERATION_MODE_CONFIG			0X00
#define OPERATION_MODE_ACCONLY			0X01
#define OPERATION_MODE_MAGONLY			0X02
#define OPERATION_MODE_GYRONLY			0X03
#define OPERATION_MODE_ACCMAG			0X04
#define OPERATION_MODE_ACCGYRO			0X05
#define OPERATION_MODE_MAGGYRO			0X06
#define OPERATION_MODE_AMG				0X07
#define OPERATION_MODE_IMUPLUS			0X08
#define OPERATION_MODE_COMPASS			0X09
#define OPERATION_MODE_M4G				0X0A
#define OPERATION_MODE_NDOF_FMC_OFF		0X0B
#define OPERATION_MODE_NDOF				0X0C

/* BNO055 Power mode settings*/
#define POWER_MODE_NORMAL 		0x00
#define POWER_MODE_LOW_POWER 	0x01
#define POWER_MODE_SUSPEND		0x02

/* Data output rates*/
#define FASTEST_MODE_1		0X00
#define FASTEST_MODE_2		0X01
#define GAME_MODE			0X02
#define UI_MODE				0X04
#define NORMAL_MODE		0X05

/* PAGE-1 definitions*/
/* Accel Range */

#define ACCEL_RANGE_2G		0X00
#define ACCEL_RANGE_4G		0X01
#define ACCEL_RANGE_8G		0X02
#define ACCEL_RANGE_16G		0X03

/* Accel Bandwidth*/
#define ACCEL_BW_7_81Hz		0x00
#define ACCEL_BW_15_63Hz	0x01
#define ACCEL_BW_31_25Hz	0x02
#define ACCEL_BW_62_5Hz		0X03
#define ACCEL_BW_125Hz		0X04
#define ACCEL_BW_250Hz		0X05
#define ACCEL_BW_500Hz		0X06
#define ACCEL_BW_1000Hz		0X07

/* Accel Power mode*/
#define ACCEL_NORMAL			0X00
#define ACCEL_SUSPEND			0X01
#define ACCEL_LOWPOWER_1		0X02
#define ACCEL_STANDBY			0X03
#define ACCEL_LOWPOWER_2		0X04
#define ACCEL_DEEPSUSPEND		0X05

/* Mag data output rate*/
#define MAG_DATA_OUTRATE_2Hz		0X00
#define MAG_DATA_OUTRATE_6Hz		0X01
#define MAG_DATA_OUTRATE_8Hz		0X02
#define MAG_DATA_OUTRATE_10Hz		0X03
#define MAG_DATA_OUTRATE_15Hz		0X04
#define MAG_DATA_OUTRATE_20Hz		0X05
#define MAG_DATA_OUTRATE_25Hz		0X06
#define MAG_DATA_OUTRATE_30Hz		0X07

/* Mag Operation mode*/
#define MAG_OPR_MODE_LOWPOWER				0X00
#define MAG_OPR_MODE_REGULAR				0X01
#define MAG_OPR_MODE_ENHANCED_REGULAR		0X02
#define MAG_OPR_MODE_HIGH_ACCURACY			0X03

/* Mag power mode*/
#define MAG_POWER_MODE_NORMAL					0X00
#define MAG_POWER_MODE_SLEEP					0X01
#define MAG_POWER_MODE_SUSPEND					0X02
#define MAG_POWER_MODE_FORCE_MODE				0X03

/* Gyro range*/
#define GYRO_RANGE_2000rps		0x00
#define GYRO_RANGE_1000rps		0x01
#define GYRO_RANGE_500rps		0x02
#define GYRO_RANGE_250rps		0x03
#define GYRO_RANGE_125rps		0x04

/* Gyro Bandwidth*/
#define GYRO_BW_523Hz		0x00
#define GYRO_BW_230Hz		0x01
#define GYRO_BW_116Hz		0x02
#define GYRO_BW_47Hz		0x03
#define GYRO_BW_23Hz		0x04
#define GYRO_BW_12Hz		0x05
#define GYRO_BW_64Hz		0x06
#define GYRO_BW_32Hz		0x07

/* Gyro Operation mode*/
#define GYRO_OPR_MODE_NORMAL				0X00
#define GYRO_OPR_MODE_FASTPOWERUP			0X01
#define GYRO_OPR_MODE_DEEPSUSPEND			0X02
#define GYRO_OPR_MODE_SUSPEND				0X03
#define GYRO_OPR_MODE_ADVANCE_POWERSAVE		0X04

/* Accel Sleep Duration */

#define BNO055_ACCEL_SLEEP_DUR_0_5MS        0x05
/* sets sleep duration to 0.5 ms  */
#define BNO055_ACCEL_SLEEP_DUR_1MS          0x06
/* sets sleep duration to 1 ms */
#define BNO055_ACCEL_SLEEP_DUR_2MS          0x07
/* sets sleep duration to 2 ms */
#define BNO055_ACCEL_SLEEP_DUR_4MS          0x08
/* sets sleep duration to 4 ms */
#define BNO055_ACCEL_SLEEP_DUR_6MS          0x09
/* sets sleep duration to 6 ms*/
#define BNO055_ACCEL_SLEEP_DUR_10MS         0x0A
/* sets sleep duration to 10 ms */
#define BNO055_ACCEL_SLEEP_DUR_25MS         0x0B
 /* sets sleep duration to 25 ms */
#define BNO055_ACCEL_SLEEP_DUR_50MS         0x0C
 /* sets sleep duration to 50 ms */
#define BNO055_ACCEL_SLEEP_DUR_100MS        0x0D
 /* sets sleep duration to 100 ms */
#define BNO055_ACCEL_SLEEP_DUR_500MS        0x0E
 /* sets sleep duration to 500 ms */
#define BNO055_ACCEL_SLEEP_DUR_1S           0x0F
/* sets sleep duration to 1 s */

/* Gyro Auto sleep duration*/
#define BNO055_GYRO_No_AuSlpDur		0x00
#define	BNO055_GYRO_4ms_AuSlpDur	0x01
#define	BNO055_GYRO_5ms_AuSlpDur	0x02
#define	BNO055_GYRO_8ms_AuSlpDur	0x03
#define	BNO055_GYRO_10ms_AuSlpDur	0x04
#define	BNO055_GYRO_15ms_AuSlpDur	0x05
#define	BNO055_GYRO_20ms_AuSlpDur	0x06
#define	BNO055_GYRO_40ms_AuSlpDur	0x07

/* Accel Any/No motion axis selection*/
#define BNO055_ACCEL_AM_NM_X_AXIS		0
#define BNO055_ACCEL_AM_NM_Y_AXIS		1
#define BNO055_ACCEL_AM_NM_Z_AXIS		2

/* Accel High g axis selection*/
#define BNO055_ACCEL_HIGH_G_X_AXIS		0
#define BNO055_ACCEL_HIGH_G_Y_AXIS		1
#define BNO055_ACCEL_HIGH_G_Z_AXIS		2

/* Gyro Any motion axis selection*/
#define BNO055_GYRO_AM_X_AXIS		0
#define BNO055_GYRO_AM_Y_AXIS		1
#define BNO055_GYRO_AM_Z_AXIS		2


/* Gyro High rate axis selection*/
#define BNO055_GYRO_HR_X_AXIS		0
#define BNO055_GYRO_HR_Y_AXIS		1
#define BNO055_GYRO_HR_Z_AXIS		2

/* Axis remap values*/
#define REMAP_X_Y			0X21
#define REMAP_Y_Z			0X18
#define REMAP_Z_X			0X06
#define REMAP_X_Y_Z_TYPE0	0X12
#define REMAP_X_Y_Z_TYPE1	0X09
#define DEFAULT_AXIS		0X24


/* PAGE0 DATA REGISTERS DEFINITION START*/
/* Chip ID*/
#define BNO055_CHIP_ID__POS             0
#define BNO055_CHIP_ID__MSK             0xFF
#define BNO055_CHIP_ID__LEN             8
#define BNO055_CHIP_ID__REG             BNO055_CHIP_ID_ADDR

/* Accel revision ID*/
#define BNO055_ACC_REV_ID__POS             0
#define BNO055_ACC_REV_ID__MSK             0xFF
#define BNO055_ACC_REV_ID__LEN             8
#define BNO055_ACC_REV_ID__REG             BNO055_ACC_REV_ID_ADDR

/* Mag revision ID*/
#define BNO055_MAG_REV_ID__POS             0
#define BNO055_MAG_REV_ID__MSK             0xFF
#define BNO055_MAG_REV_ID__LEN             8
#define BNO055_MAG_REV_ID__REG             BNO055_MAG_REV_ID_ADDR

/* Gyro revision ID*/
#define BNO055_GYR_REV_ID__POS             0
#define BNO055_GYR_REV_ID__MSK             0xFF
#define BNO055_GYR_REV_ID__LEN             8
#define BNO055_GYR_REV_ID__REG             BNO055_GYR_REV_ID_ADDR

/*Software revision ID LSB*/
#define BNO055_SW_REV_ID_LSB__POS             0
#define BNO055_SW_REV_ID_LSB__MSK             0xFF
#define BNO055_SW_REV_ID_LSB__LEN             8
#define BNO055_SW_REV_ID_LSB__REG             BNO055_SW_REV_ID_LSB_ADDR

/*Software revision ID MSB*/
#define BNO055_SW_REV_ID_MSB__POS             0
#define BNO055_SW_REV_ID_MSB__MSK             0xFF
#define BNO055_SW_REV_ID_MSB__LEN             8
#define BNO055_SW_REV_ID_MSB__REG             BNO055_SW_REV_ID_MSB_ADDR

/* BOOTLOADER revision ID*/
#define BNO055_BL_Rev_ID__POS             0
#define BNO055_BL_Rev_ID__MSK             0xFF
#define BNO055_BL_Rev_ID__LEN             8
#define BNO055_BL_Rev_ID__REG             BNO055_BL_Rev_ID_ADDR

/*Page ID*/
#define BNO055_Page_ID__POS             0
#define BNO055_Page_ID__MSK             0xFF
#define BNO055_Page_ID__LEN             8
#define BNO055_Page_ID__REG             BNO055_Page_ID_ADDR

/* Accel data X-LSB register*/
#define BNO055_ACC_DATA_X_LSB_VALUEX__POS             0
#define BNO055_ACC_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_ACC_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_ACC_DATA_X_LSB_VALUEX__REG             BNO055_ACC_DATA_X_LSB_ADDR

/* Accel data X-MSB register*/
#define BNO055_ACC_DATA_X_MSB_VALUEX__POS             0
#define BNO055_ACC_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_ACC_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_ACC_DATA_X_MSB_VALUEX__REG             BNO055_ACC_DATA_X_MSB_ADDR

/* Accel data Y-LSB register*/
#define BNO055_ACC_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_ACC_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_ACC_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_ACC_DATA_Y_LSB_VALUEY__REG             BNO055_ACC_DATA_Y_LSB_ADDR

/* Accel data Y-MSB register*/
#define BNO055_ACC_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_ACC_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_ACC_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_ACC_DATA_Y_MSB_VALUEY__REG             BNO055_ACC_DATA_Y_MSB_ADDR

/* Accel data Z-LSB register*/
#define BNO055_ACC_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_ACC_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_ACC_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_ACC_DATA_Z_LSB_VALUEZ__REG             BNO055_ACC_DATA_Z_LSB_ADDR

/* Accel data Z-MSB register*/
#define BNO055_ACC_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_ACC_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_ACC_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_ACC_DATA_Z_MSB_VALUEZ__REG             BNO055_ACC_DATA_Z_MSB_ADDR

/* Mag data X-LSB register*/
#define BNO055_MAG_DATA_X_LSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_LSB_VALUEX__REG             BNO055_MAG_DATA_X_LSB_ADDR

/* Mag data X-MSB register*/
#define BNO055_MAG_DATA_X_MSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_MSB_VALUEX__REG             BNO055_MAG_DATA_X_MSB_ADDR

/* Mag data Y-LSB register*/
#define BNO055_MAG_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_LSB_VALUEY__REG             BNO055_MAG_DATA_Y_LSB_ADDR

/* Mag data Y-MSB register*/
#define BNO055_MAG_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_MSB_VALUEY__REG             BNO055_MAG_DATA_Y_MSB_ADDR

/* Mag data Z-LSB register*/
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__REG             BNO055_MAG_DATA_Z_LSB_ADDR

/* Mag data Z-MSB register*/
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__REG             BNO055_MAG_DATA_Z_MSB_ADDR

/* Gyro data X-LSB register*/
#define BNO055_GYR_DATA_X_LSB_VALUEX__POS             0
#define BNO055_GYR_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_GYR_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_GYR_DATA_X_LSB_VALUEX__REG             BNO055_GYR_DATA_X_LSB_ADDR

/* Gyro data X-MSB register*/
#define BNO055_GYR_DATA_X_MSB_VALUEX__POS             0
#define BNO055_GYR_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_GYR_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_GYR_DATA_X_MSB_VALUEX__REG             BNO055_GYR_DATA_X_MSB_ADDR

/* Gyro data Y-LSB register*/
#define BNO055_GYR_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_GYR_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_GYR_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_GYR_DATA_Y_LSB_VALUEY__REG             BNO055_GYR_DATA_Y_LSB_ADDR

/* Gyro data Y-MSB register*/
#define BNO055_GYR_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_GYR_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_GYR_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_GYR_DATA_Y_MSB_VALUEY__REG             BNO055_GYR_DATA_Y_MSB_ADDR

/* Gyro data Z-LSB register*/
#define BNO055_GYR_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_GYR_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_GYR_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_GYR_DATA_Z_LSB_VALUEZ__REG             BNO055_GYR_DATA_Z_LSB_ADDR

/* Gyro data Z-MSB register*/
#define BNO055_GYR_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_GYR_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_GYR_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_GYR_DATA_Z_MSB_VALUEZ__REG             BNO055_GYR_DATA_Z_MSB_ADDR

/* Euler data HEADING-LSB register*/
#define BNO055_EUL_HEADING_LSB_VALUEH__POS             0
#define BNO055_EUL_HEADING_LSB_VALUEH__MSK             0xFF
#define BNO055_EUL_HEADING_LSB_VALUEH__LEN             8
#define BNO055_EUL_HEADING_LSB_VALUEH__REG         BNO055_EUL_HEADING_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EUL_HEADING_MSB_VALUEH__POS             0
#define BNO055_EUL_HEADING_MSB_VALUEH__MSK             0xFF
#define BNO055_EUL_HEADING_MSB_VALUEH__LEN             8
#define BNO055_EUL_HEADING_MSB_VALUEH__REG        BNO055_EUL_HEADING_MSB_ADDR

/* Euler data ROLL-LSB register*/
#define BNO055_EUL_ROLL_LSB_VALUER__POS             0
#define BNO055_EUL_ROLL_LSB_VALUER__MSK             0xFF
#define BNO055_EUL_ROLL_LSB_VALUER__LEN             8
#define BNO055_EUL_ROLL_LSB_VALUER__REG             BNO055_EUL_ROLL_LSB_ADDR

/* Euler data ROLL-MSB register*/
#define BNO055_EUL_ROLL_MSB_VALUER__POS             0
#define BNO055_EUL_ROLL_MSB_VALUER__MSK             0xFF
#define BNO055_EUL_ROLL_MSB_VALUER__LEN             8
#define BNO055_EUL_ROLL_MSB_VALUER__REG             BNO055_EUL_ROLL_MSB_ADDR

/* Euler data PITCH-LSB register*/
#define BNO055_EUL_PITCH_LSB_VALUEP__POS             0
#define BNO055_EUL_PITCH_LSB_VALUEP__MSK             0xFF
#define BNO055_EUL_PITCH_LSB_VALUEP__LEN             8
#define BNO055_EUL_PITCH_LSB_VALUEP__REG             BNO055_EUL_PITCH_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EUL_PITCH_MSB_VALUEP__POS             0
#define BNO055_EUL_PITCH_MSB_VALUEP__MSK             0xFF
#define BNO055_EUL_PITCH_MSB_VALUEP__LEN             8
#define BNO055_EUL_PITCH_MSB_VALUEP__REG             BNO055_EUL_PITCH_MSB_ADDR

/* Quaternion data W-LSB register*/
#define BNO055_QUA_DATA_W_LSB_VALUEW__POS             0
#define BNO055_QUA_DATA_W_LSB_VALUEW__MSK             0xFF
#define BNO055_QUA_DATA_W_LSB_VALUEW__LEN             8
#define BNO055_QUA_DATA_W_LSB_VALUEW__REG             BNO055_QUA_DATA_W_LSB_ADDR

/* Quaternion data W-MSB register*/
#define BNO055_QUA_DATA_W_MSB_VALUEW__POS             0
#define BNO055_QUA_DATA_W_MSB_VALUEW__MSK             0xFF
#define BNO055_QUA_DATA_W_MSB_VALUEW__LEN             8
#define BNO055_QUA_DATA_W_MSB_VALUEW__REG             BNO055_QUA_DATA_W_MSB_ADDR

/* Quaternion data X-LSB register*/
#define BNO055_QUA_DATA_X_LSB_VALUEX__POS             0
#define BNO055_QUA_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_QUA_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_QUA_DATA_X_LSB_VALUEX__REG             BNO055_QUA_DATA_X_LSB_ADDR

/* Quaternion data X-MSB register*/
#define BNO055_QUA_DATA_X_MSB_VALUEX__POS             0
#define BNO055_QUA_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_QUA_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_QUA_DATA_X_MSB_VALUEX__REG             BNO055_QUA_DATA_X_MSB_ADDR

/* Quaternion data Y-LSB register*/
#define BNO055_QUA_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_QUA_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_QUA_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_QUA_DATA_Y_LSB_VALUEY__REG             BNO055_QUA_DATA_Y_LSB_ADDR

/* Quaternion data Y-MSB register*/
#define BNO055_QUA_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_QUA_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_QUA_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_QUA_DATA_Y_MSB_VALUEY__REG             BNO055_QUA_DATA_Y_MSB_ADDR

/* Quaternion data Z-LSB register*/
#define BNO055_QUA_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_QUA_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_QUA_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_QUA_DATA_Z_LSB_VALUEZ__REG             BNO055_QUA_DATA_Z_LSB_ADDR

/* Quaternion data Z-MSB register*/
#define BNO055_QUA_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_QUA_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_QUA_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_QUA_DATA_Z_MSB_VALUEZ__REG             BNO055_QUA_DATA_Z_MSB_ADDR

/* Linear acceleration data X-LSB register*/
#define BNO055_LIA_DATA_X_LSB_VALUEX__POS             0
#define BNO055_LIA_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_LIA_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_LIA_DATA_X_LSB_VALUEX__REG             BNO055_LIA_DATA_X_LSB_ADDR

/* Linear acceleration data X-MSB register*/
#define BNO055_LIA_DATA_X_MSB_VALUEX__POS             0
#define BNO055_LIA_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_LIA_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_LIA_DATA_X_MSB_VALUEX__REG             BNO055_LIA_DATA_X_MSB_ADDR

/* Linear acceleration data Y-LSB register*/
#define BNO055_LIA_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_LIA_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_LIA_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_LIA_DATA_Y_LSB_VALUEY__REG             BNO055_LIA_DATA_Y_LSB_ADDR

/* Linear acceleration data Y-MSB register*/
#define BNO055_LIA_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_LIA_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_LIA_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_LIA_DATA_Y_MSB_VALUEY__REG             BNO055_LIA_DATA_Y_MSB_ADDR

/* Linear acceleration data Z-LSB register*/
#define BNO055_LIA_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_LIA_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_LIA_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_LIA_DATA_Z_LSB_VALUEZ__REG             BNO055_LIA_DATA_Z_LSB_ADDR

/* Linear acceleration data Z-MSB register*/
#define BNO055_LIA_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_LIA_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_LIA_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_LIA_DATA_Z_MSB_VALUEZ__REG             BNO055_LIA_DATA_Z_MSB_ADDR

/* Gravity data X-LSB register*/
#define BNO055_GRV_DATA_X_LSB_VALUEX__POS             0
#define BNO055_GRV_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_GRV_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_GRV_DATA_X_LSB_VALUEX__REG             BNO055_GRV_DATA_X_LSB_ADDR

/* Gravity data X-MSB register*/
#define BNO055_GRV_DATA_X_MSB_VALUEX__POS             0
#define BNO055_GRV_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_GRV_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_GRV_DATA_X_MSB_VALUEX__REG             BNO055_GRV_DATA_X_MSB_ADDR

/* Gravity data Y-LSB register*/
#define BNO055_GRV_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_GRV_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_GRV_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_GRV_DATA_Y_LSB_VALUEY__REG             BNO055_GRV_DATA_Y_LSB_ADDR

/* Gravity data Y-MSB register*/
#define BNO055_GRV_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_GRV_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_GRV_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_GRV_DATA_Y_MSB_VALUEY__REG             BNO055_GRV_DATA_Y_MSB_ADDR

/* Gravity data Z-LSB register*/
#define BNO055_GRV_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_GRV_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_GRV_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_GRV_DATA_Z_LSB_VALUEZ__REG             BNO055_GRV_DATA_Z_LSB_ADDR

/* Gravity data Z-MSB register*/
#define BNO055_GRV_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_GRV_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_GRV_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_GRV_DATA_Z_MSB_VALUEZ__REG             BNO055_GRV_DATA_Z_MSB_ADDR

/* Temperature register*/
#define BNO055_TEMP__POS             0
#define BNO055_TEMP__MSK             0xFF
#define BNO055_TEMP__LEN             8
#define BNO055_TEMP__REG             BNO055_TEMP_ADDR

/*Mag_Calib status register*/
#define BNO055_MAG_CALIB_STAT__POS             0
#define BNO055_MAG_CALIB_STAT__MSK             0X03
#define BNO055_MAG_CALIB_STAT__LEN             2
#define BNO055_MAG_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Acc_Calib status register*/
#define BNO055_ACC_CALIB_STAT__POS             2
#define BNO055_ACC_CALIB_STAT__MSK             0X0C
#define BNO055_ACC_CALIB_STAT__LEN             2
#define BNO055_ACC_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Gyro_Calib status register*/
#define BNO055_GYR_CALIB_STAT__POS             4
#define BNO055_GYR_CALIB_STAT__MSK             0X30
#define BNO055_GYR_CALIB_STAT__LEN             2
#define BNO055_GYR_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Sys_Calib status register*/
#define BNO055_SYS_CALIB_STAT__POS             6
#define BNO055_SYS_CALIB_STAT__MSK             0XC0
#define BNO055_SYS_CALIB_STAT__LEN             2
#define BNO055_SYS_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*ST_ACCEL register*/
#define BNO055_ST_ACC__POS             0
#define BNO055_ST_ACC__MSK             0X01
#define BNO055_ST_ACC__LEN             1
#define BNO055_ST_ACC__REG             BNO055_ST_RESULT_ADDR

/*ST_MAG register*/
#define BNO055_ST_MAG__POS             1
#define BNO055_ST_MAG__MSK             0X02
#define BNO055_ST_MAG__LEN             1
#define BNO055_ST_MAG__REG             BNO055_ST_RESULT_ADDR

/*ST_GYRO register*/
#define BNO055_ST_GYR__POS             2
#define BNO055_ST_GYR__MSK             0X04
#define BNO055_ST_GYR__LEN             1
#define BNO055_ST_GYR__REG             BNO055_ST_RESULT_ADDR

/*ST_MCU register*/
#define BNO055_ST_MCU__POS             3
#define BNO055_ST_MCU__MSK             0X08
#define BNO055_ST_MCU__LEN             1
#define BNO055_ST_MCU__REG             BNO055_ST_RESULT_ADDR

/*Interrupt status registers*/
#define BNO055_INT_STAT_GYRO_AM__POS	2
#define BNO055_INT_STAT_GYRO_AM__MSK	0X04
#define BNO055_INT_STAT_GYRO_AM__LEN	1
#define BNO055_INT_STAT_GYRO_AM__REG	BNO055_INT_STA_ADDR

#define BNO055_INT_STAT_GYRO_HIGH_RATE__POS		3
#define BNO055_INT_STAT_GYRO_HIGH_RATE__MSK		0X08
#define BNO055_INT_STAT_GYRO_HIGH_RATE__LEN		1
#define BNO055_INT_STAT_GYRO_HIGH_RATE__REG		BNO055_INT_STA_ADDR

#define BNO055_INT_STAT_ACC_HIGH_G__POS		5
#define BNO055_INT_STAT_ACC_HIGH_G__MSK		0X20
#define BNO055_INT_STAT_ACC_HIGH_G__LEN		1
#define BNO055_INT_STAT_ACC_HIGH_G__REG		BNO055_INT_STA_ADDR

#define BNO055_INT_STAT_ACC_AM__POS		6
#define BNO055_INT_STAT_ACC_AM__MSK		0X40
#define BNO055_INT_STAT_ACC_AM__LEN		1
#define BNO055_INT_STAT_ACC_AM__REG		BNO055_INT_STA_ADDR

#define BNO055_INT_STAT_ACC_NM__POS		7
#define BNO055_INT_STAT_ACC_NM__MSK		0X80
#define BNO055_INT_STAT_ACC_NM__LEN		1
#define BNO055_INT_STAT_ACC_NM__REG		BNO055_INT_STA_ADDR

/* System registers*/
#define BNO055_SYSTEM_STATUS_CODE__POS		0
#define BNO055_SYSTEM_STATUS_CODE__MSK		0XFF
#define BNO055_SYSTEM_STATUS_CODE__LEN		8
#define BNO055_SYSTEM_STATUS_CODE__REG		BNO055_SYS_STATUS_ADDR

#define BNO055_SYSTEM_ERROR_CODE__POS			0
#define BNO055_SYSTEM_ERROR_CODE__MSK			0XFF
#define BNO055_SYSTEM_ERROR_CODE__LEN			8
#define BNO055_SYSTEM_ERROR_CODE__REG			BNO055_SYS_ERR_ADDR

/* Accel_Unit register*/
#define BNO055_ACC_UNIT__POS             0
#define BNO055_ACC_UNIT__MSK             0X01
#define BNO055_ACC_UNIT__LEN             1
#define BNO055_ACC_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Gyro_Unit register*/
#define BNO055_GYR_UNIT__POS             1
#define BNO055_GYR_UNIT__MSK             0X02
#define BNO055_GYR_UNIT__LEN             1
#define BNO055_GYR_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Euler_Unit register*/
#define BNO055_EUL_UNIT__POS             2
#define BNO055_EUL_UNIT__MSK             0X04
#define BNO055_EUL_UNIT__LEN             1
#define BNO055_EUL_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Tilt_Unit register*/
#define BNO055_TILT_UNIT__POS             3
#define BNO055_TILT_UNIT__MSK             0X08
#define BNO055_TILT_UNIT__LEN             1
#define BNO055_TILT_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Temperature_Unit register*/
#define BNO055_TEMP_UNIT__POS             4
#define BNO055_TEMP_UNIT__MSK             0X10
#define BNO055_TEMP_UNIT__LEN             1
#define BNO055_TEMP_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* ORI android-windows register*/
#define BNO055_DATA_OUTPUT_FORMAT__POS             7
#define BNO055_DATA_OUTPUT_FORMAT__MSK             0X80
#define BNO055_DATA_OUTPUT_FORMAT__LEN             1
#define BNO055_DATA_OUTPUT_FORMAT__REG             BNO055_UNIT_SEL_ADDR

/*Data Select register*/
/* Accel data select*/
#define BNO055_DATA_SEL_ACC__POS			0
#define BNO055_DATA_SEL_ACC__MSK			0X01
#define BNO055_DATA_SEL_ACC__LEN			1
#define BNO055_DATA_SEL_ACC__REG			BNO055_DATA_SEL_ADDR

/* Mag data select*/
#define BNO055_DATA_SEL_MAG__POS			1
#define BNO055_DATA_SEL_MAG__MSK			0X02
#define BNO055_DATA_SEL_MAG__LEN			1
#define BNO055_DATA_SEL_MAG__REG			BNO055_DATA_SEL_ADDR

/* Gyro data select*/
#define BNO055_DATA_SEL_GYR__POS			2
#define BNO055_DATA_SEL_GYR__MSK			0X04
#define BNO055_DATA_SEL_GYR__LEN			1
#define BNO055_DATA_SEL_GYR__REG			BNO055_DATA_SEL_ADDR

/* Euler data select*/
#define BNO055_DATA_SEL_EUL__POS			3
#define BNO055_DATA_SEL_EUL__MSK			0X08
#define BNO055_DATA_SEL_EUL__LEN			1
#define BNO055_DATA_SEL_EUL__REG			BNO055_DATA_SEL_ADDR

/* Quaternion data select*/
#define BNO055_DATA_SEL_QUA__POS			4
#define BNO055_DATA_SEL_QUA__MSK			0X10
#define BNO055_DATA_SEL_QUA__LEN			1
#define BNO055_DATA_SEL_QUA__REG			BNO055_DATA_SEL_ADDR

/* Linear Accel data select*/
#define BNO055_DATA_SEL_LINEAR_ACC__POS			5
#define BNO055_DATA_SEL_LINEAR_ACC__MSK			0X20
#define BNO055_DATA_SEL_LINEAR_ACC__LEN			1
#define BNO055_DATA_SEL_LINEAR_ACC__REG			BNO055_DATA_SEL_ADDR

/* Gravity data select*/
#define BNO055_DATA_SEL_GRV__POS			6
#define BNO055_DATA_SEL_GRV__MSK			0X40
#define BNO055_DATA_SEL_GRV__LEN			1
#define BNO055_DATA_SEL_GRV__REG			BNO055_DATA_SEL_ADDR

/* Temperature data select*/
#define BNO055_DATA_SEL_TEMP__POS			7
#define BNO055_DATA_SEL_TEMP__MSK			0X80
#define BNO055_DATA_SEL_TEMP__LEN			1
#define BNO055_DATA_SEL_TEMP__REG			BNO055_DATA_SEL_ADDR

/*Operation Mode data register*/
#define BNO055_OPERATION_MODE__POS			0
#define BNO055_OPERATION_MODE__MSK			0X0F
#define BNO055_OPERATION_MODE__LEN			4
#define BNO055_OPERATION_MODE__REG			BNO055_OPR_MODE_ADDR

/* output data rate register*/
#define BNO055_OUTPUT_DATA_RATE__POS		4
#define BNO055_OUTPUT_DATA_RATE__MSK		0X70
#define BNO055_OUTPUT_DATA_RATE__LEN		3
#define BNO055_OUTPUT_DATA_RATE__REG		BNO055_OPR_MODE_ADDR

/* Power Mode register*/
#define BNO055_POWER_MODE__POS             0
#define BNO055_POWER_MODE__MSK             0X03
#define BNO055_POWER_MODE__LEN             2
#define BNO055_POWER_MODE__REG             BNO055_PWR_MODE_ADDR

/*Self Test register*/
#define BNO055_SELF_TEST__POS			0
#define BNO055_SELF_TEST__MSK			0X01
#define BNO055_SELF_TEST__LEN			1
#define BNO055_SELF_TEST__REG			BNO055_SYS_TRIGGER_ADDR

/* RST_SYS register*/
#define BNO055_RST_SYS__POS             5
#define BNO055_RST_SYS__MSK             0X20
#define BNO055_RST_SYS__LEN             1
#define BNO055_RST_SYS__REG             BNO055_SYS_TRIGGER_ADDR

/* RST_INT register*/
#define BNO055_RST_INT__POS             6
#define BNO055_RST_INT__MSK             0X40
#define BNO055_RST_INT__LEN             1
#define BNO055_RST_INT__REG             BNO055_SYS_TRIGGER_ADDR

/* Temp source register*/
#define BNO055_TEMP_SOURCE__POS		0
#define BNO055_TEMP_SOURCE__MSK		0X03
#define BNO055_TEMP_SOURCE__LEN		2
#define BNO055_TEMP_SOURCE__REG		BNO055_TEMP_SOURCE_ADDR

/* Axis remap value register*/
#define BNO055_REMAP_AXIS_VALUE__POS		0
#define BNO055_REMAP_AXIS_VALUE__MSK		0X3F
#define BNO055_REMAP_AXIS_VALUE__LEN		6
#define BNO055_REMAP_AXIS_VALUE__REG		BNO055_AXIS_MAP_CONFIG_ADDR

/* Axis sign value register*/
#define BNO055_REMAP_Z_SIGN__POS		0
#define BNO055_REMAP_Z_SIGN__MSK		0X01
#define BNO055_REMAP_Z_SIGN__LEN		1
#define BNO055_REMAP_Z_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_Y_SIGN__POS		1
#define BNO055_REMAP_Y_SIGN__MSK		0X02
#define BNO055_REMAP_Y_SIGN__LEN		1
#define BNO055_REMAP_Y_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_X_SIGN__POS		2
#define BNO055_REMAP_X_SIGN__MSK		0X04
#define BNO055_REMAP_X_SIGN__LEN		1
#define BNO055_REMAP_X_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

/* Soft Iron Calibration matrix register*/
#define BNO055_SIC_MATRIX_0_LSB__POS		0
#define BNO055_SIC_MATRIX_0_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_0_LSB__LEN		8
#define BNO055_SIC_MATRIX_0_LSB__REG		BNO055_SIC_MATRIX_0_LSB_ADDR

#define BNO055_SIC_MATRIX_0_MSB__POS		0
#define BNO055_SIC_MATRIX_0_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_0_MSB__LEN		8
#define BNO055_SIC_MATRIX_0_MSB__REG		BNO055_SIC_MATRIX_0_MSB_ADDR

#define BNO055_SIC_MATRIX_1_LSB__POS		0
#define BNO055_SIC_MATRIX_1_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_1_LSB__LEN		8
#define BNO055_SIC_MATRIX_1_LSB__REG		BNO055_SIC_MATRIX_1_LSB_ADDR

#define BNO055_SIC_MATRIX_1_MSB__POS		0
#define BNO055_SIC_MATRIX_1_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_1_MSB__LEN		8
#define BNO055_SIC_MATRIX_1_MSB__REG		BNO055_SIC_MATRIX_1_MSB_ADDR

#define BNO055_SIC_MATRIX_2_LSB__POS		0
#define BNO055_SIC_MATRIX_2_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_2_LSB__LEN		8
#define BNO055_SIC_MATRIX_2_LSB__REG		BNO055_SIC_MATRIX_2_LSB_ADDR

#define BNO055_SIC_MATRIX_2_MSB__POS		0
#define BNO055_SIC_MATRIX_2_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_2_MSB__LEN		8
#define BNO055_SIC_MATRIX_2_MSB__REG		BNO055_SIC_MATRIX_2_MSB_ADDR

#define BNO055_SIC_MATRIX_3_LSB__POS		0
#define BNO055_SIC_MATRIX_3_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_3_LSB__LEN		8
#define BNO055_SIC_MATRIX_3_LSB__REG		BNO055_SIC_MATRIX_3_LSB_ADDR

#define BNO055_SIC_MATRIX_3_MSB__POS		0
#define BNO055_SIC_MATRIX_3_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_3_MSB__LEN		8
#define BNO055_SIC_MATRIX_3_MSB__REG		BNO055_SIC_MATRIX_3_MSB_ADDR

#define BNO055_SIC_MATRIX_4_LSB__POS		0
#define BNO055_SIC_MATRIX_4_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_4_LSB__LEN		8
#define BNO055_SIC_MATRIX_4_LSB__REG		BNO055_SIC_MATRIX_4_LSB_ADDR

#define BNO055_SIC_MATRIX_4_MSB__POS		0
#define BNO055_SIC_MATRIX_4_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_4_MSB__LEN		8
#define BNO055_SIC_MATRIX_4_MSB__REG		BNO055_SIC_MATRIX_4_MSB_ADDR

#define BNO055_SIC_MATRIX_5_LSB__POS		0
#define BNO055_SIC_MATRIX_5_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_5_LSB__LEN		8
#define BNO055_SIC_MATRIX_5_LSB__REG		BNO055_SIC_MATRIX_5_LSB_ADDR

#define BNO055_SIC_MATRIX_5_MSB__POS		0
#define BNO055_SIC_MATRIX_5_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_5_MSB__LEN		8
#define BNO055_SIC_MATRIX_5_MSB__REG		BNO055_SIC_MATRIX_5_MSB_ADDR

#define BNO055_SIC_MATRIX_6_LSB__POS		0
#define BNO055_SIC_MATRIX_6_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_6_LSB__LEN		8
#define BNO055_SIC_MATRIX_6_LSB__REG		BNO055_SIC_MATRIX_6_LSB_ADDR

#define BNO055_SIC_MATRIX_6_MSB__POS		0
#define BNO055_SIC_MATRIX_6_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_6_MSB__LEN		8
#define BNO055_SIC_MATRIX_6_MSB__REG		BNO055_SIC_MATRIX_6_MSB_ADDR

#define BNO055_SIC_MATRIX_7_LSB__POS		0
#define BNO055_SIC_MATRIX_7_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_7_LSB__LEN		8
#define BNO055_SIC_MATRIX_7_LSB__REG		BNO055_SIC_MATRIX_7_LSB_ADDR

#define BNO055_SIC_MATRIX_7_MSB__POS		0
#define BNO055_SIC_MATRIX_7_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_7_MSB__LEN		8
#define BNO055_SIC_MATRIX_7_MSB__REG		BNO055_SIC_MATRIX_7_MSB_ADDR

#define BNO055_SIC_MATRIX_8_LSB__POS		0
#define BNO055_SIC_MATRIX_8_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_8_LSB__LEN		8
#define BNO055_SIC_MATRIX_8_LSB__REG		BNO055_SIC_MATRIX_8_LSB_ADDR

#define BNO055_SIC_MATRIX_8_MSB__POS		0
#define BNO055_SIC_MATRIX_8_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_8_MSB__LEN		8
#define BNO055_SIC_MATRIX_8_MSB__REG		BNO055_SIC_MATRIX_8_MSB_ADDR

/*Accel Offset registers*/
#define BNO055_ACC_OFFSET_X_LSB__POS		0
#define BNO055_ACC_OFFSET_X_LSB__MSK		0XFF
#define BNO055_ACC_OFFSET_X_LSB__LEN		8
#define BNO055_ACC_OFFSET_X_LSB__REG		ACC_OFFSET_X_LSB_ADDR

#define BNO055_ACC_OFFSET_X_MSB__POS		0
#define BNO055_ACC_OFFSET_X_MSB__MSK		0XFF
#define BNO055_ACC_OFFSET_X_MSB__LEN		8
#define BNO055_ACC_OFFSET_X_MSB__REG		ACC_OFFSET_X_MSB_ADDR

#define BNO055_ACC_OFFSET_Y_LSB__POS		0
#define BNO055_ACC_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_ACC_OFFSET_Y_LSB__LEN		8
#define BNO055_ACC_OFFSET_Y_LSB__REG		ACC_OFFSET_Y_LSB_ADDR

#define BNO055_ACC_OFFSET_Y_MSB__POS		0
#define BNO055_ACC_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_ACC_OFFSET_Y_MSB__LEN		8
#define BNO055_ACC_OFFSET_Y_MSB__REG		ACC_OFFSET_Y_MSB_ADDR

#define BNO055_ACC_OFFSET_Z_LSB__POS		0
#define BNO055_ACC_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_ACC_OFFSET_Z_LSB__LEN		8
#define BNO055_ACC_OFFSET_Z_LSB__REG		ACC_OFFSET_Z_LSB_ADDR

#define BNO055_ACC_OFFSET_Z_MSB__POS		0
#define BNO055_ACC_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_ACC_OFFSET_Z_MSB__LEN		8
#define BNO055_ACC_OFFSET_Z_MSB__REG		ACC_OFFSET_Z_MSB_ADDR

/*Mag Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB__POS		0
#define BNO055_MAG_OFFSET_X_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_LSB__LEN		8
#define BNO055_MAG_OFFSET_X_LSB__REG		MAG_OFFSET_X_LSB_ADDR

#define BNO055_MAG_OFFSET_X_MSB__POS		0
#define BNO055_MAG_OFFSET_X_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_MSB__LEN		8
#define BNO055_MAG_OFFSET_X_MSB__REG		MAG_OFFSET_X_MSB_ADDR

#define BNO055_MAG_OFFSET_Y_LSB__POS		0
#define BNO055_MAG_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_LSB__LEN		8
#define BNO055_MAG_OFFSET_Y_LSB__REG		MAG_OFFSET_Y_LSB_ADDR

#define BNO055_MAG_OFFSET_Y_MSB__POS		0
#define BNO055_MAG_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_MSB__LEN		8
#define BNO055_MAG_OFFSET_Y_MSB__REG		MAG_OFFSET_Y_MSB_ADDR

#define BNO055_MAG_OFFSET_Z_LSB__POS		0
#define BNO055_MAG_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_LSB__LEN		8
#define BNO055_MAG_OFFSET_Z_LSB__REG		MAG_OFFSET_Z_LSB_ADDR

#define BNO055_MAG_OFFSET_Z_MSB__POS		0
#define BNO055_MAG_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_MSB__LEN		8
#define BNO055_MAG_OFFSET_Z_MSB__REG		MAG_OFFSET_Z_MSB_ADDR

/* Gyro Offset registers*/
#define BNO055_GYR_OFFSET_X_LSB__POS		0
#define BNO055_GYR_OFFSET_X_LSB__MSK		0XFF
#define BNO055_GYR_OFFSET_X_LSB__LEN		8
#define BNO055_GYR_OFFSET_X_LSB__REG		GYRO_OFFSET_X_LSB_ADDR

#define BNO055_GYR_OFFSET_X_MSB__POS		0
#define BNO055_GYR_OFFSET_X_MSB__MSK		0XFF
#define BNO055_GYR_OFFSET_X_MSB__LEN		8
#define BNO055_GYR_OFFSET_X_MSB__REG		GYRO_OFFSET_X_MSB_ADDR

#define BNO055_GYR_OFFSET_Y_LSB__POS		0
#define BNO055_GYR_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_GYR_OFFSET_Y_LSB__LEN		8
#define BNO055_GYR_OFFSET_Y_LSB__REG		GYRO_OFFSET_Y_LSB_ADDR

#define BNO055_GYR_OFFSET_Y_MSB__POS		0
#define BNO055_GYR_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_GYR_OFFSET_Y_MSB__LEN		8
#define BNO055_GYR_OFFSET_Y_MSB__REG		GYRO_OFFSET_Y_MSB_ADDR

#define BNO055_GYR_OFFSET_Z_LSB__POS		0
#define BNO055_GYR_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_GYR_OFFSET_Z_LSB__LEN		8
#define BNO055_GYR_OFFSET_Z_LSB__REG		GYRO_OFFSET_Z_LSB_ADDR

#define BNO055_GYR_OFFSET_Z_MSB__POS		0
#define BNO055_GYR_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_GYR_OFFSET_Z_MSB__LEN		8
#define BNO055_GYR_OFFSET_Z_MSB__REG		GYRO_OFFSET_Z_MSB_ADDR
/* PAGE0 DATA REGISTERS DEFINITION END*/

/* PAGE1 DATA REGISTERS DEFINITION START*/
/* Configuration registers*/
/* Accel range configuration register*/
#define BNO055_CONFIG_ACC_RANGE__POS		0
#define BNO055_CONFIG_ACC_RANGE__MSK		0X03
#define BNO055_CONFIG_ACC_RANGE__LEN		2
#define BNO055_CONFIG_ACC_RANGE__REG		ACC_CONFIG_ADDR

/* Accel bandwidth configuration register*/
#define BNO055_CONFIG_ACC_BW__POS			2
#define BNO055_CONFIG_ACC_BW__MSK			0X1C
#define BNO055_CONFIG_ACC_BW__LEN			3
#define BNO055_CONFIG_ACC_BW__REG			ACC_CONFIG_ADDR

/* Accel power mode configuration register*/
#define BNO055_CONFIG_ACC_PWR_MODE__POS		5
#define BNO055_CONFIG_ACC_PWR_MODE__MSK		0XE0
#define BNO055_CONFIG_ACC_PWR_MODE__LEN		3
#define BNO055_CONFIG_ACC_PWR_MODE__REG		ACC_CONFIG_ADDR

/* Mag data output rate configuration register*/
#define BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__POS		0
#define BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__MSK		0X07
#define BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__LEN		3
#define BNO055_CONFIG_MAG_DATA_OUTPUT_RATE__REG		MAG_CONFIG_ADDR

/* Mag operation mode configuration register*/
#define BNO055_CONFIG_MAG_OPR_MODE__POS		3
#define BNO055_CONFIG_MAG_OPR_MODE__MSK		0X18
#define BNO055_CONFIG_MAG_OPR_MODE__LEN		2
#define BNO055_CONFIG_MAG_OPR_MODE__REG		MAG_CONFIG_ADDR

/* Mag power mode configuration register*/
#define BNO055_CONFIG_MAG_POWER_MODE__POS		5
#define BNO055_CONFIG_MAG_POWER_MODE__MSK		0X60
#define BNO055_CONFIG_MAG_POWER_MODE__LEN		2
#define BNO055_CONFIG_MAG_POWER_MODE__REG		MAG_CONFIG_ADDR

/* Gyro range configuration register*/
#define BNO055_CONFIG_GYR_RANGE__POS		0
#define BNO055_CONFIG_GYR_RANGE__MSK		0X07
#define BNO055_CONFIG_GYR_RANGE__LEN		3
#define BNO055_CONFIG_GYR_RANGE__REG		GYRO_CONFIG_ADDR

/* Gyro bandwidth configuration register*/
#define BNO055_CONFIG_GYR_BANDWIDTH__POS		3
#define BNO055_CONFIG_GYR_BANDWIDTH__MSK		0X38
#define BNO055_CONFIG_GYR_BANDWIDTH__LEN		3
#define BNO055_CONFIG_GYR_BANDWIDTH__REG		GYRO_CONFIG_ADDR

/* Gyro power mode configuration register*/
#define BNO055_CONFIG_GYR_POWER_MODE__POS		0
#define BNO055_CONFIG_GYR_POWER_MODE__MSK		0X07
#define BNO055_CONFIG_GYR_POWER_MODE__LEN		3
#define BNO055_CONFIG_GYR_POWER_MODE__REG		GYRO_MODE_CONFIG_ADDR

/* Sleep configuration registers*/
/* Accel sleep mode configuration register*/
#define BNO055_ACC_SLEEP_MODE__POS		0
#define BNO055_ACC_SLEEP_MODE__MSK		0X01
#define BNO055_ACC_SLEEP_MODE__LEN		1
#define BNO055_ACC_SLEEP_MODE__REG		ACC_SLEEP_CONFIG_ADDR

/* Accel sleep duration configuration register*/
#define BNO055_ACC_SLEEP_DUR__POS		1
#define BNO055_ACC_SLEEP_DUR__MSK		0X1E
#define BNO055_ACC_SLEEP_DUR__LEN		4
#define BNO055_ACC_SLEEP_DUR__REG		ACC_SLEEP_CONFIG_ADDR

/* Gyro sleep duration configuration register*/
#define BNO055_GYR_SLEEP_DUR__POS		0
#define BNO055_GYR_SLEEP_DUR__MSK		0X07
#define BNO055_GYR_SLEEP_DUR__LEN		3
#define BNO055_GYR_SLEEP_DUR__REG		GYR_SLEEP_CONFIG_ADDR

/* Gyro auto sleep duration configuration register*/
#define BNO055_GYR_AUTO_SLEEP_DUR__POS		3
#define BNO055_GYR_AUTO_SLEEP_DUR__MSK		0X38
#define BNO055_GYR_AUTO_SLEEP_DUR__LEN		3
#define BNO055_GYR_AUTO_SLEEP_DUR__REG		GYR_SLEEP_CONFIG_ADDR

/* Mag sleep mode configuration register*/
#define BNO055_MAG_SLEEP_MODE__POS		0
#define BNO055_MAG_SLEEP_MODE__MSK		0X01
#define BNO055_MAG_SLEEP_MODE__LEN		1
#define BNO055_MAG_SLEEP_MODE__REG		MAG_SLEEP_CONFIG_ADDR

/* Mag sleep duration configuration register*/
#define BNO055_MAG_SLEEP_DUR__POS		1
#define BNO055_MAG_SLEEP_DUR__MSK		0X1E
#define BNO055_MAG_SLEEP_DUR__LEN		4
#define BNO055_MAG_SLEEP_DUR__REG		MAG_SLEEP_CONFIG_ADDR

/* Interrupt registers*/
/* Gyro any motion interrupt msk register*/
#define BNO055_GYR_AM_INTMSK__POS		2
#define BNO055_GYR_AM_INTMSK__MSK		0X04
#define BNO055_GYR_AM_INTMSK__LEN		1
#define BNO055_GYR_AM_INTMSK__REG		INT_MSK_ADDR

/* Gyro high rate interrupt msk register*/
#define BNO055_GYR_HIGH_RATE_INTMSK__POS		3
#define BNO055_GYR_HIGH_RATE_INTMSK__MSK		0X08
#define BNO055_GYR_HIGH_RATE_INTMSK__LEN		1
#define BNO055_GYR_HIGH_RATE_INTMSK__REG		INT_MSK_ADDR

/* Accel high g interrupt msk register*/
#define BNO055_ACC_HIGH_G_INTMSK__POS		5
#define BNO055_ACC_HIGH_G_INTMSK__MSK		0X20
#define BNO055_ACC_HIGH_G_INTMSK__LEN		1
#define BNO055_ACC_HIGH_G_INTMSK__REG		INT_MSK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACC_AM_INTMSK__POS		6
#define BNO055_ACC_AM_INTMSK__MSK		0X40
#define BNO055_ACC_AM_INTMSK__LEN		1
#define BNO055_ACC_AM_INTMSK__REG		INT_MSK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACC_NM_INTMSK__POS		7
#define BNO055_ACC_NM_INTMSK__MSK		0X80
#define BNO055_ACC_NM_INTMSK__LEN		1
#define BNO055_ACC_NM_INTMSK__REG		INT_MSK_ADDR

/* Gyro any motion interrupt register*/
#define BNO055_GYR_AM_INT__POS		2
#define BNO055_GYR_AM_INT__MSK		0X04
#define BNO055_GYR_AM_INT__LEN		1
#define BNO055_GYR_AM_INT__REG		INT_ADDR

/* Gyro high rate interrupt register*/
#define BNO055_GYR_HIGH_RATE_INT__POS		3
#define BNO055_GYR_HIGH_RATE_INT__MSK		0X08
#define BNO055_GYR_HIGH_RATE_INT__LEN		1
#define BNO055_GYR_HIGH_RATE_INT__REG		INT_ADDR

/* Accel high g interrupt register*/
#define BNO055_ACC_HIGH_G_INT__POS		5
#define BNO055_ACC_HIGH_G_INT__MSK		0X20
#define BNO055_ACC_HIGH_G_INT__LEN		1
#define BNO055_ACC_HIGH_G_INT__REG		INT_ADDR

/* Accel any motion interrupt register*/
#define BNO055_ACC_AM_INT__POS		6
#define BNO055_ACC_AM_INT__MSK		0X40
#define BNO055_ACC_AM_INT__LEN		1
#define BNO055_ACC_AM_INT__REG		INT_ADDR

/*Accel any motion interrupt register*/
#define BNO055_ACC_NM_INT__POS		7
#define BNO055_ACC_NM_INT__MSK		0X80
#define BNO055_ACC_NM_INT__LEN		1
#define BNO055_ACC_NM_INT__REG		INT_ADDR

/*Accel any motion threshold setting*/
#define BNO055_ACC_AM_THRES__POS	0
#define BNO055_ACC_AM_THRES__MSK	0XFF
#define BNO055_ACC_AM_THRES__LEN	8
#define BNO055_ACC_AM_THRES__REG	ACC_AM_THRES_ADDR

/*Accel interrupt setting register*/
#define BNO055_ACC_AM_DUR_SET__POS	0
#define BNO055_ACC_AM_DUR_SET__MSK	0X03
#define BNO055_ACC_AM_DUR_SET__LEN	2
#define BNO055_ACC_AM_DUR_SET__REG	ACC_INT_SETTINGS_ADDR

/* Accel AM/NM axis selection register*/
#define BNO055_ACC_AN_MOTION_X_AXIS__POS		2
#define BNO055_ACC_AN_MOTION_X_AXIS__MSK		0X04
#define BNO055_ACC_AN_MOTION_X_AXIS__LEN		1
#define BNO055_ACC_AN_MOTION_X_AXIS__REG		ACC_INT_SETTINGS_ADDR

#define BNO055_ACC_AN_MOTION_Y_AXIS__POS		3
#define BNO055_ACC_AN_MOTION_Y_AXIS__MSK		0X08
#define BNO055_ACC_AN_MOTION_Y_AXIS__LEN		1
#define BNO055_ACC_AN_MOTION_Y_AXIS__REG		ACC_INT_SETTINGS_ADDR

#define BNO055_ACC_AN_MOTION_Z_AXIS__POS		4
#define BNO055_ACC_AN_MOTION_Z_AXIS__MSK		0X10
#define BNO055_ACC_AN_MOTION_Z_AXIS__LEN		1
#define BNO055_ACC_AN_MOTION_Z_AXIS__REG		ACC_INT_SETTINGS_ADDR

/* Accel high g axis selection register*/
#define BNO055_ACC_HIGH_G_X_AXIS__POS		5
#define BNO055_ACC_HIGH_G_X_AXIS__MSK		0X20
#define BNO055_ACC_HIGH_G_X_AXIS__LEN		1
#define BNO055_ACC_HIGH_G_X_AXIS__REG		ACC_INT_SETTINGS_ADDR

#define BNO055_ACC_HIGH_G_Y_AXIS__POS		6
#define BNO055_ACC_HIGH_G_Y_AXIS__MSK		0X40
#define BNO055_ACC_HIGH_G_Y_AXIS__LEN		1
#define BNO055_ACC_HIGH_G_Y_AXIS__REG		ACC_INT_SETTINGS_ADDR

#define BNO055_ACC_HIGH_G_Z_AXIS__POS		7
#define BNO055_ACC_HIGH_G_Z_AXIS__MSK		0X80
#define BNO055_ACC_HIGH_G_Z_AXIS__LEN		1
#define BNO055_ACC_HIGH_G_Z_AXIS__REG		ACC_INT_SETTINGS_ADDR

/* Accel High g duration setting register*/
#define BNO055_ACC_HIGH_G_DURATION__POS		0
#define BNO055_ACC_HIGH_G_DURATION__MSK		0XFF
#define BNO055_ACC_HIGH_G_DURATION__LEN		8
#define BNO055_ACC_HIGH_G_DURATION__REG		ACC_HG_DURATION_ADDR

/* Accel High g threshold setting register*/
#define BNO055_ACC_HIGH_G_THRESHOLD__POS		0
#define BNO055_ACC_HIGH_G_THRESHOLD__MSK		0XFF
#define BNO055_ACC_HIGH_G_THRESHOLD__LEN		8
#define BNO055_ACC_HIGH_G_THRESHOLD__REG		ACC_HG_THRES_ADDR

/* Accel no/slow motion threshold setting*/
#define BNO055_ACC_NS_THRESHOLD__POS		0
#define BNO055_ACC_NS_THRESHOLD__MSK		0XFF
#define BNO055_ACC_NS_THRESHOLD__LEN		8
#define BNO055_ACC_NS_THRESHOLD__REG		ACC_NM_THRES_ADDR

/* Accel no/slow motion enable setting*/
#define BNO055_ACC_NM_SM_ENABLE__POS		0
#define BNO055_ACC_NM_SM_ENABLE__MSK		0X01
#define BNO055_ACC_NM_SM_ENABLE__LEN		1
#define BNO055_ACC_NM_SM_ENABLE__REG		ACC_NM_SET_ADDR

/* Accel no/slow motion duration setting*/
#define BNO055_ACC_NS_DURATION__POS		1
#define BNO055_ACC_NS_DURATION__MSK		0X7E
#define BNO055_ACC_NS_DURATION__LEN		6
#define BNO055_ACC_NS_DURATION__REG		ACC_NM_SET_ADDR

/*Gyro interrupt setting register*/
/*Gyro any motion axis setting*/
#define BNO055_GYR_AM_X_AXIS__POS		0
#define BNO055_GYR_AM_X_AXIS__MSK		0X01
#define BNO055_GYR_AM_X_AXIS__LEN		1
#define BNO055_GYR_AM_X_AXIS__REG		GYR_INT_SETING_ADDR

#define BNO055_GYR_AM_Y_AXIS__POS		1
#define BNO055_GYR_AM_Y_AXIS__MSK		0X02
#define BNO055_GYR_AM_Y_AXIS__LEN		1
#define BNO055_GYR_AM_Y_AXIS__REG		GYR_INT_SETING_ADDR

#define BNO055_GYR_AM_Z_AXIS__POS		2
#define BNO055_GYR_AM_Z_AXIS__MSK		0X04
#define BNO055_GYR_AM_Z_AXIS__LEN		1
#define BNO055_GYR_AM_Z_AXIS__REG		GYR_INT_SETING_ADDR

/*Gyro high rate axis setting*/
#define BNO055_GYR_HR_X_AXIS__POS		3
#define BNO055_GYR_HR_X_AXIS__MSK		0X08
#define BNO055_GYR_HR_X_AXIS__LEN		1
#define BNO055_GYR_HR_X_AXIS__REG		GYR_INT_SETING_ADDR

#define BNO055_GYR_HR_Y_AXIS__POS		4
#define BNO055_GYR_HR_Y_AXIS__MSK		0X10
#define BNO055_GYR_HR_Y_AXIS__LEN		1
#define BNO055_GYR_HR_Y_AXIS__REG		GYR_INT_SETING_ADDR

#define BNO055_GYR_HR_Z_AXIS__POS		5
#define BNO055_GYR_HR_Z_AXIS__MSK		0X20
#define BNO055_GYR_HR_Z_AXIS__LEN		1
#define BNO055_GYR_HR_Z_AXIS__REG		GYR_INT_SETING_ADDR

/* Gyro filter setting*/
#define BNO055_GYR_AM_FILT__POS		6
#define BNO055_GYR_AM_FILT__MSK		0X40
#define BNO055_GYR_AM_FILT__LEN		1
#define BNO055_GYR_AM_FILT__REG		GYR_INT_SETING_ADDR

#define BNO055_GYR_HR_FILT__POS		7
#define BNO055_GYR_HR_FILT__MSK		0X80
#define BNO055_GYR_HR_FILT__LEN		1
#define BNO055_GYR_HR_FILT__REG		GYR_INT_SETING_ADDR

/* Gyro high rate X axis settings*/
#define BNO055_GYR_HR_X_THRESH__POS		0
#define BNO055_GYR_HR_X_THRESH__MSK		0X1F
#define BNO055_GYR_HR_X_THRESH__LEN		5
#define BNO055_GYR_HR_X_THRESH__REG		GYR_HR_X_SET_ADDR

#define BNO055_GYR_HR_X_HYST__POS		5
#define BNO055_GYR_HR_X_HYST__MSK		0X60
#define BNO055_GYR_HR_X_HYST__LEN		2
#define BNO055_GYR_HR_X_HYST__REG		GYR_HR_X_SET_ADDR

#define BNO055_GYR_HR_X_DUR__POS		0
#define BNO055_GYR_HR_X_DUR__MSK		0XFF
#define BNO055_GYR_HR_X_DUR__LEN		8
#define BNO055_GYR_HR_X_DUR__REG		GYR_DUR_X_ADDR

/* Gyro high rate Y axis settings*/
#define BNO055_GYR_HR_Y_THRESH__POS		0
#define BNO055_GYR_HR_Y_THRESH__MSK		0X1F
#define BNO055_GYR_HR_Y_THRESH__LEN		5
#define BNO055_GYR_HR_Y_THRESH__REG		GYR_HR_Y_SET_ADDR

#define BNO055_GYR_HR_Y_HYST__POS		5
#define BNO055_GYR_HR_Y_HYST__MSK		0X60
#define BNO055_GYR_HR_Y_HYST__LEN		2
#define BNO055_GYR_HR_Y_HYST__REG		GYR_HR_Y_SET_ADDR

#define BNO055_GYR_HR_Y_DUR__POS		0
#define BNO055_GYR_HR_Y_DUR__MSK		0XFF
#define BNO055_GYR_HR_Y_DUR__LEN		8
#define BNO055_GYR_HR_Y_DUR__REG		GYR_DUR_Y_ADDR

/* Gyro high rate Z axis settings*/
#define BNO055_GYR_HR_Z_THRESH__POS		0
#define BNO055_GYR_HR_Z_THRESH__MSK		0X1F
#define BNO055_GYR_HR_Z_THRESH__LEN		5
#define BNO055_GYR_HR_Z_THRESH__REG		GYR_HR_Z_SET_ADDR

#define BNO055_GYR_HR_Z_HYST__POS		5
#define BNO055_GYR_HR_Z_HYST__MSK		0X60
#define BNO055_GYR_HR_Z_HYST__LEN		2
#define BNO055_GYR_HR_Z_HYST__REG		GYR_HR_Z_SET_ADDR

#define BNO055_GYR_HR_Z_DUR__POS		0
#define BNO055_GYR_HR_Z_DUR__MSK		0XFF
#define BNO055_GYR_HR_Z_DUR__LEN		8
#define BNO055_GYR_HR_Z_DUR__REG		GYR_DUR_Z_ADDR

/*Gyro any motion threshold setting*/
#define BNO055_GYR_AM_THRES__POS		0
#define BNO055_GYR_AM_THRES__MSK		0X7F
#define BNO055_GYR_AM_THRES__LEN		7
#define BNO055_GYR_AM_THRES__REG		GYR_AM_THRES_ADDR

/* Gyro any motion slope sample setting*/
#define BNO055_GYR_SLOPE_SAMPLES__POS		0
#define BNO055_GYR_SLOPE_SAMPLES__MSK		0X03
#define BNO055_GYR_SLOPE_SAMPLES__LEN		2
#define BNO055_GYR_SLOPE_SAMPLES__REG		GYR_AM_SET_ADDR

/* Gyro awake duration setting*/
#define BNO055_GYR_AWAKE_DUR__POS		2
#define BNO055_GYR_AWAKE_DUR__MSK		0X0C
#define BNO055_GYR_AWAKE_DUR__LEN		2
#define BNO055_GYR_AWAKE_DUR__REG		GYR_AM_SET_ADDR

/* PAGE1 DATA REGISTERS DEFINITION END*/

#define BNO055_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)


#define BNO055_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* Function Declarations */
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);

BNO055_RETURN_FUNCTION_TYPE bno055_write_register(
unsigned char addr, unsigned char *data, unsigned char len);

BNO055_RETURN_FUNCTION_TYPE bno055_read_register(unsigned char addr,
unsigned char *data, unsigned char len);

BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(unsigned char *chip_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_revision_id(BNO055_U16 *sw_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(unsigned char *pg_id);

BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(unsigned char page_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_revision_id(
unsigned char *acc_rev_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_revision_id(
unsigned char *mag_rev_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_revision_id(
unsigned char *gyr_rev_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_bootloader_revision_id(
unsigned char *bl_rev_id);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(BNO055_S16 *acc_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(BNO055_S16 *acc_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(BNO055_S16 *acc_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(
struct bno055_accel *acc);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(BNO055_S16 *mag_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(BNO055_S16 *mag_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(BNO055_S16 *mag_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag *mag);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(BNO055_S16 *gyr_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(BNO055_S16 *gyr_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(BNO055_S16 *gyr_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro *gyr);

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(BNO055_S16 *eul_h);

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(BNO055_S16 *eul_r);

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(BNO055_S16 *eul_p);

BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(struct bno055_euler *eul);

BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(BNO055_S16 *quan_w);

BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(BNO055_S16 *quan_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(BNO055_S16 *quan_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(BNO055_S16 *quan_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(
struct bno055_quaternion *qur);

BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(BNO055_S16 *lia_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(BNO055_S16 *lia_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(BNO055_S16 *lia_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(
struct bno055_linear_accel *lia);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(BNO055_S16 *grv_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(BNO055_S16 *grv_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(BNO055_S16 *grv_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(
struct bno055_gravity *grvt);

BNO055_RETURN_FUNCTION_TYPE bno055_read_temperature_data(BNO055_S16 *temp);

BNO055_RETURN_FUNCTION_TYPE bno055_get_magcalib_status(
unsigned char *mag_calib);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accelcalib_status(
unsigned char *acc_calib);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyrocalib_status(
unsigned char *gyr_calib);

BNO055_RETURN_FUNCTION_TYPE bno055_get_syscalib_status(
unsigned char *sys_calib);

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_accel(
unsigned char *st_acc);

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_mag(
unsigned char *st_mag);

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_gyro(
unsigned char *st_gyr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_st_mcu(
unsigned char *st_mcu);

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_gyro_anymotion(
unsigned char *gyr_anymotion);

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_gyro_highrate(
unsigned char *gyr_highrate);

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_highg(
unsigned char *acc_highg);

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_anymotion(
unsigned char *acc_anymotion);

BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_status_accel_nomotion(
unsigned char *acc_nomotion);

BNO055_RETURN_FUNCTION_TYPE bno055_get_system_status_code(
unsigned char *sys_status);

BNO055_RETURN_FUNCTION_TYPE bno055_get_system_error_code(
unsigned char *sys_error);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(
unsigned char *acc_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(unsigned char acc_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(unsigned char *gyr_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(unsigned char gyr_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(unsigned char *eul_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(unsigned char eul_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(
unsigned char *tilt_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(unsigned char tilt_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_get_temperature_unit(
unsigned char *temp_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_set_temperature_unit(
unsigned char temp_unit);

BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(
unsigned char *dof);

BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(unsigned char dof);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_data_select(
unsigned char *acc_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_data_select(
unsigned char acc_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_select(
unsigned char *mag_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_select(
unsigned char mag_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_data_select(
unsigned char *gyro_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_data_select(
unsigned char gyro_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_data_select(
unsigned char *eul_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_data_select(
unsigned char eul_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_quaternion_data_select(
unsigned char *qur_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_quaternion_data_select(
unsigned char qur_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_linear_accel_data_select(
unsigned char *lia_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_linear_accel_data_select(
unsigned char lia_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gravity_data_select(
unsigned char *grv_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gravity_data_select(
unsigned char grv_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_temperature_data_select(
unsigned char *temp_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_set_temperature_data_select(
unsigned char temp_datasel);

BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_rate(
unsigned char *data_out_rate);

BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_rate(
unsigned char data_out_rate);

BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(
unsigned char *op_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(unsigned char opr_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_get_powermode(
unsigned char *pwm);

BNO055_RETURN_FUNCTION_TYPE bno055_set_powermode(
unsigned char powermode);

BNO055_RETURN_FUNCTION_TYPE bno055_get_reset_int(
unsigned char *rst_int);

BNO055_RETURN_FUNCTION_TYPE bno055_set_reset_int(unsigned char rst_int);

BNO055_RETURN_FUNCTION_TYPE bno055_get_reset_sys(
unsigned char *rst_sys);

BNO055_RETURN_FUNCTION_TYPE bno055_set_reset_sys(unsigned char rst_sys);

BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(
unsigned char *self_test);

BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(unsigned char self_test);

BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(
unsigned char *temp_sour);

BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(unsigned char temp_sour);

BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(
unsigned char *remap_axis);

BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(
unsigned char remap_axis);

BNO055_RETURN_FUNCTION_TYPE bno055_get_x_remap_sign(
unsigned char *remap_sign_x);

BNO055_RETURN_FUNCTION_TYPE bno055_set_x_remap_sign(
unsigned char remap_sign_x);

BNO055_RETURN_FUNCTION_TYPE bno055_get_y_remap_sign(
unsigned char *remap_sign_y);

BNO055_RETURN_FUNCTION_TYPE bno055_set_y_remap_sign(
unsigned char remap_sign_y);

BNO055_RETURN_FUNCTION_TYPE bno055_get_z_remap_sign(
unsigned char *remap_sign_z);

BNO055_RETURN_FUNCTION_TYPE bno055_set_z_remap_sign(
unsigned char remap_sign_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_zero(
BNO055_S16 *sic_matrix_zero);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_zero(
BNO055_S16 sic_matrix_zero);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_one(
BNO055_S16 *sic_matrix_one);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_one(
BNO055_S16 sic_matrix_one);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_two(
BNO055_S16 *sic_matrix_two);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_two(
BNO055_S16 sic_matrix_two);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_three(
BNO055_S16 *sic_matrix_three);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_three(
BNO055_S16 sic_matrix_three);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_four(
BNO055_S16 *sic_matrix_four);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_four(
BNO055_S16 sic_matrix_four);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_five(
BNO055_S16 *sic_matrix_five);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_five(
BNO055_S16 sic_matrix_five);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_six(
BNO055_S16 *sic_matrix_six);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_six(
BNO055_S16 sic_matrix_six);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_seven(
BNO055_S16 *sic_matrix_seven);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_seven(
BNO055_S16 sic_matrix_seven);

BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix_eight(
BNO055_S16 *sic_matrix_eight);

BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix_eight(
BNO055_S16 sic_matrix_eight);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_x_axis(
BNO055_S16 *acc_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_x_axis(
BNO055_S16 acc_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_y_axis(
BNO055_S16 *acc_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_y_axis(
BNO055_S16 acc_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset_z_axis(
BNO055_S16 *acc_off_z);

BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset_z_axis(
BNO055_S16 acc_off_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_x_axis(
BNO055_S16 *mag_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_x_axis(
BNO055_S16 mag_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_y_axis(
BNO055_S16 *mag_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_y_axis(
BNO055_S16 mag_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset_z_axis(
BNO055_S16 *mag_off_z);

BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset_z_axis(
BNO055_S16 mag_off_z);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_x_axis(
BNO055_S16 *gyr_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_x_axis(
BNO055_S16 gyr_off_x);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_y_axis(
BNO055_S16 *gyr_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_y_axis(
BNO055_S16 gyr_off_y);

BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset_z_axis(
BNO055_S16 *gyr_off_z);

BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset_z_axis(
BNO055_S16 gyr_off_z);

/* Page1 declarations*/
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(
unsigned char *accel_range);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(
unsigned char accel_range);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bandwidth(
unsigned char *accel_bw);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bandwidth(
unsigned char accel_bw);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_powermode(
unsigned char *accel_pw);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_powermode(
unsigned char accel_pm);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_outputrate(
unsigned char *mag_data_outrate);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_outrate(
unsigned char mag_data_outrate);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(
unsigned char *mag_operation_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(
unsigned char mag_operation_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_powermode(
unsigned char *mag_pw);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_powermode(
unsigned char mag_pw);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(
unsigned char *gyro_range);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(
unsigned char gyro_range);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bandwidth(
unsigned char *gyro_bw);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bandwidth(
unsigned char gyro_bw);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_operation_mode(
unsigned char *gyro_operation_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_operation_mode(
unsigned char gyro_operation_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleeptmr_mode(
unsigned char *sleep_tmr);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleeptmr_mode(
unsigned char sleep_tmr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_dur(
unsigned char *sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_dur(
unsigned char sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleepdur(
unsigned char *sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleepdur(
unsigned char sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_autosleepdur(
unsigned char *auto_duration);

BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_autosleepdur(
unsigned char auto_duration, unsigned char bandwith);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(
unsigned char *sleep_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(
unsigned char sleep_mode);

BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_duration(
unsigned char *sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_duration(
unsigned char sleep_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_gyro_anymotion(
unsigned char *gyro_am);

BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_gyro_anymotion(
unsigned char gyro_am);

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_gyro_highrate(
unsigned char *gyro_hr);

BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_gyro_highrate(
unsigned char gyro_hr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_high_g(
unsigned char *accel_hg);

BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_high_g(
unsigned char accel_hg);

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_anymotion(
unsigned char *accel_am);

BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_anymotion(
unsigned char accel_am);

BNO055_RETURN_FUNCTION_TYPE bno055_get_intmsk_accel_nomotion(
unsigned char *accel_nm);

BNO055_RETURN_FUNCTION_TYPE bno055_set_intmsk_accel_nomotion(
unsigned char accel_nm);

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_anymotion(
unsigned char *gyro_am);

BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_anymotion(
unsigned char gyro_am);

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_highrate(
unsigned char *gyro_hr);

BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_highrate(
unsigned char gyro_hr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_high_g(
unsigned char *accel_hg);

BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_high_g(
unsigned char accel_hg);

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_anymotion(
unsigned char *accel_am);

BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_anymotion(
unsigned char accel_am);

BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_nomotion(
unsigned char *accel_nm);

BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_nomotion(
unsigned char accel_nm);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_threshold(
unsigned char *accel_am_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_threshold(
unsigned char accel_am_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_duration(
unsigned char *accel_am_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_duration(
unsigned char accel_am_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_an_nm_axis_enable(
unsigned char channel, unsigned char *data);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_an_nm_axis_enable(
unsigned char channel, unsigned char data);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(
unsigned char channel, unsigned char *data);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(
unsigned char channel, unsigned char data);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_duration(
unsigned char *accel_hg_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_duration(
unsigned char accel_hg_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_threshold(
unsigned char *accel_hg_thr);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_threshold(
unsigned char accel_hg_thr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_threshold(
unsigned char *accel_slow_no_thr);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_threshold(
unsigned char accel_slow_no_thr);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(
unsigned char *accel_slow_no_en);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(
unsigned char accel_slow_no_en);

BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_duration(
unsigned char *accel_slow_no_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_duration(
unsigned char accel_slow_no_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_axis_enable(
unsigned char channel, unsigned char *data);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_axis_enable(
unsigned char channel, unsigned char data);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(
unsigned char channel, unsigned char *data);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(
unsigned char channel, unsigned char data);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_filter(
unsigned char *gyro_am_filter);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_filter(
unsigned char gyro_am_filter);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(
unsigned char *gyro_hr_filter);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(
unsigned char gyro_hr_filter);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_threshold(
unsigned char *gyro_hr_x_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_threshold(
unsigned char gyro_hr_x_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hysteresis(
unsigned char *gyro_hr_x_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hysteresis(
unsigned char gyro_hr_x_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_duration(
unsigned char *gyro_hr_x_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_duration(
unsigned char gyro_hr_x_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_threshold(
unsigned char *gyro_hr_y_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_threshold(
unsigned char gyro_hr_y_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hysteresis(
unsigned char *gyro_hr_y_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hysteresis(
unsigned char gyro_hr_y_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_duration(
unsigned char *gyro_hr_y_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_duration(
unsigned char gyro_hr_y_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_threshold(
unsigned char *gyro_hr_z_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_threshold(
unsigned char gyro_hr_z_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hysteresis(
unsigned char *gyro_hr_z_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hysteresis(
unsigned char gyro_hr_z_hys);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_duration(
unsigned char *gyro_hr_z_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_duration(
unsigned char gyro_hr_z_dur);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_threshold(
unsigned char *gyro_am_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_threshold(
unsigned char gyro_am_thres);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_slope_samples(
unsigned char *gyro_am_slp);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_slope_samples(
unsigned char gyro_am_slp);

BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_awake_duration(
unsigned char *gyro_am_awk);

BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_awake_duration(
unsigned char gyro_am_awk);
#endif
