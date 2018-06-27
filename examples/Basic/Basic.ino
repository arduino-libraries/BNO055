/*
 ***************************************************************************
 *
 *  Basic.pde - part of sample SW for using BNO055 with Arduino
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

#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;

void setup() //This code is executed once
{
	//Initialize I2C communication
	Wire.begin();

	//Initialization of the BNO055
	BNO_Init(&myBNO); //Assigning the structure to hold information about the device

	//Configuration to NDoF mode
	bno055_set_operation_mode(OPERATION_MODE_NDOF);

	delay(1);

	//Initialize the Serial Port to view information on the Serial Monitor
	Serial.begin(115200);

	//Read out device information
	Serial.print("Chip ID: ");
	Serial.println(myBNO.chip_id);

	//Read out the software revision ID
	Serial.print("Software Revision ID: ");
	Serial.println(myBNO.sw_revision_id);

	//Read out the page ID
	Serial.print("Page ID: ");
	Serial.println(myBNO.page_id);

	//Read out the accelerometer revision ID
	Serial.print("Accelerometer Revision ID: ");
	Serial.println(myBNO.accel_revision_id);

	//Read out the gyroscope revision ID
	Serial.print("Gyroscope Revision ID: ");
	Serial.println(myBNO.gyro_revision_id);

	//Read out the magnetometer revision ID
	Serial.print("Magnetometer Revision ID: ");
	Serial.println(myBNO.mag_revision_id);

	//Read out the bootloader revision ID
	Serial.print("Bootloader Revision ID: ");
	Serial.println(myBNO.bootloader_revision_id);

	//Read out the device address
	Serial.print("Device Address: ");
	Serial.println(myBNO.dev_addr);
}

void loop() //This code is looped forever
{
  //Blank
}