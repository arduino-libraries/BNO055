/*
 ***************************************************************************

    Euler_Streaming.pde - part of sample SW for using BNO055 with Arduino

   (C) All rights reserved by ROBERT BOSCH GMBH

   Copyright (C) 2014 Bosch Sensortec GmbH

 	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
/*	Date: 2014/01/07
 	 Revision: 1.2

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
struct bno055_euler myEulerData; //Structure to hold the Euler data

unsigned long lastTime = 0;

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
}

void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();

    bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure

    Serial.print("Time Stamp: ");				//To read out the Time Stamp
    Serial.println(lastTime);

    Serial.print("Heading(Yaw): ");				//To read out the Heading (Yaw)
    Serial.println(float(myEulerData.h) / 16.00);		//Convert to degrees

    Serial.print("Roll: ");					//To read out the Roll
    Serial.println(float(myEulerData.r) / 16.00);		//Convert to degrees

    Serial.print("Pitch: ");				//To read out the Pitch
    Serial.println(float(myEulerData.p) / 16.00);		//Convert to degrees

    Serial.println();					//Extra line to differentiate between packets
  }
}
