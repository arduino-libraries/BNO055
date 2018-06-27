/*
 ***************************************************************************
 *
 *  Command_Line_Configuration.pde - part of sample SW for using BNO055 with Arduino
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
struct bno055_euler myEulerData;	//Structure to hold the Euler data

//Variables and Flags
unsigned long lastTime = 0;
bool streamReqd = false;	//Flag to indicate the requirement for streaming of data

//Function Prototypes
void streamData(void);		//Function to stream data
void getCommand(void);		//Function to receive the command and parse it
void execCommand(char ,int);	//Function to execute relevant functions depending on the command

void setup() //This code is executed once
{
	//Initialize I2C communication
	Wire.begin();

	//Initialization of the BNO055
	BNO_Init(&myBNO); //Assigning the structure to hold information about the device

	//Configuration to NDoF mode (Currently defaulted to NDoF)
	bno055_set_operation_mode(OPERATION_MODE_NDOF);

	delay(1);

	//Initialize the Serial Port to view information on the Serial Monitor
	Serial.begin(115200);

	//Indication on the Serial Monitor that the Initialization is complete
	Serial.println("Initialization Complete"); 
	Serial.println("Set the terminal character to newline and baud rate to 115200");
	Serial.println("List of commands:");
	Serial.println("s toggles streaming of Euler data");
	Serial.println("c0 to c12 changes the Operation mode");
	Serial.println("p0 to p2 changes the Power mode");
}

void loop() //This code is looped forever
{
	if(streamReqd) //If data needs to be streamed then stream data
		streamData();
		getCommand();//To look for incoming UART commands and call relevant functions
}

void streamData(void)
{
  if((millis()-lastTime) >= 100) //To stream at 10Hz without using additional timers
	{
		lastTime = millis();
		bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure

		Serial.print("Time Stamp: ");					//To read out the Time Stamp
		Serial.println(lastTime);

		Serial.print("Heading(Yaw): ");					//To read out the Heading (Yaw)
		Serial.println(float(myEulerData.h)/16.00);		//Convert to degrees

		Serial.print("Roll: ");							//To read out the Roll
		Serial.println(float(myEulerData.r)/16.00);		//Convert to degrees

		Serial.print("Pitch: ");						//To read out the Pitch
		Serial.println(float(myEulerData.p)/16.00);		//Convert to degrees

		Serial.println();								//Extra line to differentiate between packets
	}  
}

void getCommand(void)
{
	int commPos = 0;//Register used to keep track of the index of the command
	char par1 = 0;//To store the first parameter of the command
	int par2 = 0;//To store the seconds parameter of the command
	char command[10] = {0};//Array to store the incoming commands
	int index;
	for(index = 0; index < 10; index++)//Initialize the command array to NULL
		command[index] = 0;
	if(Serial.available())
	{
		int commLen = Serial.readBytesUntil('\n', &command[0], 10);//Store the command in an array and store the length of the incoming command
		for(index = 0; index < 10; index++)//Echo the incoming command
			Serial.print(command[index]);
		Serial.println();
		par1 = command[0]; //Store the first parameter of the command
		commPos++;
		while((command[commPos] >= '0') && (command[commPos] <= '9'))//To process digits [0-9]+ and store in par2
		{
			par2 *= 10;//Shift the digit position
			par2 += command[commPos] - '0';//Convert ASCII to Integer
			commPos++;//Increment the position of the array
		}   
		execCommand(par1, par2);
	}
}
 
void execCommand(char head, int tail)
{
	switch(head)
	{
		case 's': //Command to toggle Streaming of data
			streamReqd = !streamReqd;
			if(streamReqd)
				Serial.println("Streaming ON");
			else
				Serial.println("Streaming OFF");
		break;

		case 'c': //Command to change the operation mode
			streamReqd = false; //Comment this line if you want to data streaming to be kept on
			switch(tail)
			{
				case 0:
					Serial.println("Set into Configuration Mode");
					bno055_set_operation_mode(OPERATION_MODE_CONFIG);
				break;
				
				case 1:
					Serial.println("Set into Accelerometer Only Mode");
					bno055_set_operation_mode(OPERATION_MODE_ACCONLY);
				break;
				
				case 2:
					Serial.println("Set into Magnetometer Only Mode");
					bno055_set_operation_mode(OPERATION_MODE_MAGONLY);
				break;
				
				case 3:
					Serial.println("Set into Gyroscope Only Mode");
					bno055_set_operation_mode(OPERATION_MODE_GYRONLY);
				break;
				
				case 4:
					Serial.println("Set into Accelerometer and Magnetometer Mode");
					bno055_set_operation_mode(OPERATION_MODE_ACCMAG);
				break;
				
				case 5:
					Serial.println("Set into Accelerometer and Gyroscope Mode");
					bno055_set_operation_mode(OPERATION_MODE_ACCGYRO);
				break;
				
				case 6:
					Serial.println("Set into Magnetometer and Gyroscope Mode");
					bno055_set_operation_mode(OPERATION_MODE_MAGGYRO);        
				break;
				
				case 7:
					Serial.println("Set into Accelerometer, Magnetometer and Gyroscope Mode");
					bno055_set_operation_mode(OPERATION_MODE_AMG);        
				break;
				
				case 8:
					Serial.println("Set into Sensor Fusion IMU Plus Mode");
					bno055_set_operation_mode(OPERATION_MODE_IMUPLUS);        
				break;
				
				case 9:
					Serial.println("Set into Sensor Fusion Compass Mode");
					bno055_set_operation_mode(OPERATION_MODE_COMPASS);        
				break;
				
				case 10:
					Serial.println("Set into Sensor Fusion Magnetometer for Gyroscope Mode");
					bno055_set_operation_mode(OPERATION_MODE_M4G);        
				break;
				
				case 11:
					Serial.println("Set into Sensor Fusion NDoF Mode with Fast Magnetometer Calibration Off");
					bno055_set_operation_mode(OPERATION_MODE_NDOF_FMC_OFF);        
				break;
				
				case 12:
					Serial.println("Set into Sensor Fusion NDoF Mode");
					bno055_set_operation_mode(OPERATION_MODE_NDOF);        
				break;
				
				default:
					Serial.println("Invalid Configuration Mode");
			}
		break;
		
		case 'p': //To change power modes
		streamReqd = false; //Comment this line if you want to data streaming to be kept on
			switch(tail)
			{
				case 0:
					Serial.println("Set into Normal Power Mode");
					bno055_set_powermode(POWER_MODE_NORMAL);
				break;
				
				case 1:
					Serial.println("Set into Low Power Mode");
					bno055_set_operation_mode(POWER_MODE_LOW_POWER);
				break;
				
				case 2:
					Serial.println("Set into Suspend Power Mode");
					bno055_set_operation_mode(POWER_MODE_SUSPEND);
				break;
				
				default:
					Serial.println("Invalid Power Mode");
			}
		break;
		
		default:
			Serial.println("Invalid Command");  
  }
  
}