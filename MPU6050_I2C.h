/* Class for the MPU6050 gyro
 * Plugs into the I2C port on digital sidecar
 * Relevant datasheets:
 * http://www.invensense.com/mems/gyro/documents/RM-MPU-6000A-00v4.2.pdf (register map)
 * http://invensense.com/mems/gyro/documents/PS-MPU-6000A-00v3.4.pdf (datasheet)
 */
//TODO fix magic numbers

#include "WPILib.h"

#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

class MPU6050_I2C
{
	//variables
	DigitalModule *dModule;
	I2C* me;
	Timer* gyroTimer;
	float gyroAngle;
public:
	MPU6050_I2C()
	{
		dModule = DigitalModule::GetInstance(1);
		me = dModule->GetI2C(0x68<<1);
		gyroTimer = new Timer();
		InitGyro();
	}
	void InitGyro()
	{
		//do stuff like reset/recalibrate in here
		gyroAngle = 0.0;
		gyroTimer->Start();
		me->Write(0x6B, 0); //PWR_MGMT_1:=0 take me out of sleep mode
	}
	
	void Reset()
	{
		gyroAngle = 0.0;
		gyroTimer->Reset();
	}
	float GetRate()
	{
		uint8_t gyroValue = 7;
		int GYRO_ZOUT_H;
		int GYRO_ZOUT_L;
		me->Read(0x47, sizeof(gyroValue), &gyroValue);
			GYRO_ZOUT_H=gyroValue;
		me->Read(0x48, sizeof(gyroValue), &gyroValue);
			GYRO_ZOUT_L=gyroValue;
		float zOutput;
		zOutput = (float)((GYRO_ZOUT_H<<8)+GYRO_ZOUT_L);
		//take care of signed/unsigned
		if (zOutput> 32768)
		{
			zOutput -=  32768*2;
		}
		zOutput /= 131.0; //TODO allow for other options in register 0d27
		return zOutput;
	}
	float GetAngle() //this must be called every time in a loop.
	{
		//TODO deal with the first gyro reading not being reset.
		float currRate = GetRate();
		if(currRate < 0.5 && currRate > -0.5) //Deadzone for rotation?
		{
			currRate = 0.0;
		}
		gyroAngle += currRate*gyroTimer->Get();
		gyroTimer->Reset();
		return gyroAngle;
	}
};

#endif