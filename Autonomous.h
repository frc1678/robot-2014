#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

void GyroTurn(MPU6050_I2C *gyro, float degreeOfTurn, RobotDrive *drivetrain)
{
	gyro->Reset();
	float integral = 0.0;
	Wait(0.04);

	printf("Gyro reset %f\n", gyro->GetAngle());
	float differential = 0.0;
	float oldError = 200*(degreeOfTurn-gyro->GetAngle())/90 - gyro->GetRate();
	while(gyro->GetAngle()<degreeOfTurn)
	{
		float angle = gyro->GetAngle();
		float aError = degreeOfTurn - angle;
		float target = 200*aError/90; //target velocity; change me!
		float vError = target - gyro->GetRate();
		integral += vError;
		differential = oldError -vError;
		float leftDriveTrain = 0.96*(target+vError)/200; // multiply by constant
		leftDriveTrain += 0.01*integral/200;
		leftDriveTrain -= 0.03*differential/200;

		oldError = vError;
		printf("Gyro: %f\n", gyro->GetRate());

		drivetrain->TankDrive(leftDriveTrain, -leftDriveTrain);

	}
}
#endif	
