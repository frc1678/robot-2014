#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

bool amethyst(IterativeRobot *me)
{
	if(me->IsAutonomous() && !me->IsDisabled())
	{
		return true;
	}
	return false;
}


void GyroTurn(IterativeRobot *me, MPU6050_I2C *gyro, float tdegreeOfTurn, RobotDrive *drivetrain,
		Encoder *leftDT, Encoder *rightDT, NetworkTable *dataTable)

{
	printf("start\n");
	float degreeOfTurn;
	degreeOfTurn= dataTable->GetNumber("degreeOfTurn"); //TODO turn
		//me back into a parameter
	degreeOfTurn = degreeOfTurn + 1;
	printf("others\n");
	float kpError = dataTable->GetNumber("kpError");
	float kiError = dataTable->GetNumber("kiError");
	float kdError = dataTable->GetNumber("kdError");
	printf("all\n");
	
	float integral = 0.0;
	Wait(0.04);

	float targetSlopeNum = dataTable->GetNumber("targetSlopeNum");
	float targetSlopeDenom = dataTable->GetNumber("targetSlopeDenom");
	float targetEndConstant = dataTable->GetNumber("targetEndConstant");
	float minPower = dataTable->GetNumber("minPower");
	
	gyro->Reset();
	printf("Gyro reset %f\n", gyro->GetAngle());
	float differential = 0.0;
	float oldError = targetSlopeNum*(degreeOfTurn-gyro->GetAngle())/targetSlopeDenom - gyro->GetRate();
	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn-1)
	{
		float angle = gyro->GetAngle();
		float aError = degreeOfTurn - angle;
		float target = (targetSlopeNum*aError/targetSlopeDenom); 
		//target velocity; change me!
		float vError = target - gyro->GetRate();
		integral += vError;
		differential = oldError -vError;
		float leftDriveTrain = kpError*(target+vError)/200; // multiply by constant
		leftDriveTrain += kiError*integral/200;
		leftDriveTrain -= kdError*differential/200;
		leftDriveTrain = (1 - targetEndConstant) * leftDriveTrain + targetEndConstant; 

		if(leftDriveTrain < minPower)
		{
			leftDriveTrain = minPower;
		}
		
		oldError = vError;
		printf("Gyro: %f\n, LDT: %d, RDT: %d, Target:%f, leftDT: %f", gyro->GetAngle(), leftDT->Get(), rightDT->Get(), target, leftDriveTrain);

		drivetrain->TankDrive(leftDriveTrain, -leftDriveTrain);

	}
	drivetrain->TankDrive(0.0,0.0);
}

void DriveStraight() //use CitrusPID here. can we also use it in gyroturn?
{
	
}
#endif	
