#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

bool enabledInAutonomous(IterativeRobot *me)
{
	if(me->IsAutonomous() && !me->IsDisabled())
	{
		return true;
	}
	return false;
}

/*
void GyroTurn(IterativeRobot *me, MPU6050_I2C *gyro, float tdegreeOfTurn, RobotDrive *drivetrain,
		Encoder *leftDT, Encoder *rightDT, NetworkTable *dataTable)

{
	printf("start\n");
	float degreeOfTurn;
	degreeOfTurn= dataTable->GetNumber("degreeOfTurn"); //TODO turn
		//me back into a parameter
	//degreeOfTurn = degreeOfTurn + 1;
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
	float maxPower = dataTable->GetNumber("maxPower");
	
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

*/
bool fuchsia (float degreeOfTurn, MPU6050_I2C *gyro)
{
	bool returnMe = false;
	if(degreeOfTurn > 0)
	{
		if(gyro->GetAngle() < degreeOfTurn)
		{
			returnMe = true;
		}
		else
		{
			returnMe = false;
		}
	}
	else
	{
		if(gyro->GetAngle() > degreeOfTurn)
		{
			returnMe = true;
		}
		else
		{
			returnMe = false;
		}
	}
	return returnMe;
}

void GyroTurnAngle(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		Encoder *leftDT, Encoder *rightDT, NetworkTable *dataTable)

{
	printf("start\n");
	float degreeOfTurn;
	degreeOfTurn= dataTable->GetNumber("degreeOfTurn"); 
	printf("others\n");
	//getting values from network table, the zero helps magically not get errors
	double kpError = dataTable->GetNumber("kpError", 0);
	double kiError = dataTable->GetNumber("kiError", 0);
	double kdError = dataTable->GetNumber("kdError", 0);
	double minPower = dataTable->GetNumber("minPower", 0);
	double maxPower = dataTable->GetNumber("maxPower", 0);
	printf("all\n");

	gyro->CalibrateRate(); //DO NOT BE MOVING
	float integral = 0.0;
	Wait(0.04);
	gyro->Reset();
	printf("Gyro reset %f\n", gyro->GetCalibratedAngle());
	float differential = gyro->GetCalibratedRate();
	float oldError = degreeOfTurn-gyro->GetCalibratedAngle();
//	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn)
	while(enabledInAutonomous(me) && fuchsia(degreeOfTurn, gyro))
	{
		float angle = gyro->GetCalibratedAngle();
		float aError = degreeOfTurn - angle;
		integral += aError;
		differential = gyro->GetCalibratedRate();
		float leftDriveTrain = kpError*(aError)/100; // multiply by constant
		leftDriveTrain += kiError*integral/100;
		printf("left drive train1: %f	", leftDriveTrain);
		leftDriveTrain -= kdError*differential/100;
		printf("left drive train2: %f\n", leftDriveTrain);
		oldError = aError;
		printf("differential: %f\n", differential);
		
		if(degreeOfTurn > 0.0 && leftDriveTrain < minPower)
		{
			leftDriveTrain = minPower;
		}
		else if(degreeOfTurn < 0.0 && leftDriveTrain > minPower)
		{
			leftDriveTrain = minPower;
		}
		if(degreeOfTurn < 0.0 && leftDriveTrain < -maxPower)
		{
			leftDriveTrain = -maxPower;
		}
		printf("Gyro: %f, leftDT: %f\n",
				angle,leftDriveTrain);
		printf("__________________________________________________________\n\n");
		//Drivetrain
		drivetrain->TankDrive(leftDriveTrain, -leftDriveTrain);
		
	}

	drivetrain->TankDrive(0.0,0.0);
}

void DriveStraight() //use CitrusPID here. can we also use it in gyroturn?
{
	
}
#endif	
