#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"
#include "SecondaryRollerSystem.h"
#include "ShooterSystem.h"
#include "HumanLoad.h"


#ifndef AUTONOMOUSCOMPONENTS_H
#define AUTONOMOUSCOMPONENTS_H

bool enabledInAutonomous(IterativeRobot *me)
{
	if (me->IsAutonomous() && !me->IsDisabled())
	{
		return true;
	}
	return false;
}

bool TurnIncomplete(float degreeOfTurn, MPU6050_I2C *gyro)
{
	bool returnMe = false;
	if (degreeOfTurn > 0)
	{
		if (gyro->GetAngle() < degreeOfTurn)
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
		if (gyro->GetAngle() > degreeOfTurn)
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

void GyroTurnAnglePrep(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn, double kpError, double kiError, double kdError, float integral, 
		float differential, float oldError)
{
	gyro->CalibrateRate(); //DO NOT BE MOVING
	integral = 0.0;
	Wait(0.04);
	gyro->Reset();
	printf("Gyro reset %f\n", gyro->GetCalibratedAngle());
	differential = gyro->GetCalibratedRate();
	//float oldError = degreeOfTurn+gyro->GetCalibratedAngle();
	oldError = degreeOfTurn-gyro->GetCalibratedAngle();
	//	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn)
}

bool GyroTurnAngleConditions(MPU6050_I2C *gyro, float degreeOfTurn, IterativeRobot *me)
{
	if(TurnIncomplete(degreeOfTurn, gyro) && enabledInAutonomous(me))
	{
		return true;
	}
	return false;
}

void GyroTurnAngleInLoop(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn, double kpError, double kiError, double kdError,
		float integral, float differential, float oldError)
	{
		float angle = gyro->GetCalibratedAngle();
		float aError = degreeOfTurn - angle;
		//float aError = degreeOfTurn + angle;
		integral += aError;
		printf("Error: %f\n", aError);
		differential = gyro->GetCalibratedRate();
		float leftDriveTrain = kpError*(aError)/100; // multiply by constant
		leftDriveTrain += kiError*integral/100;
		printf("integral: %f\n", integral);
		leftDriveTrain -= kdError*differential/100;
		printf("left drive train2: %f\n", leftDriveTrain);
		oldError = aError;
		printf("differential: %f\n", differential);
		printf("Gyro: %f, leftDT: %f\n",
		angle,leftDriveTrain);
		printf("__________________________________________________________\n\n");
		//Drivetrain
		drivetrain->TankDrive(leftDriveTrain, -leftDriveTrain);

	}

void GyroTurnAngleEnd(RobotDrive *drivetrain)
{
	drivetrain->TankDrive(0.0,0.0);
}


void DriveStraight(RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT,
		IterativeRobot *me) //use CitrusPID here. can we also use it in gyroturn?
{
	leftDT->Reset();
	rightDT->Reset();
	leftDT->Start();
	rightDT->Start();
	while(leftDT->Get() < 300 && rightDT->Get() > -300 && enabledInAutonomous(me))
	{
		drivetrain->TankDrive(-1.0, -1.0);
	}
}

//TODO put this in somewhere that implies not just autonomous.
void OpenFlower(IntakeSystem *frontIntake, IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers)
{
	//Unfurling the flower
	secondaryRollers->Deploy();
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
}

void ShootAutoPrep(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, SecondaryRollerSystem *secondaryRollers)
{
	OpenFlower(frontIntake, backIntake, secondaryRollers);
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	shooter->ShooterPrime(false);
	Wait(0.5); //TODO how short can this be?
	
	shooter->BeginShooterFire();
	
}

bool ShootAutoConditions(ShooterSystem *shooter, IterativeRobot *me)
{
	if(shooter->CurrentlyShooting() && enabledInAutonomous(me))
	{
		return true;
	}
	return false;
}

void ShootAutoInLoop(ShooterSystem *shooter)
{
	shooter->ShooterFire();
}

void ShootAutoEnd()
{
	
}

void LoadBackAutoPrep(ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake)
{
	frontIntake->UndeployIntake();
	shooter->ShooterPrime(true);
	secondaryRollers->Undeploy();
	timer->Reset();
	timer->Start();
}
 
bool LoadBackAutoConditions(Timer *timer, IterativeRobot *me)
{
	if(enabledInAutonomous(me) && timer->Get() < 1.4)
	{
		return true;
	}
	return false;
}

void LoadBackAutoDriveInLoop(IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers, RobotDrive *drivetrain, Timer *timer)
{
	backIntake->BackRollerLoad();
	secondaryRollers->Pulse();
	if(timer->Get() < 0.25)
	{
		drivetrain->TankDrive(-0.5, -0.5);
	}
}

void LoadBackAutoInLoop(IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers, RobotDrive *drivetrain, Timer *timer)
{
	backIntake->BackRollerLoad();
	secondaryRollers->Pulse();
	/*if(timer->Get() < 0.25)
	{
		drivetrain->TankDrive(-0.5, -0.5);
	}*/
}

void LoadBackAutoEnd(IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers, ShooterSystem *shooter)
{
	backIntake->Stop();
	secondaryRollers->Stop();
	shooter->ShooterPrime(false);
	printf("Primed!\n");
}

void LoadFrontAutoPrep(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		SecondaryRollerSystem *secondaryRollers, Timer *timer)
{
	//secondaryRollers->Undeploy;
	timer->Reset();
	timer->Start();
}

bool LoadFrontAutoConditions(IterativeRobot *me, Timer *timer)
{
	if(enabledInAutonomous(me) && timer->Get() < 1.4+1.0)
	{
		return true;
	}
	return false;
}

void LoadFrontAutoDriveInLoop(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake,
		Timer *timer, RobotDrive *drivetrain)
{
	if(timer->Get() > 0.75 )
	{
		
		frontIntake->FrontRollerLoad();
		secondaryRollers->Pulse();
		secondaryRollers->Undeploy();
		if(timer->Get() < 1.0)
		{
			drivetrain->TankDrive(0.5, 0.5);
		}
		else
		{
			drivetrain->TankDrive(0.0, 0.0);
		}
	}
}

void LoadFrontAutoInLoop(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake,
		Timer *timer, RobotDrive *drivetrain)
{
	if(timer->Get() > 1.0 )
	{
		
		frontIntake->FrontRollerLoad();
		secondaryRollers->Pulse();
		secondaryRollers->Undeploy();
		/*if(timer->Get() < 1.25)
		{
			drivetrain->TankDrive(0.5, 0.5);
		}
		else
		{
			drivetrain->TankDrive(0.0, 0.0);
		}*/
	}
}

void LoadFrontAutoEnd(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake)
{
	secondaryRollers->Stop();
	frontIntake->Stop();
}

void LoadTopAuto(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake, IntakeSystem *backIntake, Timer *timer)
{
	timer->Reset();
	timer->Start();
	while(timer->Get() < 0.7)
	{
		frontIntake->Reverse();
		backIntake->Reverse();
		secondaryRollers->Pulse();
	}
}

float ReceiveVisionProcessing(NetworkTable *table)
{
	//table->PutNumber("Start Side", startSide);
	//bool tablevalue = table->GetBoolean("Shot");
	//string startside = table->GetString("startside");
	//driverStationLCD = new DriverStationLCD; 
	float autoDirection = 0.0;
	string direction = table->GetString("Direction: ");
	
	if (direction == "GO RIGHT")
	{
		autoDirection = 2.0;
	}
	else if (direction == "GO LEFT")
	{
		autoDirection = 1.0;
	}
	else 
	{
		autoDirection = 1.0;
	}
	
	return autoDirection;
}

void DriveForwardAutoPrep(Timer *timer)
{
	timer->Start();
	timer->Reset();
}

bool DriveForwardAutoConditions(Timer *timer, IterativeRobot *me)
{
	if(timer->Get() < 2.0 && enabledInAutonomous(me))
	{
		return true;
	}
	return false;
}

void DriveForwardAutoInLoop(RobotDrive *drivetrain)
{
	drivetrain->TankDrive(-0.75, -0.75);
}

void DriveForwardAutoEnd(RobotDrive *drivetrain)
{
	drivetrain->TankDrive(0.0, 0.0);
}

#endif	
