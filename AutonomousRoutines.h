#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"
#include "SecondaryRollerSystem.h"
#include "AutonomousSubroutines.h"
#include "ShooterSystem.h"
#include "AutonomousComponents.h"

#ifndef AUTONOMOUSROUTINES_H
#define AUTONOMOUSROUTINES_H

void ShootThreeAndDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightDT)
{
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	
	ShootLoadFrontAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	//Switch to "short shot" and load.
	LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	//Shoot again.
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);

	DriveForwardAuto(drivetrain, timer, me, rightDT);
}

void ShootThreeAndDriveV2(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightDT)
{
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	
	//Shoot again.
	ShootAutoPrepBack(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	
	//Switch to "short shot" and load.
	LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	ShootLoadFrontAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
	
	//Shoot again.
	ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);

	//DriveForwardAuto(drivetrain, timer, me);	
}

void TwoShot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightDT)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	//LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	printf("Done driving!");
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
}

void CheckVision(IterativeRobot *me, NetworkTable *table)
{
	float visionResult = ReceiveVisionProcessing(table);
	if(visionResult == 2.0)
	{
		printf("Go to the right\n");
	}
	else
	{
		printf("Go to the left\n");
	}
}
void TwoShotWithVision(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *leftDT, Encoder *rightDT, MPU6050_I2C *gyro, NetworkTable *table)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	//Wait(1.0);
	float visionResult = ReceiveVisionProcessing(table);
	if(visionResult == 2.0)
	{
		printf("Go to the right");
		SpinAutoClock(52, drivetrain, leftDT, rightDT, me);
	}
	else if(visionResult == 1.0)
	{
		printf("Go to the left");
		SpinAutoAnti(52, drivetrain, leftDT, rightDT, me);
	}
	else
	{
		SpinAutoAnti(52, drivetrain, leftDT, rightDT, me);
		printf("This should NOT have happened!");
	}
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	//TODO load back while turning.
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	if(visionResult == 2.0)
	{
		SpinAutoAnti(104, drivetrain, leftDT, rightDT, me); 
	}
	else if(visionResult == 1.0)
	{
		SpinAutoClock(104, drivetrain, leftDT, rightDT, me);
	}
	else
	{
		SpinAutoAnti(104, drivetrain, leftDT, rightDT, me);
		printf("This should NOT have happened!");
	}
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
}

void OneShot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightDT)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
}
void ShootTwoThenOne(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Timer *shottimer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightDT)
{ 
	shottimer->Reset();
	shottimer->Start();
	frontIntake->FrontRollerAutoSlow();

	backIntake->BackRollerAutoSlow();
	
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	
	ShootLoadFrontAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
	
	while(EnabledInAutonomous(me))
	{
		if(shottimer->Get() > 4.5)
		{
			break;			
		}
	}
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
		
	//Switch to "short shot" and load.
	LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	//Shoot again.
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightDT);
		
	DriveForwardAuto(drivetrain, timer, me, rightDT);
}

void heliotrope(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Timer *goalTimer, SecondaryRollerSystem *secondaryRollers, MPU6050_I2C *gyro, 
		float autoDirection, IterativeRobot *me)
{
	OpenFlower(frontIntake, backIntake, secondaryRollers);
	Wait(0.5);
	goalTimer->Reset();
	goalTimer->Start();
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	if(autoDirection == 1.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585);//TODO constents 
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55);//TODO constents 
	}
	//TODO incorporate hot goal stuff.
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	if(autoDirection == 1.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, 20.0, 2.0, 0.14, 0.55);//TODO constents
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, -20.0, 2.0, 0.15, 0.585);//TODO constents
	}
	while(EnabledInAutonomous(me))
	{
		if(goalTimer->Get() > 4.5)
		{
			break;
		}
	}
	
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
}

void TurnRightThenLeft(RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	SpinAutoClock(52, drivetrain, leftDT, rightDT, me);
	SpinAutoAnti(104, drivetrain, leftDT, rightDT, me);
}
void ThreeBallVisionRight(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Timer *goalTimer, SecondaryRollerSystem *secondaryRollers, MPU6050_I2C *gyro, 
		IterativeRobot *me, NetworkTable *table)
{
	//TODO REDUCE TIMES 
	OpenFlower(frontIntake, backIntake, secondaryRollers);
	table->PutNumber("Start Side", 2.0);
	Wait(0.6);
	float autoDirection = ReceiveVisionProcessing(table);
	goalTimer->Reset();
	goalTimer->Start();
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	if(autoDirection == 1.0)
	{
		printf("LEFT!\n");
		GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585);//TODO constents 
		//GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55);//TODO constents 
	}
	else
	{
		printf("RIGHT!\n");
		GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55);//TODO constents 
		//GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585);//TODO constents 
	}
	//TODO incorporate hot goal stuff.
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	if(autoDirection == 1.0)
	{
		GyroTurnLoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, 
				me, gyro, drivetrain, -20.0, 2.0, 0.15, 0.585);//TODO constents
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnLoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, 
				me, gyro, drivetrain, 20.0, 2.0, 0.14, 0.55);//TODO constents
	}
	while(EnabledInAutonomous(me))
	{
		if(goalTimer->Get() > 4.5)
		{
			break;
		}
	}
	
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
}

void wisteria(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, MPU6050_I2C *gyro,
		IterativeRobot *me)
{
	//goalTimer->Reset();
	//goalTimer->Start();
	OpenFlower(frontIntake, backIntake, secondaryRollers);
	GyroTurnAngle(me, gyro, drivetrain,-10.0, 2.0, 0.04, 0.65); 
	//TODO incorporate hot goal stuff.
	//ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	//ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	GyroTurnAngle(me, gyro, drivetrain, 20.0, 2.0, 0.04, 0.65);
	/*while(enabledInAutonomous(me))
	{
		if(goalTimer->Get() > 4.5)
		{
			break;
		}
	}*/
	
	//LoadBackAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	//ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
}

#endif	
