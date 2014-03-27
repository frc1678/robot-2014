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

/*void ShootThreeAndDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	
	//ShootLoadFrontAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	//Switch to "short shot" and load.
	//LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	//Shoot again.
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);

	//DriveForwardAuto(drivetrain, timer, me, rightEncoder);
}*/

void TwoShotShortLong(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	spitShortSwap->Set(false);
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	//ShootAutoLoadBack(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	Wait(1.5);
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	spitShortSwap->Set(true);
	LoadBackAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
	spitShortSwap->Set(true);
	Wait(1.0);
	//drive and shoot.
	ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me, drivetrain, rightEncoder);
}

void TwoShotShortShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, 
		DriverStation *driverStation)
{
	spitShortSwap->Set(true);
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	
	TwoShotShortPrep(shooter, timer2); 
	
	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
			driverStation);
	frontIntake->DeployIntake();
	backIntake->DeployIntake(); 
	Wait(1.0);
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
	timer2->Stop();
	/*ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRolles, spitShortSwap, me, drivetrain, rightEncoder);
	LoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, spitShortSwap, me);*/
}


void OneShotShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, DriverStationLCD *driverStationLCD, NetworkTable *table, float startSide)
{
	table->PutNumber("Enabled", 1);
	spitShortSwap->Set(true);
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	Wait(1.0);
	float visionInput = ReceiveVisionProcessing(table);
	driverStationLCD->PrintfLine((DriverStationLCD::Line)0,"Vision: %f", visionInput);
	driverStationLCD->UpdateLCD();
	shooter->ShooterPrime(true);
	
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false;
	
	bool backintakeup = false; //unused
	
	timer2->Reset();
	bool stopSecondary = false;
	if(visionInput == 0.0)
	{
		Wait(2.0);
	}
	else if(visionInput != startSide) //both are 1.0 for left and 2.0 for right
	{
		Wait(4.0);
	}
	
	while(ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone)
	{
		//first
		if(rightEncoder->Get() > -500)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second
		if(!shootPrep && rightEncoder->Get() <- 2300) //3 feet forward? TODO number
		{
			timer2->Start();
			timer2->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if(shootPrep && ShootAutoConditions(shooter, me))
		{
			ShootAutoInLoop(shooter);
		}
		else if(shootPrep && !doneShooting)
		{
			ShootAutoEnd();
		}
		//first
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
			break;
		}
	}
	frontIntake->UndeployIntake();
	backIntake->UndeployIntake();
	autoTimer->Stop();
	timer2->Stop();
	/*ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRolles, spitShortSwap, me, drivetrain, rightDT);
	LoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, spitShortSwap, me);*/
}

/*void ThreeShortShot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	ShootAutoLoadBack(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	//drive and shoot.
	ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
	LoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	ShootShortAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
}

void ShootThreeAndDriveV2(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	
	//Shoot again.
	ShootAutoLoadBack(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	
	//Switch to "short shot" and load.
	LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	
	ShootLoadFrontAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
	
	//Shoot again.
	ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);

	//DriveForwardAuto(drivetrain, timer, me);	
}*/

void ThreeShotShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain,DriverStation *driverStation, Timer *autoTimer, Timer *timer2,
		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap, IterativeRobot *me, 
		Encoder *rightEncoder, NetworkTable *table, float startSide)
{
	//table->PutNumber("Enabled", 1);
	spitShortSwap->Set(true);
	printf("secondary pistons set true\n");
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	printf("Loaded from top\n");
	//float visionInput = ReceiveVisionProcessing(table);
	shooter->ShooterPrime(true);
	printf("Shooter primed\n");
	Wait(1.0);
	printf("Past wait");
		
	TwoShotShortPrep(shooter, timer2); 

	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
			driverStation);
	printf("Hi! WATCH ME ACTUALLY WORK FOR THREE BALL!\n");
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	autoTimer->Stop();
	LoadBackAutoDrive(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
	//Wait(1.0);
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
}

void TwoShot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer,
		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap, IterativeRobot *me, Encoder *rightEncoder)
{
	spitShortSwap->Set(false);
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	
	Wait(1.5);
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	//LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	spitShortSwap->Set(true);
	LoadBackAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
	printf("Done driving!");
	spitShortSwap->Set(false);
	Wait(1.0);
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me, drivetrain, rightEncoder);
}

/*void CheckVision(IterativeRobot *me, NetworkTable *table)
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
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, 
		Encoder *leftEncoder, Encoder *rightEncoder, MPU6050_I2C *gyro, 
		NetworkTable *table)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	//Wait(1.0);
	float visionResult = ReceiveVisionProcessing(table);
	if(visionResult == 2.0)
	{
		printf("Go to the right");
		SpinAutoClock(52, drivetrain, leftEncoder, rightEncoder, me);
	}
	else if(visionResult == 1.0)
	{
		printf("Go to the left");
		SpinAutoAnti(52, drivetrain, leftEncoder, rightEncoder, me);
	}
	else
	{
		SpinAutoAnti(52, drivetrain, leftEncoder, rightEncoder, me);
		printf("This should NOT have happened!");
	}
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	//TODO load back while turning.
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	if(visionResult == 2.0)
	{
		SpinAutoAnti(104, drivetrain, leftEndoer, rightEncoder, me); 
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
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, timer, shooter, me);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
}
void ShootTwoThenOne(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Timer *shottimer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
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
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain, rightEncoder);
		
	DriveForwardAuto(drivetrain, timer, me, rightEncoder);
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

void TurnRightThenLeft(RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder, IterativeRobot *me)
{
	SpinAutoClock(52, drivetrain, leftEncoder, rightEncoder, me);
	SpinAutoAnti(104, drivetrain, leftEncoder, rightEncoder, me);
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
//}

#endif	
