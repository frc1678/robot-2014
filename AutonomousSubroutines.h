#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"
#include "SecondaryRollerSystem.h"
#include "AutonomousComponents.h"


#ifndef AUTONOMOUSSUBROUTINES_H
#define AUTONOMOUSSUBROUTINES_H

//Positive is anticlock
void GyroTurn(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain, float degreeOfTurn)
{
	printf("turn");
	gyro->Reset();
	while(GyroTurnAngleConditions(gyro, degreeOfTurn, me))
	{
		printf("in loop");
		if(degreeOfTurn<0.0)
		{
			drivetrain->TankDrive(-0.5, 0.5); 
		}
		else
		{
			drivetrain->TankDrive(0.5, -0.5);
		}
	}
	printf("end");
	gyro->Stop();
	drivetrain->TankDrive(0.0, 0.0);
}

void GyroTurnAngle(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn, double kpError, double kiError, double kdError)
{
	
	float integral = 0.0;
	float oldError = degreeOfTurn - gyro->GetAngle();
	float differential = 0.0;

	GyroTurnAnglePrep(me, gyro, drivetrain,degreeOfTurn, kpError, kiError,
			kdError, integral, differential, oldError);
	
	while (GyroTurnAngleConditions(gyro, degreeOfTurn, me))
	{
		GyroTurnAngleInLoop(me, gyro, drivetrain,degreeOfTurn, kpError, kiError,
		kdError, integral, differential, oldError);
	}

	GyroTurnAngleEnd(drivetrain);
}


void GyroTurnAngle(IterativeRobot *me, MPU6050_I2C *gyro,
		RobotDrive *drivetrain,	NetworkTable *dataTable)
{
	printf("start\n");
	float degreeOfTurn;
	degreeOfTurn= dataTable->GetNumber("degreeOfTurn");
	printf("others\n");
	//getting values from network table, the zero helps magically not get errors
	double kpError = dataTable->GetNumber("kpError", 0);
	double kiError = dataTable->GetNumber("kiError", 0);
	double kdError = dataTable->GetNumber("kdError", 0);
	//double minPower = dataTable->GetNumber("minPower", 0);
	//double maxPower = dataTable->GetNumber("maxPower", 0);
	printf("all\n");
	
	GyroTurnAngle(me, gyro, drivetrain, degreeOfTurn, kpError, kiError, kdError);
}

void SpinAutoClock(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder, IterativeRobot *me)
{
	/*leftDT->Reset();
	rightDT->Reset();
	while (EnabledInAutonomous(me) && (leftDT->Get()>(0-numEncoderClicks)&& rightDT->Get()<(numEncoderClicks)))
	{
		drivetrain->TankDrive(-0.75, 0.75);
	}
	drivetrain->TankDrive(0.0, 0.0);*/
	SpinAutoPrep(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	while(SpinAutoClockConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me))
	{
		SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
}
	//Spin counterclockwise for (approximately) so many encoder clicks. Anti is shorter than counter. Un is shorter than Anti.
void SpinAutoAnti(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder, IterativeRobot *me)
{
	/*
	leftDT->Reset();
	rightDT->Reset();
	while (EnabledInAutonomous(me) && (leftDT->Get()<(numEncoderClicks)
			&& rightDT->Get()>(-numEncoderClicks)))
	{
		drivetrain->TankDrive(0.75, -0.75);
	}
	drivetrain->TankDrive(0.0, 0.0);*/

	SpinAutoPrep(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	while(SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me))
	{
		SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
}
	
void ShootAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	while(ShootAutoConditions(shooter, me))
	{
		ShootAutoInLoop(shooter);
	}
	ShootAutoEnd();
}

void ShootShortAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
	while(ShootAutoConditions(shooter, me))
	{
		ShootAutoInLoop(shooter);
	}
	ShootAutoEnd();
}

void ShootLoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me, RobotDrive *drivetrain)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) 
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoInLoop(secondaryRollers, frontIntake, timer, drivetrain);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}

void ShootLoadFrontAutoDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightEncoder)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) 
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoDriveInLoop(secondaryRollers, frontIntake, timer, drivetrain, rightEncoder);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}

void ShootAutoLoadBack(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,IterativeRobot *me, RobotDrive *drivetrain)
{
	timer->Start();
	timer->Reset();
	bool secondary = false;
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	while(ShootAutoConditions(shooter, me))
	{
		ShootAutoInLoop(shooter);
		if(timer->Get() > 1.0 && !secondary)
		{
			secondaryRollers->Undeploy();
			shooter->ShooterPrime(true);
		}
		if(timer->Get() > 2.25)
		{
			LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, timer);
		}
	}
	ShootAutoEnd();
}

void ShootDriveForwardAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightEncoder)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	bool driveInit = false;
	autoTimer->Start();
	autoTimer->Reset();
	
	while(ShootAutoConditions(shooter, me) || DriveForwardAutoConditions(autoTimer, me, rightEncoder))
	{
		if(!driveInit && autoTimer->Get() > 1.0)
		{
			DriveForwardAutoPrep(autoTimer, rightEncoder);
			driveInit = true;
			backIntake->UndeployIntake();
		}
		if(driveInit && DriveForwardAutoConditions(autoTimer, me, rightEncoder))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		ShootAutoInLoop(shooter);
	}
	DriveForwardAutoEnd(drivetrain);
	ShootAutoEnd();
}
void LoadTopAuto(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake, 
		IntakeSystem *backIntake, Timer *autoTimer, ShooterSystem *shooter, 
		IterativeRobot *me)
{
	/*timer->Reset();
	timer->Start();
	
	shooter->ShooterPrime(false); 
	
	//secondaryRollers->Deploy();
	//frontIntake->DeployIntake();
	//backIntake->DeployIntake();*/
	LoadTopAutoPrep(autoTimer, shooter);
	while(LoadTopAutoConditions(autoTimer, me))//(timer->Get() < 0.8)//0.4)
	{
		/*frontIntake->Reverse();
		backIntake->Reverse();
		//secondaryRollers->Pulse();
		secondaryRollers->Run();*/
		LoadTopAutoInLoop(frontIntake, backIntake, secondaryRollers, autoTimer);
	}
	/*
	secondaryRollers->Stop();
	frontIntake->Stop();
	backIntake->Stop();
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	//OpenFlower(frontIntake, backIntake, secondaryRollers);
	//secondaryRollers->Undeploy();
	//Wait(1.0);
	Wait(1.5);*/
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
}

void LoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadBackAutoPrep(shooter, autoTimer, secondaryRollers, frontIntake);
	while(LoadBackAutoConditions(autoTimer, me)) 
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, autoTimer);
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
}

void LoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, autoTimer);
	while(LoadFrontAutoConditions(me, autoTimer)) 
	{
		LoadFrontAutoInLoop(secondaryRollers, frontIntake, autoTimer, drivetrain);
	}
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}
void LoadBackAutoDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadBackAutoPrep(shooter, autoTimer, secondaryRollers, frontIntake);
	while(LoadBackAutoConditions(autoTimer, me))
	{
		LoadBackAutoDriveInLoop(backIntake, secondaryRollers, drivetrain, autoTimer);
		
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
	drivetrain->TankDrive(0.0, 0.0);
}

void LoadBackSpin(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me,
		Encoder *leftEncoder, Encoder *rightEncoder, MPU6050_I2C *gyro, NetworkTable *table,
		bool isClockwise, int numEncoderClicks)
{
	LoadBackAutoPrep(shooter, autoTimer, secondaryRollers, frontIntake);
	
	bool started = false;
	while(LoadBackAutoConditions(autoTimer, me))
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, autoTimer);
		if(autoTimer->Get()> 0.5 &&!started)
		{
			started = true;
			SpinAutoPrep(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
		}
		if(started && SpinAutoClockConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me) && isClockwise)
		{
			SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
		}
		if(started && SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me) && !isClockwise)
		{
			SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
		}
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
	while(SpinAutoClockConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me) && isClockwise)
	{
		SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	}
	while(SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me) && !isClockwise)
	{
		SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftEncoder, rightEncoder, me);
}

void GyroTurnLoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn, double kpError, double kiError, double kdError)
{

	float integral = 0.0;
	float oldError = 0.0;
	float differential = 0.0;

	GyroTurnAnglePrep(me, gyro, drivetrain, degreeOfTurn, kpError, kiError, kdError, integral,
			differential, oldError);
	LoadBackAutoPrep(shooter, autoTimer, secondaryRollers, frontIntake);
		
	while (GyroTurnAngleConditions(gyro, degreeOfTurn, me) && LoadBackAutoConditions(autoTimer, me))
	{
		GyroTurnAngleInLoop(me, gyro, drivetrain,degreeOfTurn, kpError, kiError,
		kdError, integral, differential, oldError);
	}

	GyroTurnAngleEnd(drivetrain);
	
	while(autoTimer->Get() < 1.5)
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, autoTimer); 
	}
	
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
}

void DriveForwardAuto(RobotDrive *drivetrain, Timer *autoTimer, IterativeRobot *me, Encoder *rightEncoder)
{
	DriveForwardAutoPrep(autoTimer, rightEncoder);
	while(DriveForwardAutoConditions(autoTimer, me, rightEncoder))
	{
		DriveForwardAutoInLoop(drivetrain);
	}
	DriveForwardAutoEnd(drivetrain);
}

void ShortShootDriveForwardAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightEncoder)
{
	autoTimer->Start();
	autoTimer->Reset();

	DriveForwardAutoPrep(autoTimer, rightEncoder);
	
	bool shootPrep = false;
	
	bool doneDriving = false;
	bool doneShooting = false;
	
	while(ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder))
	{	
		if(DriveForwardShootAutoConditions(autoTimer, me, rightEncoder))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if(!shootPrep && rightEncoder->Get() <- 1500) //3 feet forward? TODO number
		{
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
	}
	DriveForwardAutoEnd(drivetrain);
	ShootAutoEnd();
}

void ShortShootTwoForwardAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *autoTimer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap, IterativeRobot *me, RobotDrive *drivetrain,
		Encoder *rightEncoder)
{
	autoTimer->Start();
	autoTimer->Reset();

	DriveForwardAutoPrep(autoTimer, rightEncoder);
	
	bool shootPrep = false;
	
	bool doneDriving = false;
	bool doneShooting = false;
	
	while(ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder))
	{	
		if(DriveForwardShootAutoConditions(autoTimer, me, rightEncoder))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if(!shootPrep && rightEncoder->Get() <- 1500) //3 feet forward? TODO number
		{
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
	}
	DriveForwardAutoEnd(drivetrain);
	ShootAutoEnd();
}
void MultiAutoLoop(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *shotTimer,
		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap,
		IterativeRobot *me, Encoder *rightEncoder, DriverStation *driverStation)
{
	/*
	 * 1. Prep
	 * 2. pulse secondarys until -500 encoder clicks then stop 
	 * pulsing if not alredy stopped pulsing && drive forward until -3300 encoder clicks
	 * then stop if not already stopped && when we hit start and reset shot timer and
	 * do shoot auto prep
	 * 3. when that is done shoot and do shoot auto end
	 * when shot timer hits 1.9, if the back intake is down then put up 
	 * secondarys and back intake, regardless drive forward slowly, front pickup and pulse secondarys
	 * 
	 * DISCLAIMER: this loop is very confusing, I probably spelled many things wrong in 
	 * this comment
	 * 
	 * --Bryton 
	 */
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false;
			
	bool backintakeup = false;
	bool stopSecondary = false;
			
	while(MultiAutoConditions(shooter, allDone, autoTimer, rightEncoder, me))
	{
		//first
		if(rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second
		if(!shootPrep && rightEncoder->Get() <- 3300)//2300) //3 feet forward? TODO number
		{
			shotTimer->Start();
			shotTimer->Reset();
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
		//third
		if(shotTimer->Get() > 1.0)//1.9) 
		{
			if(!backintakeup)
			{
				secondaryRollers->Undeploy();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			if(shotTimer->Get() < 3.0)
			{
				drivetrain->TankDrive(0.4, 0.4);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			//frontIntake->FrontRollerLoad();
			if(shotTimer->Get() > 1.5)
			{
				frontIntake->FrontPickup(driverStation);
			}
			else
			{
				//frontIntake->FrontRollerLoad();
				frontIntake->Stop();
			}
			secondaryRollers->Pulse();
		}
		
		//First
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if(shotTimer->Get() > 4.0)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			break;
		}
	}
	printf("out of loop");
}

#endif	
