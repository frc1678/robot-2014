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
			drivetrain->TankDrive(-0.5, 0.5); //TODO direction?
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

void SpinAutoClock(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	/*leftDT->Reset();
	rightDT->Reset();
	while (EnabledInAutonomous(me) && (leftDT->Get()>(0-numEncoderClicks)&& rightDT->Get()<(numEncoderClicks)))
	{
		drivetrain->TankDrive(-0.75, 0.75);
	}
	drivetrain->TankDrive(0.0, 0.0);*/
	SpinAutoPrep(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	while(SpinAutoClockConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me))
	{
		SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftDT, rightDT, me);
}
	//Spin counterclockwise for (approximately) so many encoder clicks. Anti is shorter than counter. Un is shorter than Anti.
void SpinAutoAnti(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
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

	SpinAutoPrep(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	while(SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me))
	{
		SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftDT, rightDT, me);
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

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) //TODO time condition for intake?
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoInLoop(secondaryRollers, frontIntake, timer, drivetrain);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}

void ShootLoadFrontAutoDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,
		IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightDT)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) //TODO time condition for intake?
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoDriveInLoop(secondaryRollers, frontIntake, timer, drivetrain, rightDT);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}

void ShootAutoLoadBack(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,
		IterativeRobot *me, RobotDrive *drivetrain)
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
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,
		IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightDT)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
	bool driveInit = false;
	timer->Start();
	timer->Reset();
	
	while(ShootAutoConditions(shooter, me) || DriveForwardAutoConditions(timer, me, rightDT))
	{
		if(!driveInit && timer->Get() > 1.0)
		{
			DriveForwardAutoPrep(timer, rightDT);
			driveInit = true;
			backIntake->UndeployIntake();
		}
		if(driveInit && DriveForwardAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		ShootAutoInLoop(shooter);
	}
	DriveForwardAutoEnd(drivetrain);
	ShootAutoEnd();
}
void LoadTopAuto(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake, IntakeSystem *backIntake, Timer *timer, ShooterSystem *shooter, IterativeRobot *me)
{
	/*timer->Reset();
	timer->Start();
	
	shooter->ShooterPrime(false); 
	
	//secondaryRollers->Deploy();
	//frontIntake->DeployIntake();
	//backIntake->DeployIntake();*/
	LoadTopAutoPrep(timer, shooter);
	while(LoadTopAutoConditions(timer, me))//(timer->Get() < 0.8)//0.4)
	{
		/*frontIntake->Reverse();
		backIntake->Reverse();
		//secondaryRollers->Pulse();
		secondaryRollers->Run();*/
		LoadTopAutoInLoop(frontIntake, backIntake, secondaryRollers);
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
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadBackAutoPrep(shooter, timer, secondaryRollers, frontIntake);
	while(LoadBackAutoConditions(timer, me)) //TODO end condition
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, timer);
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
}

void LoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);
	while(LoadFrontAutoConditions(me, timer)) //TODO end condition
	{
		LoadFrontAutoInLoop(secondaryRollers, frontIntake, timer, drivetrain);
	}
	LoadFrontAutoEnd(secondaryRollers, frontIntake, drivetrain);
}
void LoadBackAutoDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadBackAutoPrep(shooter, timer, secondaryRollers, frontIntake);
	while(LoadBackAutoConditions(timer, me)) //TODO end condition
	{
		LoadBackAutoDriveInLoop(backIntake, secondaryRollers, drivetrain, timer);
		
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
	drivetrain->TankDrive(0.0, 0.0);
}

void LoadBackSpin(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me,
		Encoder *leftDT, Encoder *rightDT, MPU6050_I2C *gyro, NetworkTable *table,
		bool isClockwise, int numEncoderClicks)
{
	LoadBackAutoPrep(shooter, timer, secondaryRollers, frontIntake);
	
	bool started = false;
	while(LoadBackAutoConditions(timer, me))
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, timer);
		if(timer->Get()> 0.5 &&!started)
		{
			started = true;
			SpinAutoPrep(numEncoderClicks, drivetrain, leftDT, rightDT, me);
		}
		if(started && SpinAutoClockConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me) && isClockwise)
		{
			SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
		}
		if(started && SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me) && !isClockwise)
		{
			SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
		}
	}
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
	while(SpinAutoClockConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me) && isClockwise)
	{
		SpinAutoClockInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	}
	while(SpinAutoAntiConditions(numEncoderClicks, drivetrain, leftDT, rightDT, me) && !isClockwise)
	{
		SpinAutoAntiInLoop(numEncoderClicks, drivetrain, leftDT, rightDT, me);
	}
	SpinAutoEnd(numEncoderClicks, drivetrain, leftDT, rightDT, me);
}

void GyroTurnLoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn, double kpError, double kiError, double kdError)
{

	float integral = 0.0;
	float oldError = 0.0;
	float differential = 0.0;

	GyroTurnAnglePrep(me, gyro, drivetrain, degreeOfTurn, kpError, kiError, kdError, integral,
			differential, oldError);
	LoadBackAutoPrep(shooter, timer, secondaryRollers, frontIntake);
		
	while (GyroTurnAngleConditions(gyro, degreeOfTurn, me) && LoadBackAutoConditions(timer, me))
	{
		GyroTurnAngleInLoop(me, gyro, drivetrain,degreeOfTurn, kpError, kiError,
		kdError, integral, differential, oldError);
	}

	GyroTurnAngleEnd(drivetrain);
	
	while(timer->Get() < 1.5)
	{
		LoadBackAutoInLoop(backIntake, secondaryRollers, drivetrain, timer); 
	}
	
	LoadBackAutoEnd(backIntake, frontIntake, secondaryRollers, shooter);
}

void DriveForwardAuto(RobotDrive *drivetrain, Timer *timer, IterativeRobot *me, Encoder *rightDT)
{
	DriveForwardAutoPrep(timer, rightDT);
	while(DriveForwardAutoConditions(timer, me, rightDT))
	{
		DriveForwardAutoInLoop(drivetrain);
	}
	DriveForwardAutoEnd(drivetrain);
}

void ShortShootDriveForwardAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,
		IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightDT)
{
	timer->Start();
	timer->Reset();

	DriveForwardAutoPrep(timer, rightDT);
	
	bool shootPrep = false;
	
	bool doneDriving = false;
	bool doneShooting = false;
	
	while(ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(timer, me, rightDT))
	{	
		if(DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if(!shootPrep && rightDT->Get() <- 1500) //3 feet forward? TODO number
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
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		Solenoid *spitShortSwap,
		IterativeRobot *me, RobotDrive *drivetrain, Encoder *rightDT)
{
	timer->Start();
	timer->Reset();

	DriveForwardAutoPrep(timer, rightDT);
	
	bool shootPrep = false;
	
	bool doneDriving = false;
	bool doneShooting = false;
	
	while(ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(timer, me, rightDT))
	{	
		if(DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if(!shootPrep && rightDT->Get() <- 1500) //3 feet forward? TODO number
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

#endif	
