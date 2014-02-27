#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

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

void GyroTurnAngle(IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn,
		double kpError, double kiError, double kdError, double minPower, double maxPower)
{
	gyro->CalibrateRate(); //DO NOT BE MOVING
	float integral = 0.0;
	Wait(0.04);
	gyro->Reset();
	printf("Gyro reset %f\n", gyro->GetCalibratedAngle());
	float differential = gyro->GetCalibratedRate();
	float oldError = degreeOfTurn-gyro->GetCalibratedAngle();
	//	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn)
	while (enabledInAutonomous(me) && TurnIncomplete(degreeOfTurn, gyro))
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

		if (degreeOfTurn > 0.0 && leftDriveTrain < minPower)
		{
			leftDriveTrain = minPower;
		}
		else if (degreeOfTurn < 0.0 && leftDriveTrain> minPower)
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
	double minPower = dataTable->GetNumber("minPower", 0);
	double maxPower = dataTable->GetNumber("maxPower", 0);
	printf("all\n");
	
	GyroTurnAngle(me, gyro, drivetrain, 
			degreeOfTurn, kpError, kiError, kdError, minPower, maxPower);
}

void DriveStraight() //use CitrusPID here. can we also use it in gyroturn?
{

}

//TODO put this in somewhere that implies not just autonomous.
void OpenFlower(IntakeSystem *frontIntake, IntakeSystem *backIntake, Solenoid *armPiston)
{
	//Unfurling the flower
	armPiston->Set(true);
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
}

void ShootAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, Solenoid *armPiston,
		IterativeRobot *me)
{
	timer->Reset();
	timer->Start();
	shooter->ShooterPrime(false);
	OpenFlower(frontIntake,backIntake, armPiston);
	shooter->BeginShooterFire();
	while(shooter->CurrentlyShooting() && enabledInAutonomous(me))
	{
		shooter->ShooterFire();
	}
}

void ShootLoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, Solenoid *armPiston,
		IterativeRobot *me)
{
	timer->Reset();
	timer->Start();

	shooter->ShooterPrime(false);
	OpenFlower(frontIntake, backIntake, armPiston);
	shooter->BeginShooterFire();

	while(shooter->CurrentlyShooting() && enabledInAutonomous(me) && timer->Get() < 2.0) //TODO time condition for intake?
	{
		shooter->ShooterFire();

		if(timer->Get() > 0.5 )
		{
			frontIntake->FrontRollerLoad();
			armPiston->Set(false);
			frontIntake->RunSecondaryRollers();
		}
	}
	frontIntake->Stop();
	frontIntake->StopSecondaryRollers();
}

void LoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, Solenoid *armPiston,
		IterativeRobot *me)
{
	shooter->ShooterPrime(true);
	armPiston->Set(false);
	timer->Reset();
	timer->Start();
	while(enabledInAutonomous(me) && timer->Get() < 2.0) //TODO end condition
	{
		backIntake->BackRollerLoad();
		backIntake->RunSecondaryRollers();
	}
	backIntake->Stop();
	backIntake->StopSecondaryRollers();
}

void aubergine(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Solenoid *armPiston, IterativeRobot *me)
{
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	//Switch to "short shot" and load.
	LoadBackAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
}

void heliotrope(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		Timer *goalTimer, Solenoid *armPiston, MPU6050_I2C *gyro, \
		IterativeRobot *me)
{
	goalTimer->Reset();
	goalTimer->Start();
	GyroTurnAngle(me, gyro, drivetrain,-10.0, 2.0, 0.02, 0.65, 0.0, 1.0); 
	//TODO incorporate hot goal stuff.
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	GyroTurnAngle(me, gyro, drivetrain, 20.0, 2.0, 0.02, 0.65, 0.0, 1.0);
	while(enabledInAutonomous(me))
	{
		if(goalTimer->Get() > 4.5)
		{
			break;
		}
	}
	ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);

	LoadBackAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
}

#endif	
