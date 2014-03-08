#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"
#include "SecondaryRollerSystem.h"

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

bool EnabledInAutonomous(IterativeRobot *me)
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
	//float oldError = degreeOfTurn+gyro->GetCalibratedAngle();
	float oldError = degreeOfTurn-gyro->GetCalibratedAngle();
	//	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn)
	while (EnabledInAutonomous(me) && TurnIncomplete(degreeOfTurn, gyro))
	{
		float angle = gyro->GetCalibratedAngle();
		float aError = degreeOfTurn - angle;
		//float aError = degreeOfTurn + angle;
		integral += aError;
		differential = gyro->GetCalibratedRate();
		float leftDriveTrain = kpError*(aError)/100; // multiply by constant
		leftDriveTrain += kiError*integral/100;
		printf("left drive train1: %f	", leftDriveTrain);
		leftDriveTrain -= kdError*differential/100;
		printf("left drive train2: %f\n", leftDriveTrain);
		oldError = aError;
		printf("differential: %f\n", differential);
/*
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
		*/
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
	
void DriveStraight(RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT,
		IterativeRobot *me) //use CitrusPID here. can we also use it in gyroturn?
{
	leftDT->Reset();
	rightDT->Reset();
	leftDT->Start();
	rightDT->Start();
	while(leftDT->Get() < 300 && rightDT->Get() > -300 && EnabledInAutonomous(me))
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

void ShootAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me)
{
	OpenFlower(frontIntake,backIntake, secondaryRollers);
	shooter->ShooterPrime(false);
	Wait(0.5); //TODO how short can this be?
	timer->Reset();
	timer->Start();
	shooter->BeginShooterFire();
	while(shooter->CurrentlyShooting() && EnabledInAutonomous(me))
	{
		shooter->ShooterFire();
	}
}

void ShootLoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me)
{

	OpenFlower(frontIntake,backIntake, secondaryRollers);
	shooter->ShooterPrime(false);
	Wait(0.5); //TODO how short can this be?
	timer->Reset();
	timer->Start();
	
	shooter->BeginShooterFire();

	while(shooter->CurrentlyShooting() && EnabledInAutonomous(me) && timer->Get() < 1.4+1.0) //TODO time condition for intake?
	{
		shooter->ShooterFire();

		if(timer->Get() > 1.0 )
		{
			frontIntake->FrontRollerLoad();
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
		}
	}
	frontIntake->Stop();
	secondaryRollers->Stop();
}

void LoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me)
{
	shooter->ShooterPrime(true);
	secondaryRollers->Undeploy();
	timer->Reset();
	timer->Start();
	while(EnabledInAutonomous(me) && timer->Get() < 2.0) //TODO end condition
	{
		backIntake->BackRollerLoad();
		secondaryRollers->Run();
	}
	backIntake->Stop();
	secondaryRollers->Stop();
}

void GyroTurnLoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, MPU6050_I2C *gyro, RobotDrive *drivetrain,
		float degreeOfTurn,
		double kpError, double kiError, double kdError, double minPower, double maxPower)
{
	gyro->CalibrateRate(); //DO NOT BE MOVING
	float integral = 0.0;
	shooter->ShooterPrime(true);
	secondaryRollers->Undeploy();
	timer->Reset();
	timer->Start();
	Wait(0.04);
	gyro->Reset();
	printf("Gyro reset %f\n", gyro->GetCalibratedAngle());
	float differential = gyro->GetCalibratedRate();
	//float oldError = degreeOfTurn+gyro->GetCalibratedAngle();
	float oldError = degreeOfTurn-gyro->GetCalibratedAngle();
	//	while(amethyst(me) && gyro->GetAngle()<degreeOfTurn)
	bool intakeUpped = false;
	while (EnabledInAutonomous(me) && TurnIncomplete(degreeOfTurn, gyro))
	{
		float angle = gyro->GetCalibratedAngle();
		float aError = degreeOfTurn - angle;
		//float aError = degreeOfTurn + angle;
		integral += aError;
		differential = gyro->GetCalibratedRate();
		float leftDriveTrain = kpError*(aError)/100; // multiply by constant
		leftDriveTrain += kiError*integral/100;
		leftDriveTrain -= kdError*differential/100;
		oldError = aError;

		//Drivetrain
		drivetrain->TankDrive(leftDriveTrain, -leftDriveTrain);

		//Loading.
		backIntake->BackRollerLoad();
		secondaryRollers->Run();
		/*if(timer->Get() > 1.0 && !intakeUpped)
		{
			backIntake->UndeployIntake();
			intakeUpped = true;
		}*/
	}

	drivetrain->TankDrive(0.0,0.0);	

	while(timer->Get() < 1.5)
	{
		//Loading.
		backIntake->BackRollerLoad();
		secondaryRollers->Run();
		/*if(timer->Get() > 1.0 && !intakeUpped)
		{
			backIntake->UndeployIntake();
			intakeUpped = true;
		}*/ 
	}
	backIntake->Stop();
	secondaryRollers->Stop();
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

void ShootThreeAndDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *timer,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me)
{
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	//Switch to "short shot" and load.
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	//Shoot again.
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
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
		GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585, 0.0, 1.0);//TODO constents 
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55, 0.0, 1.0);//TODO constents 
	}
	//TODO incorporate hot goal stuff.
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	if(autoDirection == 1.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, 20.0, 2.0, 0.14, 0.55, 0.0, 1.0);//TODO constents
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnAngle(me, gyro, drivetrain, -20.0, 2.0, 0.15, 0.585, 0.0, 1.0);//TODO constents
	}
	while(EnabledInAutonomous(me))
	{
		if(goalTimer->Get() > 4.5)
		{
			break;
		}
	}
	
	LoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
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
		GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585, 0.0, 1.0);//TODO constents 
		//GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55, 0.0, 1.0);//TODO constents 
	}
	else
	{
		printf("RIGHT!\n");
		GyroTurnAngle(me, gyro, drivetrain, 10.0, 2.0, 0.345, 0.55, 0.0, 1.0);//TODO constents 
		//GyroTurnAngle(me, gyro, drivetrain, -10.0, 2.0, 0.327, 0.585, 0.0, 1.0);//TODO constents 
	}
	//TODO incorporate hot goal stuff.
	ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	
	if(autoDirection == 1.0)
	{
		GyroTurnLoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, 
				me, gyro, drivetrain, -20.0, 2.0, 0.15, 0.585, 0.0, 1.0);//TODO constents
	}
	else if(autoDirection == 2.0)
	{
		GyroTurnLoadBackAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, 
				me, gyro, drivetrain, 20.0, 2.0, 0.14, 0.55, 0.0, 1.0);//TODO constents
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
	GyroTurnAngle(me, gyro, drivetrain,-10.0, 2.0, 0.04, 0.65, 0.0, 1.0); 
	//TODO incorporate hot goal stuff.
	//ShootLoadFrontAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	//ShootAuto(frontIntake, backIntake, shooter, timer, armPiston, me);
	
	GyroTurnAngle(me, gyro, drivetrain, 20.0, 2.0, 0.04, 0.65, 0.0, 1.0);
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
