#include "WPILib.h"
#include "IntakeSystem.h"
#include "MPU6050_I2C.h"
#include "CitrusPID.h"
#include "NetworkTables/NetworkTable.h"
#include "SecondaryRollerSystem.h"
#include "AutonomousComponents.h"


#ifndef AUTONOMOUSSUBROUTINES_H
#define AUTONOMOUSSUBROUTINES_H

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


void ShootAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers);
	while(ShootAutoConditions(shooter, me))
	{
		ShootAutoInLoop(shooter);
	}
	ShootAutoEnd();
}

void ShootLoadFrontAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, RobotDrive *drivetrain)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers);
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) //TODO time condition for intake?
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoInLoop(secondaryRollers, frontIntake, timer, drivetrain);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake);
}

void ShootLoadFrontAutoDrive(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, RobotDrive *drivetrain)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers);
	LoadFrontAutoPrep(frontIntake, backIntake, secondaryRollers, timer);

	while(ShootAutoConditions(shooter, me) || LoadFrontAutoConditions(me, timer)) //TODO time condition for intake?
	{
		ShootAutoInLoop(shooter);
		LoadFrontAutoDriveInLoop(secondaryRollers, frontIntake, timer, drivetrain);
	}
	ShootAutoEnd();
	LoadFrontAutoEnd(secondaryRollers, frontIntake);
}

void ShootDriveForwardAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		IterativeRobot *me, RobotDrive *drivetrain)
{
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers);
	bool driveInit = false;
	timer->Start();
	timer->Reset();
	
	while(ShootAutoConditions(shooter, me) || DriveForwardAutoConditions(timer, me))
	{
		if(!driveInit && timer->Get() > 1.0)
		{
			DriveForwardAutoPrep(timer);
			driveInit = true;
		}
		if(driveInit)
		{
			DriveForwardAutoInLoop(drivetrain);
		}
		ShootAutoInLoop(shooter);
	}
	DriveForwardAutoEnd(drivetrain);
	ShootAutoEnd();
}
 
void LoadBackAuto(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, Timer *timer, SecondaryRollerSystem *secondaryRollers,
		RobotDrive *drivetrain, IterativeRobot *me)
{
	LoadBackAutoPrep(shooter, timer, secondaryRollers, frontIntake);
	while(LoadBackAutoConditions(timer, me)) //TODO end condition
	{
		LoadBackAutoDriveInLoop(backIntake, secondaryRollers, drivetrain, timer);
	}
	LoadBackAutoEnd(backIntake, secondaryRollers, shooter);
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
	LoadBackAutoEnd(backIntake, secondaryRollers, shooter);
	drivetrain->TankDrive(0.0, 0.0);
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
	
	LoadBackAutoEnd(backIntake, secondaryRollers, shooter);
}

void DriveForwardAuto(RobotDrive *drivetrain, Timer *timer, IterativeRobot *me)
{
	DriveForwardAutoPrep(timer);
	while(DriveForwardAutoConditions(timer, me))
	{
		DriveForwardAutoInLoop(drivetrain);
	}
	DriveForwardAutoEnd(drivetrain);
}

#endif	
