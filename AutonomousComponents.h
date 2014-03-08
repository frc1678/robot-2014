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
	if(TurnIncomplete(degreeOfTurn, gyro) && EnabledInAutonomous(me))
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
	//gyro->Stop();
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

void ShootAutoPrep(IntakeSystem *frontIntake, IntakeSystem *backIntake, 
		ShooterSystem *shooter, SecondaryRollerSystem *secondaryRollers, bool shortShot)
{
	//OpenFlower(frontIntake, backIntake, secondaryRollers);
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	
	frontIntake->FrontRollerAutoSlow();
	backIntake->BackRollerAutoSlow();
	shooter->ShooterPrime(shortShot);
	//Wait(0.3); //TODO how short can this be?
	
	secondaryRollers->Deploy();
	shooter->BeginShooterFire();
}

bool ShootAutoConditions(ShooterSystem *shooter, IterativeRobot *me)
{
	if(shooter->CurrentlyShooting() && EnabledInAutonomous(me))
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
	if(EnabledInAutonomous(me) && timer->Get() < 1.8)//1.4)
	{
		return true;
	}
	return false;
}

void LoadBackAutoDriveInLoop(IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers, RobotDrive *drivetrain, Timer *timer)
{
	backIntake->BackRollerLoad();
	secondaryRollers->Pulse();
	//secondaryRollers->Run();
	if(timer->Get() < 0.25)
	{
		drivetrain->TankDrive(-0.5, -0.5);
	}
}

void LoadBackAutoInLoop(IntakeSystem *backIntake, SecondaryRollerSystem *secondaryRollers, RobotDrive *drivetrain, Timer *timer)
{
	backIntake->BackRollerLoad();
	secondaryRollers->Pulse();
	//secondaryRollers->Run();
	//backIntake->Pickup();
	/*if(timer->Get() > 0.8 && backIntake->intakeDeployed)
	{
		backIntake->UndeployIntake();
	}*/
}

void LoadBackAutoEnd(IntakeSystem *backIntake, IntakeSystem *frontIntake, SecondaryRollerSystem *secondaryRollers, ShooterSystem *shooter)
{
	backIntake->Stop();
	secondaryRollers->Stop();
	shooter->ShooterPrime(false);
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	//OpenFlower(frontIntake, backIntake, secondaryRollers);
	Wait(1.0);//(0.5);
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
	if(EnabledInAutonomous(me) && timer->Get() < 1.4+1.0)
	{
		return true;
	}
	return false;
}

void LoadFrontAutoDriveInLoopV2(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake,
		Timer *timer, RobotDrive *drivetrain, Encoder *rightDT)
{
	frontIntake->FrontRollerLoad();
	if(timer->Get() > 1.5 ) //0.75 and 1.0 for quick fire??
	{
		
		//secondaryRollers->Pulse();
		secondaryRollers->Run();
		secondaryRollers->Undeploy();
		
	}
	if(timer->Get() < 3.5 && timer->Get()>1.0 && rightDT->Get() < 3000) //number?
	{
		//drivetrain->TankDrive(0.5, 0.5);
		//drivetrain->TankDrive(-0.5, -0.5);
		drivetrain->TankDrive(-1.0,-1.0);
	}
	else
	{
		drivetrain->TankDrive(0.0, 0.0);
	}
}

void LoadFrontAutoDriveInLoop(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake,
		Timer *timer, RobotDrive *drivetrain, Encoder *rightDT)
{
	frontIntake->FrontRollerLoad();
	if(timer->Get() > 1.5 ) //0.75 and 1.0 for quick cat
	{
		
		//secondaryRollers->Pulse();
		secondaryRollers->Run();
		secondaryRollers->Undeploy();
		if(timer->Get() < 3.5 && timer->Get()>1.0 && rightDT->Get() < 3000)
		{
			//drivetrain->TankDrive(0.5, 0.5);
			drivetrain->TankDrive(-0.5, -0.5);
			//drivetrain->TankDrive(-1.0,-1.0);
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
		//secondaryRollers->Pulse();
		secondaryRollers->Run();
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

void LoadFrontAutoEnd(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake, RobotDrive *drivetrain)
{
	secondaryRollers->Stop();
	frontIntake->Stop();
	drivetrain->TankDrive(0.0, 0.0);
}

void LoadTopAuto(SecondaryRollerSystem *secondaryRollers, IntakeSystem *frontIntake, IntakeSystem *backIntake, Timer *timer, ShooterSystem *shooter)
{
	timer->Reset();
	timer->Start();
	
	shooter->ShooterPrime(false); 
	
	//secondaryRollers->Deploy();
	//frontIntake->DeployIntake();
	//backIntake->DeployIntake();
	while(timer->Get() < 0.8)//0.4)
	{
		frontIntake->Reverse();
		backIntake->Reverse();
		//secondaryRollers->Pulse();
		secondaryRollers->Run();
	}
	secondaryRollers->Stop();
	frontIntake->Stop();
	backIntake->Stop();
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	//OpenFlower(frontIntake, backIntake, secondaryRollers);
	//secondaryRollers->Undeploy();
	//Wait(1.0);
	Wait(1.5);
}


void DriveForwardAutoPrep(Timer *timer, Encoder *rightDT)
{
	timer->Start();
	timer->Reset();
	rightDT->Reset();
	rightDT->Start();
}

bool DriveForwardAutoConditions(Timer *timer, IterativeRobot *me, Encoder *rightDT)
{
	if(timer->Get() < 2.0 && EnabledInAutonomous(me) && rightDT->Get() > - 1000)//3000)
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

void SpinAutoPrep(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	leftDT->Reset();
	rightDT->Reset();
}

bool SpinAutoClockConditions(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	if(EnabledInAutonomous(me) && (leftDT->Get()>(0-numEncoderClicks)&& rightDT->Get()<(numEncoderClicks)))
	{
		return true;
	}
	return false;
}

bool SpinAutoAntiConditions(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	if (EnabledInAutonomous(me) && (leftDT->Get()<(numEncoderClicks)
			&& rightDT->Get()>(-numEncoderClicks)))
	{
		return true;
	}
	return false;
}

void SpinAutoClockInLoop(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	drivetrain->TankDrive(-0.75, 0.75);
}

void SpinAutoAntiInLoop(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	drivetrain->TankDrive(0.75, -0.75);
}

void SpinAutoEnd(int numEncoderClicks, RobotDrive *drivetrain, Encoder *leftDT, Encoder *rightDT, IterativeRobot *me)
{
	drivetrain->TankDrive(0.0, 0.0);
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
		autoDirection = 0.0; //Testing.
	}
	
	return autoDirection;
}

#endif	
