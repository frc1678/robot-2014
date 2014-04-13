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

//TODO Reference
void OneShotShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, DriverStationLCD *driverStationLCD,
		NetworkTable *table, float startSide)
{
	spitShortSwap->Set(true);
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	table->PutNumber("Enabled", 1);
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
	
	rightEncoder->Reset();
	//leftEncoder->Reset();
	
	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
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
}*/ //IO ONE
void ThreeShotGoalie3(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap)
{
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter);
	while (LoadTopAutoConditions(autoTimer, me)) {

		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		secondaryRollers->Run();
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
	shooter->ShooterPrime(true);
	//backIntake->DeployIntake();
	backIntake->BackRollerAutoSlow();
	frontIntake->FrontRollerAutoSlow();
	//Wait(1.0);
	bool shootPrep = false;
	bool doneShooting = false;
	bool stopSecondary = false;
	bool backintakeup = false;
	allDone = false;
	bool doneDriving = false;
	int startPos = 0;
	bool startedSecondTurn = false;
	//while((rightEncoder->Get() > - 3300 || !doneShooting) && IsAutonomous())
	while (ShootAutoConditions(shooter, me)
			|| DriveForwardShootAutoConditions(autoTimer, me,
					rightEncoder) || !allDone) {
		//first
		if (rightEncoder->Get() > -1000) {
			secondaryRollers->Pulse();
		} else if (!stopSecondary) {
			stopSecondary = true;
			secondaryRollers->Stop();
		}

		//Added this- remove for return to known good code.
		if(!shootPrep && rightEncoder->Get() < -1550)
		{
			backIntake->BackRollerAutoSlow();
			//backIntake->ReverseSlow();
		}
		else if(!shootPrep)
		{
			backIntake->BackRollerAutoSlow();
		}
		
		//second
		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
		{
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter,
					secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if (shootPrep && ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
			printf("Shot");
		} else if (shootPrep && !doneShooting) {
			ShootAutoEnd();
		}
		//third
		if (shotTimer->Get() > 1.4) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive();
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad();
			printf("load");
			
			secondaryRollers->Pulse();
		}
		else if(shotTimer->Get() < 0.7 && shotTimer->Get() > 0.3) 
		{
			drivetrain->TankDrive(-0.6, 0.7); //Second turn
		}
		

		//First
		else if (rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
				drivetrain->TankDrive(-0.9, -0.9);
		} else if (!doneDriving) {
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if (shotTimer->Get() > 3.0)//4.2)
		{
			printf("Shot timer > 3.0");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			break;
		}

	}
	drivetrain->TankDrive(0.0, 0.0);
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	Wait(0.5);
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
			spitShortSwap, true);
	autoTimer->Stop();
	autoTimer->Reset();
	autoTimer->Start();
	backintakeup = false;
	bool shotDone = false;
	while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
			< 3.0) && EnabledInAutonomous(me)) {
		if (ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
		} else if (!shotDone) {
			ShootAutoEnd();
			shotDone = false;
		}
		if (autoTimer->Get() > 0.2) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				frontIntake->DeployIntake();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive(0.4, 0.4);
			//frontIntake->FrontRollerLoad();
			if (autoTimer->Get() < 0.5) {
				drivetrain->TankDrive(0.8, 0.4);
			} else if (autoTimer->Get() < 0.9) {
				drivetrain->TankDrive(-0.4, -0.8);
			} else {
				drivetrain->TankDrive(0.0, 0.0);
			}

			if (autoTimer->Get() < 0.9) {
				frontIntake->FrontRollerLoad();
			}
			//frontIntake->FrontPickup(driverStation);
			else if (autoTimer->Get() > 0.9) {
				if (autoTimer->Get() > 1.1) {
					frontIntake->UndeployIntake();
				}
				//frontIntake->ReverseSlow();
				backIntake->ReverseSlow();
				frontIntake->Stop();
			}

			secondaryRollers->Pulse();
		}
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	Wait(0.3);
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
			secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
}

//IO 8 (is crazy right now)
void ThreeShotGoalieLeftRight(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap)
{
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter);
	while (LoadTopAutoConditions(autoTimer, me)) {

		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		secondaryRollers->Run();
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
	shooter->ShooterPrime(true);
	//backIntake->DeployIntake();
	backIntake->BackRollerAutoSlow();
	frontIntake->FrontRollerAutoSlow();
	//Wait(1.0);
	bool shootPrep = false;
	bool doneShooting = false;
	bool stopSecondary = false;
	bool backintakeup = false;
	allDone = false;
	bool doneDriving = false;
	int startPos = 0;
	bool startedSecondTurn = false;
	//while((rightEncoder->Get() > - 3300 || !doneShooting) && IsAutonomous())
	while (ShootAutoConditions(shooter, me)
			|| DriveForwardShootAutoConditions(autoTimer, me,
					rightEncoder) || !allDone) {
		//first
		if (rightEncoder->Get() > -1000) {
			secondaryRollers->Pulse();
		} else if (!stopSecondary) {
			stopSecondary = true;
			secondaryRollers->Stop();
		}

		//Added this- remove for return to known good code.
		if(!shootPrep && rightEncoder->Get() < -1550)
		{
			backIntake->BackRollerAutoSlow();
			//backIntake->ReverseSlow();
		}
		else if(!shootPrep)
		{
			backIntake->BackRollerAutoSlow();
		}
		
		//second
		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
		{
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter,
					secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if (shootPrep && ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
			printf("Shot");
		} else if (shootPrep && !doneShooting) {
			ShootAutoEnd();
		}
		//third
		if (shotTimer->Get() > 1.4) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive();
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad();
			printf("load");
			
			secondaryRollers->Pulse();
		}
		else if(shotTimer->Get() < 0.9 && shotTimer->Get() > 0.5) 
		{
			drivetrain->TankDrive(-0.7, 0.8); //Second turn
		}
		

		//First
		else if (rightEncoder->Get() > -13000)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			if(rightEncoder->Get() > -80)
			{
				drivetrain->TankDrive(0.0, -0.8);
			}
			else
			{
				//DriveForwardAutoInLoop(drivetrain);
				drivetrain->TankDrive(-0.9, -0.9);
			}
		} else if (!doneDriving) {
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if (shotTimer->Get() > 3.0)//4.2)
		{
			printf("Shot timer > 3.0");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			break;
		}

	}
	drivetrain->TankDrive(0.0, 0.0);
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	Wait(0.5);
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
			spitShortSwap, true);
	autoTimer->Stop();
	autoTimer->Reset();
	autoTimer->Start();
	backintakeup = false;
	bool shotDone = false;
	while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
			< 3.0) && EnabledInAutonomous(me)) {
		if (ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
		} else if (!shotDone) {
			ShootAutoEnd();
			shotDone = false;
		}
		if (autoTimer->Get() > 0.2) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				frontIntake->DeployIntake();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive(0.4, 0.4);
			//frontIntake->FrontRollerLoad();
			if (autoTimer->Get() < 0.5) {
				drivetrain->TankDrive(0.7, 0.9);
			} else if (autoTimer->Get() < 0.9) {
				drivetrain->TankDrive(-0.9, -0.7);
			} else {
				drivetrain->TankDrive(0.0, 0.0);
			}

			if (autoTimer->Get() < 0.9) {
				frontIntake->FrontRollerLoad();
			}
			//frontIntake->FrontPickup(driverStation);
			else if (autoTimer->Get() > 0.9) {
				if (autoTimer->Get() > 1.1) {
					frontIntake->UndeployIntake();
				}
				//frontIntake->ReverseSlow();
				backIntake->ReverseSlow();
				frontIntake->Stop();
			}

			secondaryRollers->Pulse();
		}
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	Wait(0.3);
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
			secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
}

//IO 3
void ThreeShotGoalieRightLeft(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap)
{
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter);
	while (LoadTopAutoConditions(autoTimer, me)) {

		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		secondaryRollers->Run();
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
	shooter->ShooterPrime(true);
	//backIntake->DeployIntake();
	backIntake->BackRollerAutoSlow();
	frontIntake->FrontRollerAutoSlow();
	//Wait(1.0);
	bool shootPrep = false;
	bool doneShooting = false;
	bool stopSecondary = false;
	bool backintakeup = false;
	allDone = false;
	bool doneDriving = false;
	int startPos = 0;
	bool startedSecondTurn = false;
	//while((rightEncoder->Get() > - 3300 || !doneShooting) && IsAutonomous())
	while (ShootAutoConditions(shooter, me)
			|| DriveForwardShootAutoConditions(autoTimer, me,
					rightEncoder) || !allDone) {
		//first
		if (rightEncoder->Get() > -1000) {
			secondaryRollers->Pulse();
		} else if (!stopSecondary) {
			stopSecondary = true;
			secondaryRollers->Stop();
		}

		//Added this- remove for return to known good code.
		if(!shootPrep && rightEncoder->Get() < -1550)
		{
			backIntake->BackRollerAutoSlow();
			//backIntake->ReverseSlow();
		}
		else if(!shootPrep)
		{
			backIntake->BackRollerAutoSlow();
		}
		
		//second
		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
		{
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter,
					secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if (shootPrep && ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
			printf("Shot");
		} else if (shootPrep && !doneShooting) {
			ShootAutoEnd();
		}
		//third
		if (shotTimer->Get() > 1.4) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive();
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad();
			printf("load");
			
			secondaryRollers->Pulse();
		}
		else if(shotTimer->Get() < 0.7 && shotTimer->Get() > 0.3) 
		{
			drivetrain->TankDrive(0.7, -0.7); //Second turn
		}
		

		//First
		else if (rightEncoder->Get() > -3340)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			if(rightEncoder->Get() > -35)
			{
				drivetrain->TankDrive(-0.8, 0.0);
			}
			else
			{
				//DriveForwardAutoInLoop(drivetrain);
				drivetrain->TankDrive(-0.8, -0.8);
			}
		} else if (!doneDriving) {
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}
		if (shotTimer->Get() > 3.0)//4.2)
		{
			printf("Shot timer > 3.0");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			break;
		}

	}
	drivetrain->TankDrive(0.0, 0.0);
	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	Wait(0.5);
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
			spitShortSwap, true);
	autoTimer->Stop();
	autoTimer->Reset();
	autoTimer->Start();
	backintakeup = false;
	bool shotDone = false;
	while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
			< 3.0) && EnabledInAutonomous(me)) {
		if (ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
		} else if (!shotDone) {
			ShootAutoEnd();
			shotDone = false;
		}
		if (autoTimer->Get() > 0.2) {
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				frontIntake->DeployIntake();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive(0.4, 0.4);
			//frontIntake->FrontRollerLoad();
			if (autoTimer->Get() < 0.5) {
				drivetrain->TankDrive(0.8, 0.4);
			} else if (autoTimer->Get() < 0.9) {
				drivetrain->TankDrive(-0.4, -0.8);
			} else {
				drivetrain->TankDrive(0.0, 0.0);
			}

			if (autoTimer->Get() < 0.9) {
				frontIntake->FrontRollerLoad();
			}
			//frontIntake->FrontPickup(driverStation);
			else if (autoTimer->Get() > 0.9) {
				if (autoTimer->Get() > 1.1) {
					frontIntake->UndeployIntake();
				}
				//frontIntake->ReverseSlow();
				backIntake->ReverseSlow();
				frontIntake->Stop();
			}

			secondaryRollers->Pulse();
		}
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	Wait(0.3);
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
			secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
}

void ThreeShotShort1(IntakeSystem *frontIntake, IntakeSystem *backIntake,
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

void TwoShotShortVision(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder,Encoder *leftEncoder,
		DriverStation *driverStation,float direction)
{
	spitShortSwap->Set(true);
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	
	TwoShotShortPrep(shooter, timer2); 
	while((direction == 1.0 && rightEncoder->Get() > -35) || 
			(direction == 2.0 && leftEncoder->Get() > -35))
	{
		if(direction == 1.0)
		{
			drivetrain->TankDrive(0.0, -8.0);
		}	
		else if(direction == 2.0)
		{
			drivetrain->TankDrive(-8.0, 0.0);
		}
		else
		{
			printf("Vision Error, float direction is not 1.0 or 2.0");
			drivetrain->TankDrive(0.0, -8.0);
		}
	}
	drivetrain->TankDrive(0.0, 0.0);
	
	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
			driverStation);

	TwoShotShortPrep(shooter, timer2); 
	while((direction == 1.0 && rightEncoder->Get() > -105) || 
			(direction == 2.0 && leftEncoder->Get() > -105))
	{
		if(direction == 1.0)
		{
			drivetrain->TankDrive(0.0, 8.0);
		}	
		else if(direction == 2.0)
		{
			drivetrain->TankDrive(8.0, 0.0);
		}
		else
		{
			printf("Vision Error, float direction is not 1.0 or 2.0");
			drivetrain->TankDrive(0.0, 8.0);
		}
	}
	drivetrain->TankDrive(0.0, 0.0);
	
	frontIntake->DeployIntake();
	backIntake->DeployIntake(); 
	Wait(1.0);
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	
	while((direction == 1.0 && rightEncoder->Get() < -35) || 
				(direction == 2.0 && leftEncoder->Get() < -35))
	{
		if(direction == 1.0)
		{
			drivetrain->TankDrive(0.0, -8.0);
		}	
		else if(direction == 2.0)
		{
			drivetrain->TankDrive(-8.0, 0.0);
		}
		else
		{
			printf("Vision Error, float direction is not 1.0 or 2.0");
			drivetrain->TankDrive(0.0, -8.0);
		}
	}
	autoTimer->Stop();
	timer2->Stop();
	autoTimer->Start();
	autoTimer->Reset();
	while(autoTimer->Get() < 1.0)
	{
		drivetrain->TankDrive(-1.0, -1.0);
	}
	
	//autoTimer->Stop();
	//timer2->Stop();
	/*ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRolles, spitShortSwap, me, drivetrain, rightEncoder);
	LoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, spitShortSwap, me);*/
}

/*void TwoShotHot() TODO TODO Finish this!!!
{
	spitShortSwap->Set(true);
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
	table->PutNumber("Enabled", 1);
	Wait(1.0);
	float visionInput = ReceiveVisionProcessing(table);
	driverStationLCD->PrintfLine((DriverStationLCD::Line)0,"Vision: %f", visionInput);
	driverStationLCD->UpdateLCD();
	shooter->ShooterPrime(true);
	
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false;
	
	timer2->Reset();
	bool stopSecondary = false;
	if(visionInput == 0.0)
	{
		Wait(0.3);
	}
	else if(visionInput != startSide) //both are 1.0 for left and 2.0 for right
	{
		Wait(4.0);
	}
	
	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
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
__________________________________________________________
	
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
			spitShortSwap, true);
	autoTimer->Stop();
	autoTimer->Reset();
	autoTimer->Start();
	backintakeup = false;
	bool shotDone = false;
	while ((ShootAutoConditions(shooter, this) || autoTimer->Get()
			< 2.8) && IsAutonomous()) 
	{
		if (ShootAutoConditions(shooter, this)) 
		{
			ShootAutoInLoop(shooter);
		} 
		else if (!shotDone) 
		{
			ShootAutoEnd();
			shotDone = false;
		}
		if (autoTimer->Get() > 0.2) 
		{
			if (!backintakeup) 
			{
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				frontIntake->DeployIntake();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive(0.4, 0.4);
			//frontIntake->FrontRollerLoad();
			if (autoTimer->Get() < 0.5) 
			{
				drivetrain->TankDrive(0.7, 0.7);
			}
			else if (autoTimer->Get() < 0.9) 
			{
				drivetrain->TankDrive(-0.7, -0.7);
			} 
			else 
			{
				drivetrain->TankDrive(0.0, 0.0);
			}

			if (autoTimer->Get() < 0.9) 
			{
				frontIntake->FrontRollerLoad();
			}
			//frontIntake->FrontPickup(driverStation);
			else if (autoTimer->Get() > 0.9) 
			{
				if (autoTimer->Get() > 1.1) 
				{
					frontIntake->UndeployIntake();
				}
				//frontIntake->ReverseSlow();
				backIntake->ReverseSlow();
				frontIntake->Stop();
			}

			secondaryRollers->Pulse();
		}
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	Wait(0.3);
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
			secondaryRollers, spitShortSwap, this);
	autoTimer->Stop();
}
*/


#endif	
