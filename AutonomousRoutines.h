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

//Two shots, one short and one long
void TwoShotShortLong(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	spitShortSwap->Set(false);  //Shortest fingers down
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //suck the ball in from the top
	//ShootAutoLoadBack(frontIntake, backIntake, shooter, timer, secondaryRollers, me, drivetrain);
	Wait(1.5);
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me); //Shoot the ball
	spitShortSwap->Set(true); //Shortest fingers up
	LoadBackAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me); //load the ball in from the back
	spitShortSwap->Set(true); //shortest fingers up
	Wait(1.0);
	//drive forward and shoots a short shot when closer
	ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me, drivetrain, rightEncoder);
}

//two shots, both shorter shots
void TwoShotShortShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, 
		DriverStation *driverStation)
{
	spitShortSwap->Set(true); //shortest fingers up
	frontIntake->FrontRollerAutoSlow(); //slowly roll balls against the bumpers
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks the ball in down from the top
	
	TwoShotShortPrep(shooter, timer2); //Set everything for the shot
	
	//drives, pulse secondaries and shoots
	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
			driverStation); //driving and stuff and shoots once
	//sets the intakes down
	frontIntake->DeployIntake(); 
	backIntake->DeployIntake(); 
	Wait(1.0);
	//shoots a short shot
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
	timer2->Stop();
	/*ShortShootDriveForwardAuto(frontIntake, backIntake, shooter, timer, secondaryRolles, spitShortSwap, me, drivetrain, rightEncoder);
	LoadFrontAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	Wait(1.0);
	ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, spitShortSwap, me);*/
}

//Drive and shoot one short shot in auto
void OneShotShort(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, DriverStationLCD *driverStationLCD,
		NetworkTable *table, float startSide)
{
	spitShortSwap->Set(true); //shortest fingers up
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //suck down the ball
	Wait(0.5);
	table->PutNumber("Enabled", 1); //send to network table that robot is on
	Wait(1.0);
	float visionInput = ReceiveVisionProcessing(table); //Tabble direction returned
	driverStationLCD->PrintfLine((DriverStationLCD::Line)0,"Vision: %f", visionInput);
	driverStationLCD->UpdateLCD();
	shooter->ShooterPrime(true); //longest fingers down
	
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false; //for continuing the while loop
	
	bool backintakeup = false; //unused
	
	timer2->Reset();
	bool stopSecondary = false;
	if(visionInput == 0.0)
	{
		Wait(2.0); //hot is in front
	}
	else if(visionInput != startSide) //both are 1.0 for left and 2.0 for right
	{
		Wait(3.5);//(4.0); //wait for hot
	}
	
	rightEncoder->Reset();
	//leftEncoder->Reset();
	
	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
	{
		//first
		if(rightEncoder->Get() > -500) //for the short distance, settle the ball
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary) //preset to false as to not run more than once
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second
		if(!shootPrep && rightEncoder->Get() <- 2300) 
		{
			timer2->Start();
			timer2->Reset();
			//Shortest fingers up, longest down
			ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if(shootPrep && ShootAutoConditions(shooter, me)) //if a certain point has been reached, and aren't already shooting in auto
		{
			ShootAutoInLoop(shooter); //shoot the ball
		}
		else if(shootPrep && !doneShooting)
		{
			ShootAutoEnd(); //does... nothing
		}
		//first. Just driving forward to a point
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain); //Driving at almost full speed
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain); //sets motors to stop
			doneDriving = true;
			break;
		}
	}
	//bring up the intakes 
	frontIntake->UndeployIntake();
	backIntake->UndeployIntake();
	autoTimer->Stop(); //stop so nothing breaks
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
	backIntake->DeployIntake(); //set intakes down
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter);
	while (LoadTopAutoConditions(autoTimer, me)) //check time since prep
	{

		backIntake->BackRollerAutoSlow(); //roll ball to hold against bumpers
		frontIntake->FrontRollerAutoSlow();
		secondaryRollers->Run(); //suck the ball down to settle
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake); //sets intakes down
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
	shooter->ShooterPrime(true); //longest fingers down
	//backIntake->DeployIntake();
	backIntake->BackRollerAutoSlow(); //roll ball to hold against bumpers
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
					rightEncoder) || !allDone) 
	{
		//first. Pulse to a certain point
		if (rightEncoder->Get() > -1000) 
		{
			secondaryRollers->Pulse();
		} 
		else if (!stopSecondary) 
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}

		//Added this- remove for return to known good code.
		if(!shootPrep && rightEncoder->Get() < -1550) //past a certain point
		{
			backIntake->BackRollerAutoSlow(); //roll to hold ball against bumpers
			//backIntake->ReverseSlow();
		}
		else if(!shootPrep)
		{
			backIntake->BackRollerAutoSlow(); //continually roll against bumpers
		}
		
		//second
		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
		{
			//get ready to shoot. Start/reset everything
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter,
					secondaryRollers, spitShortSwap, true);
			shootPrep = true; //runs loop once
		}
		if (shootPrep && ShootAutoConditions(shooter, me)) 
		{
			ShootAutoInLoop(shooter); //shoots
			printf("Shot");
		}
		else if (shootPrep && !doneShooting) 
		{
			ShootAutoEnd(); //nothing
		}
		//third
		if (shotTimer->Get() > 1.4) //past a point
		{
			if (!backintakeup) 
			{
				secondaryRollers->Undeploy(); //bring in rollers
				//frontIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive();
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad(); //load from the back
			printf("load");
			
			secondaryRollers->Pulse(); //settle the ball
		}
		else if(shotTimer->Get() < 0.7 && shotTimer->Get() > 0.3)  //between a ceratin point
		{
			drivetrain->TankDrive(-0.6, 0.7); //turn
		}
		

		//First. Drive forward 
		else if (rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
				drivetrain->TankDrive(-0.9, -0.9);
		}
		else if (!doneDriving) 
		{
			DriveForwardAutoEnd(drivetrain); //stop motors
			doneDriving = true;
		}
		if (shotTimer->Get() > 3.0)//4.2)
		{
			printf("Shot timer > 3.0");
			secondaryRollers->Stop(); //stop rollers
			DriveForwardAutoEnd(drivetrain); //stop motors
			allDone = true;
			break;
		}

	}
	drivetrain->TankDrive(0.0, 0.0);
	frontIntake->DeployIntake(); //set intakes down
	backIntake->DeployIntake();
	Wait(0.5);
	//get ready before the shoot
	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
			spitShortSwap, true);
	autoTimer->Stop();
	autoTimer->Reset();
	autoTimer->Start();
	backintakeup = false;
	bool shotDone = false;
	//enabled, plus timer is under a point, and/or we are ready to shoot
	while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
			< 3.0) && EnabledInAutonomous(me)) 
	{
		if (ShootAutoConditions(shooter, me)) 
		{
			ShootAutoInLoop(shooter); //take the shoot
		} 
		else if (!shotDone) 
		{
			ShootAutoEnd();
			shotDone = false;
		}
		if (autoTimer->Get() > 0.2) //past a specific point
		{
			if (!backintakeup) 
			
			{
				//bring intakes up
				secondaryRollers->Undeploy();
				//frontIntake->UndeployIntake();
				frontIntake->DeployIntake();
				backIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive(0.4, 0.4);
			//frontIntake->FrontRollerLoad();
			if (autoTimer->Get() < 0.5) 
			
				drivetrain->TankDrive(0.8, 0.4); //turn right
			}
			else if (autoTimer->Get() < 0.9) 
			{
				drivetrain->TankDrive(-0.4, -0.8); //moreso to the right
			}
			else {
				drivetrain->TankDrive(0.0, 0.0); //stop motors
			}

			if (autoTimer->Get() < 0.9)
			{
				frontIntake->FrontRollerLoad(); //load another ball while turning
			}
			//frontIntake->FrontPickup(driverStation);
			else if (autoTimer->Get() > 0.9)
			{
				if (autoTimer->Get() > 1.1)
				{
					frontIntake->UndeployIntake(); //bring intake back up
				}
				//frontIntake->ReverseSlow();
				backIntake->ReverseSlow();
				frontIntake->Stop();
			}

			secondaryRollers->Pulse(); //settle the ball
		}
	
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake); //bring everything in and stop 
	Wait(0.3);
	//short shot
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
			secondaryRollers, spitShortSwap, me);
	autoTimer->Stop();
}

//IO 8 (is crazy right now)
void ThreeShotGoalieLeftRight(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap)
{
	backIntake->DeployIntake(); //set intakes down
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter); //largest fingers up
	while (LoadTopAutoConditions(autoTimer, me))
	{
		//rolls to hold the ball against the bumpers
		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		secondaryRollers->Run(); //run to suck down the ball
	}
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake); //stop everything
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
	shooter->ShooterPrime(true); //longest fingers down
	//backIntake->DeployIntake();
	backIntake->BackRollerAutoSlow(); //rolls to hold balls against bumpers
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
					rightEncoder) || !allDone)
	{
		//first. Until a certain point, pulse rollers
		if (rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}

		//Added this- remove for return to known good code.
		if(!shootPrep && rightEncoder->Get() < -1550) //past a certain point 
		{
			backIntake->BackRollerAutoSlow(); //roll the ball against bimpers
			//backIntake->ReverseSlow();
		}
		else if(!shootPrep)
		{
			backIntake->BackRollerAutoSlow(); //just continually roll against bumpers
		}
		
		//second. Past a ceratin point, prep for the shot
		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
		{ //once we reach -1750 then prepair to shoot
			shotTimer->Start();
			shotTimer->Reset();
			//prep for the shot
			ShootAutoPrep(frontIntake, backIntake, shooter,
					secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if (shootPrep && ShootAutoConditions(shooter, me)) //check time since prep 
		{
			ShootAutoInLoop(shooter); //take the shot
			printf("Shot");
		}
		else if (shootPrep && !doneShooting)
		{
			ShootAutoEnd();
		}
		//third
		if (shotTimer->Get() > 1.4)
		{
			if (!backintakeup)
			{
				secondaryRollers->Undeploy(); //bring rollers in
				//frontIntake->UndeployIntake();
				backintakeup = true;
			}
			//drivetrain->TankDrive();
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad(); //roller the ball against bumpers
			printf("load");
			
			secondaryRollers->Pulse(); //pulse to settle
		}
		else if(shotTimer->Get() < 0.9 && shotTimer->Get() > 0.5) 
		{
			drivetrain->TankDrive(-0.7, 0.8); // turn
		}
		

		//First
		else if (rightEncoder->Get() > -13000)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			if(rightEncoder->Get() > -80)
			{
				drivetrain->TankDrive(0.0, -0.8); //turn to the right
			}
			else
			{
				//DriveForwardAutoInLoop(drivetrain);
				drivetrain->TankDrive(-0.9, -0.9); //drive forward
			}
		}
		else if (!doneDriving)
		{
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

//3 short shots
void ThreeShotShort1(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain,DriverStation *driverStation, Timer *autoTimer, Timer *timer2,
		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap, IterativeRobot *me, 
		Encoder *rightEncoder, NetworkTable *table, float startSide)
{
	//table->PutNumber("Enabled", 1);
	spitShortSwap->Set(true); //shortest fingers up
	printf("secondary pistons set true\n");
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //load the ball of rollers
	printf("Loaded from top\n");
	//float visionInput = ReceiveVisionProcessing(table);
	shooter->ShooterPrime(true); //longest fingers down
	printf("Shooter primed\n");
	Wait(1.0);
	printf("Past wait");
		
	TwoShotShortPrep(shooter, timer2); 

	//shopts once and drives forward
	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
			driverStation);
	printf("Hi! WATCH ME ACTUALLY WORK FOR THREE BALL!\n");
	frontIntake->DeployIntake(); //set intakes down
	backIntake->DeployIntake();
	autoTimer->Stop();
	//Load ball from the back while driving
	LoadBackAutoDrive(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
	//Wait(1.0);
	//shoot the abll once loaded
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
}

//two shots taken
void TwoShot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer,
		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap, IterativeRobot *me, Encoder *rightEncoder)
{
	spitShortSwap->Set(false); //shortest fingers down 
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //suck ball down off rollers
	
	Wait(1.5);
	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me); //shoots the ball
	//LoadBackAutoDrive(frontIntake, backIntake, shooter, timer, secondaryRollers, drivetrain, me);
	spitShortSwap->Set(true); //shortest fingers up
	//load the ball from the back
	LoadBackAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
	printf("Done driving!");
	spitShortSwap->Set(false); //shortest fingers set down
	Wait(1.0); 
	//ShootAuto(frontIntake, backIntake, shooter, timer, secondaryRollers, me);
	//shoot while driving forward. Ends with intakes down
	ShootDriveForwardAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me, drivetrain, rightEncoder);
}

void TwoShotShortVision(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder,Encoder *leftEncoder,
		DriverStation *driverStation,float direction, NetworkTable *table)
{	
		spitShortSwap->Set(true); //shortest fingers up 
		//frontIntake->FrontRollerAutoSlow();
		//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me);
		LoadTopAutoPrep(autoTimer, shooter); //timers and long fingers up
		while(LoadTopAutoConditions(autoTimer, me))//(timer->Get() < 0.8)//0.4) //before a ceratin point
		{
			/*frontIntake->Reverse();
			backIntake->Reverse();
			//secondaryRollers->Pulse();
			secondaryRollers->Run();*/
			if(autoTimer->Get() > 0.4)
			{
				table->PutNumber("Enabled", 1); //send number to network tables so vision will run
			}
			LoadTopAutoInLoop(frontIntake, backIntake, secondaryRollers, autoTimer); //suck down the ball 
		}
		LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake); //intakes down

		//timer reset, and long fingers down
		TwoShotShortPrep(shooter, timer2); 

		autoTimer->Start();
		autoTimer->Reset();
		secondaryRollers->Stop();
		Wait(0.4);
		float visionInput = ReceiveVisionProcessing(table); //should say which direction to go
		if(visionInput == 1.0) //go left
		{
			while(autoTimer->Get() < 0.5)
			{
				drivetrain->TankDrive(-0.8, 0.8);
			}
		}
		else if(visionInput == 2.0) //go right
		{
			while(autoTimer->Get() < 0.5)
			{
				drivetrain->TankDrive(0.8, -0.8);
			}	
		}
		else //just go right
		{
			while(autoTimer->Get() < 0.5)
			{
				drivetrain->TankDrive(0.8, -0.8);
			}	
		}
		autoTimer->Stop();
		
		//drive forward and shoots a ball
		MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
				autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
				driverStation); //driving and stuff and shoots once
		frontIntake->DeployIntake();//sets intakes down
		backIntake->DeployIntake(); 
		Wait(0.5);
		
		ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
		
		autoTimer->Stop();
		autoTimer->Reset();
}

void TwoShotHot(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, DriverStationLCD *driverStationLCD,
		NetworkTable *table, float startSide, Timer *shotTimer) 

{
	table->PutNumber("Enabled", 1); //sends to network tables that robot is enabled
	spitShortSwap->Set(true); //shortest fingers up 
	//frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
	
	Wait(1.0);
	float visionInput = ReceiveVisionProcessing(table); //check to see what direction we should go
	driverStationLCD->PrintfLine((DriverStationLCD::Line)0,"Vision: %f", visionInput); //print said value to driverstation
	driverStationLCD->UpdateLCD();
	shooter->ShooterPrime(true);
	
	//preset so things only run once
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false;
	
	bool backintakeup = false; //unused
	
	timer2->Reset();
	bool stopSecondary = false;
	if(visionInput == 0.0)
	{
		Wait(0.6); //hot is in front of you
	}
	else if(visionInput != startSide) //both are 1.0 for left and 2.0 for right
	{
		Wait(4.0); //hot will soon be in front of you
	}
	
	rightEncoder->Reset();
	//leftEncoder->Reset();
	
	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
	{
		//first. Just drving while pulsing
		if(rightEncoder->Get() > -500)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second. Over a certaion point, prep for shot
		if(!shootPrep && rightEncoder->Get() <- 2300) 
		{
			timer2->Start();
			timer2->Reset();
			ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
			shootPrep = true;
		}
		if(shootPrep && ShootAutoConditions(shooter, me))
		{
			ShootAutoInLoop(shooter); //take the shot
			Wait(0.3);
			//load another ball in from the back
			LoadBackAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
		}
		//check again for next shot
		else if(shootPrep && ShootAutoConditions(shooter, me))
		{
			//prep shot
			ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
			Wait(0.3);
			ShootAutoInLoop(shooter); //take shot
		}
		else if(shootPrep && !doneShooting)
		{
			ShootAutoEnd();
		}
		//first
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			DriveForwardAutoInLoop(drivetrain); //drive forward to a point
			backIntake->ReverseSlow(); //slowly roll away
		}
		else if(!shootPrep)
		{
			if(!shootPrep && rightEncoder->Get() > -1000 && shotTimer->Get() < 1.4)
			{
				backIntake->RunAt(0.45); //load the ball
			}
			else if(!shootPrep && rightEncoder->Get() > -2800)
			{
				backIntake->RunAt(0.4);
			}
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
			break;
		}
	}
	frontIntake->UndeployIntake(); //bring intakes in
	backIntake->UndeployIntake();
	autoTimer->Stop();
	timer2->Stop();

}

void FunAuto(RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder) //Bryton, how will they tell how long they should turn for...
{
	while(leftEncoder->Get() >= -1000)
	{
		drivetrain->TankDrive(-0.3,-0.3); //Drive forward
		printf("in fun auto");
		if (leftEncoder -> Get() <= -400 && leftEncoder -> Get() >= -800)
		{
			drivetrain->TankDrive(0.3,-0.3); //Here's where you should turn
		}
	}
	drivetrain->TankDrive(0.0, 0.0);
}

void PIDDriveStraight(CitrusPID *PID, RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder)
{ //Currently, this is just a basic outline of what the PID should be,
	//There may be more math involved and the Coefficients will be tuned
	float pError = 0.0;
	float iError = 0.0;
	float dError = 0.0;
	float pCoefficient = 0.0;
	float iCoefficient = 0.0;
	float dCoefficient = 0.0;
	int targetEncoderClicks = 100;
	float leftDriveTrainInput = 0.0;
	float rightDriveTrainInput = 0.0;
	float correction = 0.0;
	
	while(((leftEncoder->Get() + rightEncoder->Get()) / 2) <= targetEncoderClicks)
	{
		//proportional error, the difference between the two encoders
		pError = (leftEncoder->Get() - rightEncoder->Get()) * pCoefficient;
		
		//integral error
		//each time you re-multiply by the coefficient, do we want to do this?
		iError += pError;
		iError *= iCoefficient;
		
		//Derivitive error
		dError = (leftEncoder->GetRate() - rightEncoder->GetRate()) * dCoefficient;
		
		//putting things together
		correction = pError + iError + dError;
		
		leftDriveTrainInput = 0.75 + correction;
		rightDriveTrainInput = 0.75 - correction;
		
		drivetrain->TankDrive(-leftDriveTrainInput, -rightDriveTrainInput); //negative so that it goes forward.  
	}
	
	
}

#endif	
