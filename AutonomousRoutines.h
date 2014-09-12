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



///////////////////////////
//
//	AUTO NUMBER ONE
//
///////////////////////////

/*
 * OVERVIEW
 * Starts on one side of the field or the other, center of one of the goals.
 * Drives forward, and turns to the opposite goal, firing a long shot there
 * Turn in an arc and curve back to the orginial goal, firing again
 */
void LeftRightShot (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, 
		RobotDrive *drivetrain, DriverStation *driverStation, bool randomNum, Solenoid *spitShortSwap)
{
	//preset things so they only run once
		bool shootPrep = false;
		bool doneDriving = false;
		bool doneShooting = false;
		
		allDone = false;
		bool allDone2 = false;
		
		bool stopSecondary = false;
		bool backintakeup = false;

		spitShortSwap->Set(true); //shortest fingers up 
		frontIntake->FrontRollerAutoSlow();
		LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
		shooter->ShooterPrime(true); // Prep for short shot

		
		autoTimer->Reset();
		shotTimer->Reset();
		rightEncoder->Reset();


		while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
		{


			//first. Just drving while pulsing
			if(rightEncoder->Get() > -1000)
			{
				secondaryRollers->Pulse();
			}
			else if (!stopSecondary)
			{
				stopSecondary = true;
				secondaryRollers->Stop();
			}


			
			//second. Over a certaion point, prep for shot
			if(!shootPrep && rightEncoder->Get() < -3300) 
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



			//THIRD
			if(shotTimer->Get() > 1.0)//1.9) 
			{
				if(!backintakeup)
				{
					backIntake->Stop();
					secondaryRollers->Undeploy();
					backIntake->UndeployIntake();
					backintakeup = true;
				}


				if(shotTimer->Get() > 1.5)
				{
					frontIntake->FrontPickup(driverStation);
				}
				else
				{
					//frontIntake->FrontRollerLoad();
					frontIntake->Stop();
				}


				if(shotTimer->Get() < 3.0)
				{
					drivetrain->TankDrive(0.1, 0.1);
				}
				else
				{
					drivetrain->TankDrive(0.0, 0.0);
				}
				
				//frontIntake->FrontRollerLoad();
				
				secondaryRollers->Pulse();
			}
			
			//FIRST. Driving
			else if(leftEncoder->Get() > -45 || rightEncoder->Get() > -45) {
								
				drivetrain->TankDrive(0.0, -0.6);	
				
			}
			else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
			{
				drivetrain->TankDrive(-0.8, -0.8); // Drive slightly to the right to start.
			}
			else if(!doneDriving)
			{
				DriveForwardAutoEnd(drivetrain);
				doneDriving = true;
			}


			if(shotTimer->Get() > 3.7)//3.0)//4.2)
			{
				printf("Shot timer > 4.2");
				secondaryRollers->Stop();
				DriveForwardAutoEnd(drivetrain);
				allDone = true;
				autoTimer->Start();
				autoTimer->Reset();
				break;
			}
		}

			//------------------------------------------------------



		frontIntake->DeployIntake();
		backIntake->DeployIntake();

		while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone2) && EnabledInAutonomous(me)){
			
			if (autoTimer->Get() < 0.2) { //TODO Time

				drivetrain->TankDrive(0.0, 0.0);
			}
			else if (autoTimer->Get() < 0.5) {
				drivetrain->TankDrive(-0.65, 0.65);
			}
//			else if (autoTimer->Get() < 1.2) {
//				drivetrain->TankDrive(0.0, 0.0);
//			}
//			else if (autoTimer->Get() < 1.6) {
//				drivetrain->TankDrive(0.75, -0.75);
//			}
			else {
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			if (autoTimer->Get() > 2.0){
				
				autoTimer->Stop();
				shotTimer->Stop();
				allDone2 = true;
			}
		}
		
		ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
		
	}


///////////////////////////
//
//	AUTO NUMBER TWO
//
///////////////////////////

/*
 * OVERVIEW
 * Starts on one side of the field or the other, center of one of the goals.
 * Drives forward, and turns to the opposite goal, firing a long shot there
 * Turn in an arc and curve back to the orginial goal, firing again
 */
void RightLeftShot (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers, 
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer,
		RobotDrive *drivetrain, Solenoid *spitShortSwap, DriverStation *driverStation, bool randomNum)
{
	//preset things so they only run once
		bool shootPrep = false;
		bool doneDriving = false;
		bool doneShooting = false;
		
		allDone = false;
		bool allDone2 = false;
		
		bool stopSecondary = false;
		bool backintakeup = false;

		spitShortSwap->Set(true); //shortest fingers up 
		frontIntake->FrontRollerAutoSlow();
		LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
		shooter->ShooterPrime(true); // Prep for short shot

		
		autoTimer->Reset();
		shotTimer->Reset();
		rightEncoder->Reset();


		while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
		{


			//first. Just drving while pulsing
			if(rightEncoder->Get() > -1000)
			{
				secondaryRollers->Pulse();
			}
			else if (!stopSecondary)
			{
				stopSecondary = true;
				secondaryRollers->Stop();
			}


			
			//second. Over a certaion point, prep for shot
			if(!shootPrep && rightEncoder->Get() < -3300) 
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



			//THIRD
			if(shotTimer->Get() > 1.0)//1.9) 
			{
				if(!backintakeup)
				{
					backIntake->Stop();
					secondaryRollers->Undeploy();
					backIntake->UndeployIntake();
					backintakeup = true;
				}


				if(shotTimer->Get() > 1.5)
				{
					frontIntake->FrontPickup(driverStation);
				}
				else
				{
					//frontIntake->FrontRollerLoad();
					frontIntake->Stop();
				}


				if(shotTimer->Get() < 3.0)
				{
					drivetrain->TankDrive(0.1, 0.1);
				}
				else
				{
					drivetrain->TankDrive(0.0, 0.0);
				}
				
				//frontIntake->FrontRollerLoad();
				
				secondaryRollers->Pulse();
			}
			
			//FIRST. Driving
			else if(leftEncoder->Get() > -45 || rightEncoder->Get() > -45) {
				
				drivetrain->TankDrive(-0.6, 0.0);
			}
			else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
			{
				drivetrain->TankDrive(-0.8, -0.8); // Drive slightly to the right to start.
			}
			else if(!doneDriving)
			{
				DriveForwardAutoEnd(drivetrain);
				doneDriving = true;
			}


			if(shotTimer->Get() > 3.7)//3.0)//4.2)
			{
				printf("Shot timer > 4.2");
				secondaryRollers->Stop();
				DriveForwardAutoEnd(drivetrain);
				allDone = true;
				autoTimer->Start();
				autoTimer->Reset();
				break;
			}
		}

			//------------------------------------------------------



		frontIntake->DeployIntake();
		backIntake->DeployIntake();

		while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone2) && EnabledInAutonomous(me)){
			
			if (autoTimer->Get() < 0.2) { //TODO Time

				drivetrain->TankDrive(0.0, 0.0);
			}
			else if (autoTimer->Get() < 0.5) {
				drivetrain->TankDrive(0.7, -0.7);
			}
//			else if (autoTimer->Get() < 1.2) {
//				drivetrain->TankDrive(0.0, 0.0);
//			}
//			else if (autoTimer->Get() < 1.6) {
//				drivetrain->TankDrive((0.75 * -k), (0.75 * k));
//			}
			else {
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			if (autoTimer->Get() > 2.0){
				
				autoTimer->Stop();
				shotTimer->Stop();
				allDone2 = true;
			}
		}
		
		ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
		
	}


///////////////////////////
//
//	AUTO NUMBER THREE
//
///////////////////////////

/*
 * OVERVIEW
 * Start on one side of the field. 
 * Drive straight foward
 * Turn at the last second toward one side of the goal
 * Shoot a short shot
 * Load a ball from the front
 * Turn to the other side of the goal 
 * Shoot a short shot once more
 */

void LeftRightLeftShot (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers, 
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer,
		RobotDrive *drivetrain, Solenoid *spitShortSwap, DriverStation *driverStation, bool randomNum)
{
	//preset things so they only run once
		bool shootPrep = false;
		bool doneDriving = false;
		bool doneShooting = false;
		
		allDone = false;
		bool allDone2 = false;
		
		bool stopSecondary = false;
		bool backintakeup = false;

		spitShortSwap->Set(true); //shortest fingers up 
		frontIntake->FrontRollerAutoSlow();
		LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
		shooter->ShooterPrime(true); // Prep for short shot

		
		autoTimer->Reset();
		shotTimer->Reset();
		rightEncoder->Reset();


		while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
		{


			//first. Just drving while pulsing
			if(rightEncoder->Get() > -1000)
			{
				secondaryRollers->Pulse();
			}
			else if (!stopSecondary)
			{
				stopSecondary = true;
				secondaryRollers->Stop();
			}


			
			//second. Over a certaion point, prep for shot
			if(!shootPrep && rightEncoder->Get() < -3300) 
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



			//THIRD
			if(shotTimer->Get() > 1.0)//1.9) 
			{
				if(!backintakeup)
				{
					backIntake->Stop();
					secondaryRollers->Undeploy();
					backIntake->UndeployIntake();
					backintakeup = true;
				}


				if(shotTimer->Get() > 1.5)
				{
					frontIntake->FrontPickup(driverStation);
				}
				else
				{
					//frontIntake->FrontRollerLoad();
					frontIntake->Stop();
				}


				if(shotTimer->Get() < 3.0)
				{
					drivetrain->TankDrive(0.1, 0.1);
				}
				else
				{
					drivetrain->TankDrive(0.0, 0.0);
				}
				
				//frontIntake->FrontRollerLoad(); 
				
				secondaryRollers->Pulse();
			}
			
			//FIRST. Driving
			else if((leftEncoder->Get() > -45 && randomNum) || rightEncoder->Get() > -45) {
										
				drivetrain->TankDrive(0.0, -0.6);	
			
			}
			else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
			{
				drivetrain->TankDrive(-0.8, -0.8); // Drive slightly to the right to start.
			}
			else if(!doneDriving)
			{
				DriveForwardAutoEnd(drivetrain);
				doneDriving = true;
			}


			if(shotTimer->Get() > 3.7)//3.0)//4.2)
			{
				printf("Shot timer > 4.2");
				secondaryRollers->Stop();
				DriveForwardAutoEnd(drivetrain);
				allDone = true;
				autoTimer->Start();
				autoTimer->Reset();
				break;
			}
		}

			//------------------------------------------------------



		frontIntake->DeployIntake();
		backIntake->DeployIntake();

		while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone2) && EnabledInAutonomous(me)){
			
			if (autoTimer->Get() < 0.2) { //TODO Time

				drivetrain->TankDrive(0.0, 0.0);
			}
			else if (autoTimer->Get() < 0.6) {
				drivetrain->TankDrive(-0.75, 0.75);
			}
			else if (autoTimer->Get() < 1.0) {
				drivetrain->TankDrive(0.2, 0.2);
			}
			else if (autoTimer->Get() < 1.3) {
				drivetrain->TankDrive(0.75, -0.75);
			}
			else if (autoTimer->Get() < 1.6) {
				drivetrain->TankDrive(0.4, 0.6);
			}
			else {
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			if (autoTimer->Get() > 2.0){
				
				autoTimer->Stop();
				shotTimer->Stop();
				allDone2 = true;
			}
		}
		
		ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
		
	}


///////////////////////////
//
//	AUTO NUMBER FOUR
//
///////////////////////////

/*
 * OVERVIEW
 * Take in vision processing
 * See which goal is hot
 * Drive forward
 * Shoot one ball into the hot goal
 */

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
			drivetrain->TankDrive(-0.85, -0.85);
			//DriveForwardAutoInLoop(drivetrain); //Driving at almost full speed
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



///////////////////////////
//
//	AUTO NUMBER FOUR
//
///////////////////////////

/*
 * OVERVIEW
 * Take in vision processing
 * See which goal is hot
 * Drive forward
 * Shoot one ball into the hot goal
 */

void DriveForward (IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder, DriverStationLCD *driverStationLCD,
		NetworkTable *table, float startSide)
{
	//spitShortSwap->Set(true); //shortest fingers up
	//frontIntake->FrontRollerAutoSlow();
	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //suck down the ball
	Wait(0.5);
	table->PutNumber("Enabled", 1); //send to network table that robot is on
	Wait(1.0);
	float visionInput = ReceiveVisionProcessing(table); //Tabble direction returned
	driverStationLCD->PrintfLine((DriverStationLCD::Line)0,"Vision: %f", visionInput);
	driverStationLCD->UpdateLCD();
	//shooter->ShooterPrime(true); //longest fingers down
	
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	bool allDone = false; //for continuing the while loop
	
	
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
//		if(rightEncoder->Get() > -500) //for the short distance, settle the ball
//		{
//			secondaryRollers->Pulse();
//		}
//		else if (!stopSecondary) //preset to false as to not run more than once
//		{
//			stopSecondary = true;
//			secondaryRollers->Stop();
//		}
		
		//second
//		if(!shootPrep && rightEncoder->Get() <- 2300) 
//		{
//			timer2->Start();
//			timer2->Reset();
//			//Shortest fingers up, longest down
//			ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
//			shootPrep = true;
//		}
		
//		if(shootPrep && ShootAutoConditions(shooter, me)) //if a certain point has been reached, and aren't already shooting in auto
//		{
//			ShootAutoInLoop(shooter); //shoot the ball
//		}
//		else if(shootPrep && !doneShooting)
//		{
//			ShootAutoEnd(); //does... nothing
//		}
		
		
		//first. Just driving forward to a point
		if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightDT))
		{
			drivetrain->TankDrive(-0.85, -0.85);
			//DriveForwardAutoInLoop(drivetrain); //Driving at almost full speed
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




///////////////////////////
//
//	AUTO NUMBER FIVE
//
///////////////////////////

/*
 * OVERVIEW
 * Same as auto routine 2, but with vision
 * Start in the center of the field
 * Deploy intakes and prep to load top
 * Load the top ball down 
 * Prime shooter and play with variables
 * Shoot (long?)
 * Load a ball from the front
 * Shoot   (do driving during most of the routine)
 * Load a ball from the back
 * Shoot one last time
 */

void ThreeBallVision (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, 
		RobotDrive *drivetrain, DriverStation *driverStation, Solenoid *spitShortSwap, 
		NetworkTable *table, DriverStationLCD *driverStationLCD)
{
	table->PutNumber("Enabled", 1);
	backIntake->DeployIntake();
	frontIntake->DeployIntake();
	LoadTopAutoPrep(autoTimer, shooter);
	int encoderStartValue = 0;
	while (LoadTopAutoConditions(autoTimer, me)) {

		secondaryRollers->Run();secondaryRollers->Run();
		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		
	}
	
	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
	
	float direction = ReceiveVisionProcessing(table); // Figure out which goal is hot
	// FOR DIRECTION. 1.0 is left, 2.0 is right. 0.0 means there was an error
	if(direction == 0.0)
	{
		driverStationLCD->Printf((DriverStationLCD::Line)3, 1, "VISION ERROR");
		direction = 1.0;
	}
	
	shooter->ShooterPrime(true);
	backIntake->BackRollerAutoSlow();
	frontIntake->FrontRollerAutoSlow();
	bool shootPrep	   		= false;
	bool doneShooting  		= false;
	bool stopSecondary 		= false;
	bool backintakeup  		= false;
	allDone 		  		= false;
	bool doneDriving  		= false;
	bool startedSecondTurn  = false;
	bool finishedSecondTurn = false;
	
	
	while (ShootAutoConditions(shooter, me)
			|| DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) 
			|| !allDone) 
	{
		//Print information to the driverstation
		driverStationLCD->Printf((DriverStationLCD::Line) 0, 1, "Left: %d", leftEncoder->Get());
		driverStationLCD->Printf((DriverStationLCD::Line) 1, 1, "Right: %d", rightEncoder->Get());
		driverStationLCD->Printf((DriverStationLCD::Line) 2, 1, "Start Value: %d", encoderStartValue);
		
		//first
		
		if(direction == 2.0)
		{
			if (rightEncoder->Get() > -1000) 
			{
				secondaryRollers->Pulse();
			} 
			else if (!stopSecondary) 
			{
				stopSecondary = true;
				secondaryRollers->Stop();
			}
		}
		
		if(direction == 1.0)
		{
			if (leftEncoder->Get() > -1000)
			{
				secondaryRollers->Pulse();
			} 
			else if (!stopSecondary) 
			{
				stopSecondary = true;
				secondaryRollers->Stop();
			}
		}

		if (direction == 2.0)
		{
			if(!shootPrep && rightEncoder->Get() < -1900)
			{
				backIntake->ReverseSlow();
			}
			else if(!shootPrep)
			{
				backIntake->BackRollerAutoSlow();
			}
		}
		
		if(direction == 1.0)
		{
			if(!shootPrep && leftEncoder->Get() < -1900)
			{
				backIntake->ReverseSlow();
			}
			else if(!shootPrep)
			{
				backIntake->BackRollerAutoSlow();
			}
		}
		
		//second
		if(direction == 1.0)
		{
			if (!shootPrep && leftEncoder->Get() < -2000) //3 feet forward?
			{
				shotTimer->Start();
				shotTimer->Reset();
				ShootAutoPrep(frontIntake, backIntake, shooter,
						secondaryRollers, spitShortSwap, true);
				shootPrep = true;
			}
		}
		if(direction == 2.0)
		{
			if (!shootPrep && rightEncoder->Get() < -2000) //3 feet forward?
			{
				shotTimer->Start();
				shotTimer->Reset();
				ShootAutoPrep(frontIntake, backIntake, shooter,
						secondaryRollers, spitShortSwap, true);
				shootPrep = true;
			}
		}
		
		if (shootPrep && ShootAutoConditions(shooter, me)) {
			ShootAutoInLoop(shooter);
			printf("Shot");
		} 
		else if (shootPrep && !doneShooting) {
			ShootAutoEnd();
		}
		//third
						
		if (shotTimer->Get() > 1.4) {
			driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "  NOT!");
			if (!backintakeup) {
				secondaryRollers->Undeploy();
				backintakeup = true;
			}
			
			if(shotTimer->Get() < 3.0)
			{
				// Drive forward a little more
				drivetrain->TankDrive(-0.6, -0.6);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			//frontIntake->FrontRollerLoad();
			backIntake->BackRollerLoad(); // Load the next ball from the back
			printf("load");
			
			secondaryRollers->Pulse();
		}
		// Comes before the previous loop, but the first loop has more priority which is why it's list first
		else if(shotTimer->Get() > 0.3)
		{
			// If the second turn hasn't been executed yet, get the encoder values
			if(!startedSecondTurn)
			{
				if(direction == 1.0)
				{
					encoderStartValue = leftEncoder->Get();
				}
				if(direction == 2.0)
				{
					encoderStartValue = rightEncoder->Get();
				}
				startedSecondTurn = true;
			}

			if(direction == 1.0)
			{
				if(leftEncoder->Get() - encoderStartValue > -480) 
				//for directions
				{

					driverStationLCD->Printf((DriverStationLCD::Line) 4, 1, "Turn");
					// The turn
					drivetrain->TankDrive(-0.8, 0.4);
				}
				else
				{
					finishedSecondTurn = true;
					drivetrain->TankDrive(0.0, 0.0);
				}
			}
			
			else if(direction == 2.0)
			{
				if(rightEncoder->Get() - encoderStartValue > -480)
					//for directions
				{
					// The turn
					driverStationLCD->Printf((DriverStationLCD::Line) 4, 1, "Turn");
					
					drivetrain->TankDrive(0.4, -0.8);
				}
				else
				{
					finishedSecondTurn = true;
					drivetrain->TankDrive(0.0, 0.0);
				}
			}
		}

		//first
		else if (((direction == 2.0 &&rightEncoder->Get() > -3300) ||
				(direction == 1.0 &&leftEncoder->Get() > -3300)) &&
				(!startedSecondTurn || finishedSecondTurn))
		{
			
			if(direction == 2.0)
			{
				if(rightEncoder->Get() > -14)
				{
					// Make a slight turn at the start, then drive straight
					driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "Drive %d", leftEncoder->Get());
					drivetrain->TankDrive(-0.8, 0.0);
				}
				else
				{
					drivetrain->TankDrive(-0.8, -0.8);
				}
			}
			if(direction == 1.0)
			{
				if(leftEncoder->Get() > -14) 
				{
					// Make a slight turn at the start, the drive straight
					driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "Drive %d", leftEncoder->Get());
					drivetrain->TankDrive(0.0, -0.8);
				}
				else
				{
					drivetrain->TankDrive(-0.8, -0.8);
				}
			}
		} 
		// Stop driving
		else if (!doneDriving) {
			drivetrain->TankDrive(0.0, 0.0);
			doneDriving = true;
		}
		
		if (shotTimer->Get() > 3.2)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			drivetrain->TankDrive(0.0, 0.0);
			allDone = true;
			break;
			
		}
		driverStationLCD->UpdateLCD();
						
	}
	
	// Pause before the next shot
	driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "  NOT!");
	driverStationLCD->UpdateLCD();

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
			< 2.8) && EnabledInAutonomous(me)) 
	{
		if (ShootAutoConditions(shooter, me)) 
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
			
			// The wiggle to grab the ball. 
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

			if (autoTimer->Get() < 1.1) 
			{
				frontIntake->FrontRollerLoad();
			}
			else if (autoTimer->Get() > 1.1) 
			{
				if (autoTimer->Get() > 1.3) 
				{
					frontIntake->UndeployIntake();
				}
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


///////////////////////////
//
//	AUTO NUMBER SIX
//
///////////////////////////

/*
 * OVERVIEW
 * Start on the left side of the field 
 * Look for vision
 * Load top ball down
 * Drive forward and shoot 
 */

void TwoShotShortVision(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Timer *timer2, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder,Encoder *leftEncoder,
		DriverStation *driverStation, float direction, NetworkTable *table)
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


///////////////////////////
//
//	AUTO NUMBER SEVEN
//
///////////////////////////

/*
 * OVERVIEW
 * Doesn't use vision
 * Load top ball
 * Drive straight foward
 * Shoot
 * Load the back ball
 * Shoot again
 */

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


///////////////////////////
//
//	AUTO NUMBER EIGHT
//
///////////////////////////

/*
 * OVERVIEW
 * Start on one side of the field. 
 * Randomly choose a direction, and turn slightly that way
 * Drive up and shoot one
 * Load a ball form the front
 * Turn the opposite direciton before juking back
 * Shoot once more
 */

//For CC and CCC(PP) 
void RightLeftRightShot (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap, bool randomNum, DriverStation *driverStation)
{
	int k;
	// Even number = true. Odd number = false. Even means go right, then left. Odd means go left, then right. 
	if (randomNum){
		k = 1;
	}
	else {
		k = -1;
	}
	//preset things so they only run once
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	allDone = false;
	bool allDone2 = false;
	
	bool stopSecondary = false;
	bool backintakeup = false;

	spitShortSwap->Set(true); //shortest fingers up 
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
	shooter->ShooterPrime(true); // Prep for short shot

	
	autoTimer->Reset();
	shotTimer->Reset();
	rightEncoder->Reset();


	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
	{


		//first. Just drving while pulsing
		if(rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}


		
		//second. Over a certaion point, prep for shot
		if(!shootPrep && rightEncoder->Get() < -3300) 
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



		//THIRD
		if(shotTimer->Get() > 1.0)//1.9) 
		{
			if(!backintakeup)
			{
				backIntake->Stop();
				secondaryRollers->Undeploy();
				backIntake->UndeployIntake();
				backintakeup = true;
			}


			if(shotTimer->Get() > 1.5)
			{
				frontIntake->FrontPickup(driverStation);
			}
			else
			{
				//frontIntake->FrontRollerLoad();
				frontIntake->Stop();
			}


			if(shotTimer->Get() < 3.0)
			{
				drivetrain->TankDrive(0.1, 0.1);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			//frontIntake->FrontRollerLoad();
			
			secondaryRollers->Pulse();
		}
		
		//FIRST. Driving
		else if((leftEncoder->Get() > -45 && randomNum) || rightEncoder->Get() > -45) {
			drivetrain->TankDrive(-0.6, 0.0);
		}
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			drivetrain->TankDrive(-0.8, -0.8); // Drive slightly to the right to start.
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}


		if(shotTimer->Get() > 3.7)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			autoTimer->Start();
			autoTimer->Reset();
			break;
		}
	}

		//------------------------------------------------------



	frontIntake->DeployIntake();
	backIntake->DeployIntake();

	while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone2) && EnabledInAutonomous(me)){
		
		if (autoTimer->Get() < 0.2) { //TODO Time

			drivetrain->TankDrive(0.0, 0.0);
		}
		else if (autoTimer->Get() < 0.6) {
			drivetrain->TankDrive(0.75, -0.75);
		}
		else if (autoTimer->Get() < 1.0) {
			drivetrain->TankDrive(0.2, 0.2);
		}
		else if (autoTimer->Get() < 1.3) {
			drivetrain->TankDrive(-0.75, 0.75);
		}
		else if (autoTimer->Get() < 1.6) {
			drivetrain->TankDrive(0.6, 0.4);
		}
		else {
			drivetrain->TankDrive(0.0, 0.0);
		}
		
		if (autoTimer->Get() > 2.0){
			
			autoTimer->Stop();
			shotTimer->Stop();
			allDone2 = true;
		}
	}
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	
}	







// UNUSED
//Two shots, one short and one long
void TwoShotShortLong(IntakeSystem *frontIntake, IntakeSystem *backIntake,
		ShooterSystem *shooter, RobotDrive *drivetrain, Timer *autoTimer, Solenoid *spitShortSwap,
		SecondaryRollerSystem *secondaryRollers, IterativeRobot *me, Encoder *rightEncoder)
{
	spitShortSwap->Set(false);  //Shortest fingers down. Long shot
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



void PIDDriveStraight(CitrusPID *PID, RobotDrive *drivetrain, Encoder *leftEncoder, Encoder *rightEncoder)
{ //Currently, this is just a basic outline of what the PID should be,
	//There may be more math involved and the Coefficients will be tuned
	float pError = 0.0;
	float iError = 0.0;
	float dError = 0.0;
	float pCoefficient = 0.0;//These should be between 0 and 1 i think
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
		
		leftDriveTrainInput = 0.75 - correction;
		rightDriveTrainInput = 0.75 + correction;
		
		drivetrain->TankDrive(-leftDriveTrainInput, -rightDriveTrainInput); //negative so that it goes forward.  
	}
}





// UNUSED
//IO 3
//void ThreeShotGoalieRightLeft(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
//		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
//		bool allDone, Encoder *rightEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap)
//{
//	backIntake->DeployIntake();
//	frontIntake->DeployIntake();
//	LoadTopAutoPrep(autoTimer, shooter);
//	while (LoadTopAutoConditions(autoTimer, me)) {
//
//		backIntake->BackRollerAutoSlow();
//		frontIntake->FrontRollerAutoSlow();
//		secondaryRollers->Run();
//	}
//	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
//	//LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, this);
//	shooter->ShooterPrime(true);
//	//backIntake->DeployIntake();
//	backIntake->BackRollerAutoSlow();
//	frontIntake->FrontRollerAutoSlow();
//	//Wait(1.0);
//	bool shootPrep = false;
//	bool doneShooting = false;
//	bool stopSecondary = false;
//	bool backintakeup = false;
//	allDone = false;
//	bool doneDriving = false;
//
//	//while((rightEncoder->Get() > - 3300 || !doneShooting) && IsAutonomous())
//	while (ShootAutoConditions(shooter, me)
//			|| DriveForwardShootAutoConditions(autoTimer, me,
//					rightEncoder) || !allDone) {
//		//first
//		if (rightEncoder->Get() > -1000) {
//			secondaryRollers->Pulse();
//		} else if (!stopSecondary) {
//			stopSecondary = true;
//			secondaryRollers->Stop();
//		}
//
//		//Added this- remove for return to known good code.
//		if(!shootPrep && rightEncoder->Get() < -1550)
//		{
//			backIntake->BackRollerAutoSlow();
//			//backIntake->ReverseSlow();
//		}
//		else if(!shootPrep)
//		{
//			backIntake->BackRollerAutoSlow();
//		}
//		
//		//second
//		if (!shootPrep && rightEncoder->Get() < -1750) //3 feet forward?
//		{
//			shotTimer->Start();
//			shotTimer->Reset();
//			ShootAutoPrep(frontIntake, backIntake, shooter,
//					secondaryRollers, spitShortSwap, true);
//			shootPrep = true;
//		}
//		if (shootPrep && ShootAutoConditions(shooter, me)) {
//			ShootAutoInLoop(shooter);
//			printf("Shot");
//		} else if (shootPrep && !doneShooting) {
//			ShootAutoEnd();
//		}
//		//third
//		if (shotTimer->Get() > 1.4) {
//			if (!backintakeup) {
//				secondaryRollers->Undeploy();
//				//frontIntake->UndeployIntake();
//				backintakeup = true;
//			}
//			//drivetrain->TankDrive();
//			//frontIntake->FrontRollerLoad();
//			backIntake->BackRollerLoad();
//			printf("load");
//			
//			secondaryRollers->Pulse();
//		}
//		else if(shotTimer->Get() < 0.7 && shotTimer->Get() > 0.3) 
//		{
//			drivetrain->TankDrive(0.7, -0.7); //Second turn
//		}
//		
//
//		//First
//		else if (rightEncoder->Get() > -3340)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
//		{
//			if(rightEncoder->Get() > -35)
//			{
//				drivetrain->TankDrive(-0.8, 0.0);
//			}
//			else
//			{
//				//DriveForwardAutoInLoop(drivetrain);
//				drivetrain->TankDrive(-0.8, -0.8);
//			}
//		} else if (!doneDriving) {
//			DriveForwardAutoEnd(drivetrain);
//			doneDriving = true;
//		}
//		if (shotTimer->Get() > 3.0)//4.2)
//		{
//			printf("Shot timer > 3.0");
//			secondaryRollers->Stop();
//			DriveForwardAutoEnd(drivetrain);
//			allDone = true;
//			break;
//		}
//
//	}
//	drivetrain->TankDrive(0.0, 0.0);
//	frontIntake->DeployIntake();
//	backIntake->DeployIntake();
//	Wait(0.5);
//	ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
//			spitShortSwap, true);
//	autoTimer->Stop();
//	autoTimer->Reset();
//	autoTimer->Start();
//	backintakeup = false;
//	bool shotDone = false;
//	while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
//			< 3.0) && EnabledInAutonomous(me)) {
//		if (ShootAutoConditions(shooter, me)) {
//			ShootAutoInLoop(shooter);
//		} else if (!shotDone) {
//			ShootAutoEnd();
//			shotDone = false;
//		}
//		if (autoTimer->Get() > 0.2) {
//			if (!backintakeup) {
//				secondaryRollers->Undeploy();
//				//frontIntake->UndeployIntake();
//				frontIntake->DeployIntake();
//				backIntake->UndeployIntake();
//				backintakeup = true;
//			}
//			//drivetrain->TankDrive(0.4, 0.4);
//			//frontIntake->FrontRollerLoad();
//			if (autoTimer->Get() < 0.5) {
//				drivetrain->TankDrive(0.8, 0.4);
//			} else if (autoTimer->Get() < 0.9) {
//				drivetrain->TankDrive(-0.4, -0.8);
//			} else {
//				drivetrain->TankDrive(0.0, 0.0);
//			}
//
//			if (autoTimer->Get() < 0.9) {
//				frontIntake->FrontRollerLoad();
//			}
//			//frontIntake->FrontPickup(driverStation);
//			else if (autoTimer->Get() > 0.9) {
//				if (autoTimer->Get() > 1.1) {
//					frontIntake->UndeployIntake();
//				}
//				//frontIntake->ReverseSlow();
//				backIntake->ReverseSlow();
//				frontIntake->Stop();
//			}
//
//			secondaryRollers->Pulse();
//		}
//	}
//	LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
//	Wait(0.3);
//	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer,
//			secondaryRollers, spitShortSwap, me);
//	autoTimer->Stop();
//}

//3 short shots
// UNUSED
//void ThreeShotShort1(IntakeSystem *frontIntake, IntakeSystem *backIntake,
//		ShooterSystem *shooter, RobotDrive *drivetrain,DriverStation *driverStation, Timer *autoTimer, Timer *timer2,
//		SecondaryRollerSystem *secondaryRollers, Solenoid *spitShortSwap, IterativeRobot *me, 
//		Encoder *rightEncoder, NetworkTable *table, float startSide)
//{
//	//table->PutNumber("Enabled", 1);
//	spitShortSwap->Set(true); //shortest fingers up
//	printf("secondary pistons set true\n");
//	//frontIntake->FrontRollerAutoSlow();
//	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //load the ball of rollers
//	printf("Loaded from top\n");
//	//float visionInput = ReceiveVisionProcessing(table);
//	shooter->ShooterPrime(true); //longest fingers down
//	printf("Shooter primed\n");
//	Wait(1.0);
//	printf("Past wait");
//		
//	TwoShotShortPrep(shooter, timer2); 
//
//	//shopts once and drives forward
//	MultiAutoLoop(frontIntake, backIntake, shooter, drivetrain, 
//			autoTimer, timer2, secondaryRollers, spitShortSwap, me, rightEncoder,
//			driverStation);
//	printf("Hi! WATCH ME ACTUALLY WORK FOR THREE BALL!\n");
//	frontIntake->DeployIntake(); //set intakes down
//	backIntake->DeployIntake();
//	autoTimer->Stop();
//	//Load ball from the back while driving
//	LoadBackAutoDrive(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, drivetrain, me);
//	//Wait(1.0);
//	//shoot the abll once loaded
//	ShootAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
//}

/*
 * OVERVIEW
 * Starts on one side of the field or the other, center of one of the goals.
 * Drives forward, and turns to the opposite goal, firing a long shot there
 * Turn in an arc and curve back to the orginial goal, firing again
 */
void LongShortRandomGoalie (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, 
		RobotDrive *drivetrain, DriverStation *driverStation, bool randomNum, Solenoid *spitShortSwap)
{
	int k;
	// Even number = true. Odd number = false. Even means go right, then left. Odd means go left, then right. 
	if (randomNum){
		k = 1;
	}
	else {
		k = -1;
	}
	
	
	spitShortSwap->Set(false); // Prepping for long shot
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
	shooter->ShooterPrime(false); // Prep for long shot

	frontIntake->DeployIntake();
	backIntake->DeployIntake();
	
	autoTimer->Reset();
	shotTimer->Reset();
	rightEncoder->Reset();


	//backIntake->DeployIntake();
	//Wait(1.0);
	bool shootPrep = false;
	bool doneShooting = false;
	bool stopSecondary = false;
	bool backintakeup = false;
	allDone = false;
	bool doneDriving = false;
	bool startedSecondTurn = false;
	
	//while((rightEncoder->Get() > - 3300 || !doneShooting) && IsAutonomous())
	while (ShootAutoConditions(shooter, me)
			|| (EnabledInAutonomous(me) && leftEncoder->Get() > -1500) || !allDone) 
	{
		
		//first. Just drving while pulsing
		if (rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second. Over a certaion point, prep for shot
		if (!shootPrep && leftEncoder->Get() < -500) 
		{
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep (frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, false);
			shootPrep = true;
		}
		

		// Take the shot
		if(shootPrep && ShootAutoConditions(shooter, me))
		{
			ShootAutoInLoop(shooter);
		}
		else if(shootPrep && !doneShooting)
		{
			ShootAutoEnd();
		}
		
		//THIRD
		if(shotTimer->Get() > 1.0)//1.9) 
		{
			if(!backintakeup)
			{
				backIntake->Stop();
				secondaryRollers->Undeploy();
				backIntake->UndeployIntake();
				backintakeup = true;
			}

			if(shotTimer->Get() > 1.5)
			{
				frontIntake->FrontPickup(driverStation);
			}
			else
			{
				frontIntake->Stop();
			}


			if(shotTimer->Get() < 3.0)
			{
				drivetrain->TankDrive(0.1, 0.1);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			
			secondaryRollers->Pulse();
		}
		
		//FIRST. Driving
		// Turn slightly one way or the other
		else if ((leftEncoder->Get() > -60 || rightEncoder->Get() > -60) && randomNum) 
		{
			if (randomNum){
				drivetrain->TankDrive((0.8 * -k), 0.0);
			}
			else {
				drivetrain->TankDrive(0.0, (0.5 * -k));	
			}
		}
		else if(leftEncoder->Get() > -1500) // Not the full distance, only partially
		{
			drivetrain->TankDrive((0.8 * -k), (0.8 * -k)); // Drive straight forward
		}
		else if(!doneDriving) 
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}


		if(shotTimer->Get() > 3.7)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			autoTimer->Start();
			autoTimer->Reset();
			break;
		}		

	}
	
	
	frontIntake->DeployIntake();
	backIntake->DeployIntake();

	while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !startedSecondTurn) && EnabledInAutonomous(me)){
		
		if (autoTimer->Get() < 1.1) { //TODO 0.6 seconds

			drivetrain->TankDrive((0.6 * -k), (0.8 * -k));
		}
		// TODO check the second turn and see if anything else is needed. 
//		else if (autoTimer->Get() < 1.1) { // 0.5 seconds
//			drivetrain->TankDrive((0.6 * k), (0.6 * -k));
//		}
//		else if (autoTimer->Get() < 1.4) { // 0.3 seconds
//			drivetrain->TankDrive(0.7, 0.1);
//		}
//		else if (autoTimer->Get() < 1.6) { // 0.2 seconds
//			drivetrain->TankDrive((0.3 * -k), (0.1 * k));
//		}
//		else {
//			drivetrain->TankDrive(0.0, 0.0);
//		}
		
		if (autoTimer->Get() > 0.7){
			
			autoTimer->Stop();
			shotTimer->Stop();
			startedSecondTurn = true;
		}
		
	}
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);

}




/*
 * OVERVIEW
 * Start in the center of the field
 * Deploy intakes and prep to load top
 * Load the top ball down 
 * Prime shooter and play with variables
 * Shoot (long?)
 * Load a ball from the front
 * Shoot   (do driving during most of the routine)
 * Load a ball from the back
 * Shoot one last time
 */

void ThreeBallStraight (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, 
		RobotDrive *drivetrain, DriverStation *driverStation, Solenoid *spitShortSwap) 
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
		
		shooter->ShooterPrime(true);
		
		backIntake->BackRollerAutoSlow();
		frontIntake->FrontRollerAutoSlow();
		
		
		bool shootPrep 	   = false;
		bool doneShooting  = false;
		bool stopSecondary = false;
		bool backintakeup  = false;
		allDone 	       = false;
		bool doneDriving   = false;
		
		while ((ShootAutoConditions(shooter, me)
				|| DriveForwardShootAutoConditions(autoTimer, me,
						rightEncoder) || !allDone)&& EnabledInAutonomous(me)) 
		{
			//first
			if(shotTimer->Get() < 1.4)
			{
				if (rightEncoder->Get() > -1400) {
					secondaryRollers->RunAt(1.0);
				} else if (!stopSecondary) {
					stopSecondary = true;
					secondaryRollers->Stop();
				}
			}
			
			if(!shootPrep && rightEncoder->Get() < -2800)//1550)
			{
				backIntake->ReverseSlow();
			}
			else if(!shootPrep)
			{
				if(!shootPrep && rightEncoder->Get() > -1000 && shotTimer->Get() < 1.4)
				{
					backIntake->RunAt(0.45);
				}
				else if(!shootPrep && rightEncoder->Get() > -2800)
				{
					backIntake->RunAt(0.4);
				}
			}
			
			//second
			if (!shootPrep && rightEncoder->Get() < -3000)//1750) //3 feet forward?
			{
				shotTimer->Start();
				shotTimer->Reset();
				ShootAutoPrep(frontIntake, backIntake, shooter,
						secondaryRollers, spitShortSwap, true);
				shootPrep = true;
			}
			if (shootPrep && ShootAutoConditions(shooter, me)) {
				ShootAutoInLoop(shooter);
			} else if (shootPrep && !doneShooting) {
				ShootAutoEnd();
			}
			
			//Drivetrain.
			//third
			if (shotTimer->Get() > 1.4) { //&& < 0.3
				if (!backintakeup) {
					secondaryRollers->Undeploy();
					//frontIntake->UndeployIntake();
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
				backIntake->BackRollerLoad();
				secondaryRollers->Pulse();
			}

			//First
			else if (rightEncoder->Get() > -3300)
			{
				DriveForwardAutoInLoop(drivetrain);
			} else if (!doneDriving) {
				DriveForwardAutoEnd(drivetrain);
				doneDriving = true;
			}
			if (shotTimer->Get() > 3.4)//4.2)
			{
				secondaryRollers->Stop();
				DriveForwardAutoEnd(drivetrain);
				allDone = true;
				break;
			}

		}
		// Intermediate pause before going into the next shot
		drivetrain->TankDrive(0.0, 0.0);
		frontIntake->DeployIntake();
		backIntake->DeployIntake();
		Wait(0.5); // Wait for intakes to settle
		ShootAutoPrep(frontIntake, backIntake, shooter, secondaryRollers,
				spitShortSwap, true);
		autoTimer->Stop();
		autoTimer->Reset();
		autoTimer->Start();
		backintakeup = false;
		bool shotDone = false;
		
		
		while ((ShootAutoConditions(shooter, me) || autoTimer->Get()
				< 2.4) && EnabledInAutonomous(me)) { //3.0
			
			if (ShootAutoConditions(shooter, me)) {
				ShootAutoInLoop(shooter);
			} 
			else if (!shotDone) {
				ShootAutoEnd();
				shotDone = false;
			}
			
			if (autoTimer->Get() > 0.2) {
				if (!backintakeup) {
					secondaryRollers->Undeploy();
					frontIntake->DeployIntake();
					backIntake->UndeployIntake();
					backintakeup = true;
				}
				if (autoTimer->Get() < 0.5) { //Doing a little dance to get the ball in
					drivetrain->TankDrive(0.7, 0.7);
				} else if (autoTimer->Get() < 0.9) {
					drivetrain->TankDrive(-0.7, -0.7);
				} else {
					drivetrain->TankDrive(0.0, 0.0);
				}

				if (autoTimer->Get() < 0.9) {
					frontIntake->FrontRollerLoad();
				}
				else if (autoTimer->Get() > 0.9) {
					if (autoTimer->Get() > 1.1) {
						frontIntake->UndeployIntake();
					}
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


/*
 * OVERVIEW
 * Start on one side of the field. 
 * Drive straight foward
 * Turn at the last second toward one side of the goal
 * Shoot a short shot
 * Load a ball from the front
 * Turn to the other side of the goal 
 * Shoot a short shot once more
 */

void CloseLongShortRandomGoalie (IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers, 
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer,
		RobotDrive *drivetrain, Solenoid *spitShortSwap, DriverStation *driverStation, bool randomNum)
{
	int k;
	// Even number = true. Odd number = false. Even means go right, then left. Odd means go left, then right. 
	if (randomNum){
		k = 1;
	}
	else {
		k = -1;
	}
	
	
	spitShortSwap->Set(true); //shortest fingers up 
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
	shooter->ShooterPrime(false); // Prep for long shot

	
	autoTimer->Reset();
	shotTimer->Reset();
	rightEncoder->Reset();


	//backIntake->DeployIntake();
	//Wait(1.0);
	bool shootPrep = false;
	bool doneShooting = false;
	bool stopSecondary = false;
	bool backintakeup = false;
	allDone = false;
	bool doneDriving = false;
	
	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
	{
		
		//first. Just drving while pulsing
		if (rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}
		
		//second. Over a certaion point, prep for shot
		if (!shootPrep && leftEncoder->Get() < -3300) 
		{
			shotTimer->Start();
			shotTimer->Reset();
			ShootAutoPrep (frontIntake, backIntake, shooter, secondaryRollers, spitShortSwap, true);
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
		
		//THIRD
		if(shotTimer->Get() > 1.0)//1.9) 
		{
			if(!backintakeup)
			{
				backIntake->Stop();
				secondaryRollers->Undeploy();
				backIntake->UndeployIntake();
				backintakeup = true;
			}


			if(shotTimer->Get() > 1.5)
			{
				frontIntake->FrontPickup(driverStation);
			}
			else
			{
				//frontIntake->FrontRollerLoad();
				frontIntake->Stop();
			}


			if(shotTimer->Get() < 3.0)
			{
				drivetrain->TankDrive(0.1, 0.1);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			
			secondaryRollers->Pulse();
		}
		
		//FIRST. Driving
		
		else if(leftEncoder->Get() > -3200) // TODO fix distance. Applies for ALL driving in this routine
		{
			drivetrain->TankDrive((0.8 * -k), (0.8 * -k)); // Drive straight forward
		}
		else if ((leftEncoder->Get() > -3300 || rightEncoder->Get() > -3300) && randomNum) {
			// Otherwise, turn slightly one way or another for the first shot
			if (randomNum){
				drivetrain->TankDrive((0.5 * -k), 0.0);
			}
			else {
				drivetrain->TankDrive(0.0, (0.5 * -k));	
			}
		}			
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}


		if(shotTimer->Get() > 3.7)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			autoTimer->Start();
			autoTimer->Reset();
			break;
		}		

	}
	
	
	frontIntake->DeployIntake();
	backIntake->DeployIntake();

	
	// TODO any other driving should be add back in
	while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder)) && EnabledInAutonomous(me)){
		
		// Turn back the other way
		if ((leftEncoder->Get() > -3400 || rightEncoder->Get() > -3400) && randomNum) {
			// Maybe instead of turning by driving forward, turn by driving backward
			// Turn backward before the next shot
			if (randomNum){
				drivetrain->TankDrive(0.0, (0.5 * -k));
			}
			else {
				drivetrain->TankDrive((0.5 * -k), 0.0);	 
			}
		}	
		else {
			DriveForwardAutoEnd(drivetrain);
		}
		
					
//		if (autoTimer->Get() < 0.6) { //TODO 0.6 seconds
//
//			drivetrain->TankDrive(0.2, 0.6);
//		}
//		else if (autoTimer->Get() < 1.1) { // 0.5 seconds
//			drivetrain->TankDrive((0.6 * k), (0.6 * -k));
//		}
//		else if (autoTimer->Get() < 1.4) { // 0.3 seconds
//			drivetrain->TankDrive(0.7, 0.1);
//		}
//		else if (autoTimer->Get() < 1.6) { // 0.2 seconds
//			drivetrain->TankDrive((0.3 * -k), (0.1 * k));
//		}
//		else {
//			drivetrain->TankDrive(0.0, 0.0);
//		}
//		
//		if (autoTimer->Get() > 1.7){
//			
//			autoTimer->Stop();
//			shotTimer->Stop();
//			startedSecondTurn = true;
//		}
//			
	}
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);

}



/*
 * OVERVIEW
 * Start on one side of the field. 
 * Randomly choose a direction, and turn slightly that way
 * Drive up and shoot one
 * Load a ball form the front
 * Turn the opposite direciton before juking back
 * Shoot once more
 */

//For CC and CCC(PP) 
void TwoShotRandom(IntakeSystem *backIntake, IntakeSystem *frontIntake, Timer *autoTimer,
		ShooterSystem *shooter, IterativeRobot *me, SecondaryRollerSystem *secondaryRollers,
		bool allDone, Encoder *rightEncoder, Encoder *leftEncoder, Timer *shotTimer, RobotDrive *drivetrain, Solenoid *spitShortSwap, bool randomNum, DriverStation *driverStation)
{
	int k;
	// Even number = true. Odd number = false. Even means go right, then left. Odd means go left, then right. 
	if (randomNum){
		k = 1;
	}
	else {
		k = -1;
	}
	//preset things so they only run once
	bool shootPrep = false;
	bool doneDriving = false;
	bool doneShooting = false;
	
	allDone = false;
	bool allDone2 = false;
	
	bool stopSecondary = false;
	bool backintakeup = false;

	spitShortSwap->Set(true); //shortest fingers up 
	frontIntake->FrontRollerAutoSlow();
	LoadTopAuto(secondaryRollers, frontIntake, backIntake, autoTimer, shooter, me); //sucks ball in off the rollers
	shooter->ShooterPrime(true); // Prep for short shot

	
	autoTimer->Reset();
	shotTimer->Reset();
	rightEncoder->Reset();


	while((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone) && EnabledInAutonomous(me))
	{


		//first. Just drving while pulsing
		if(rightEncoder->Get() > -1000)
		{
			secondaryRollers->Pulse();
		}
		else if (!stopSecondary)
		{
			stopSecondary = true;
			secondaryRollers->Stop();
		}


		
		//second. Over a certaion point, prep for shot
		if(!shootPrep && rightEncoder->Get() < -3300) 
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



		//THIRD
		if(shotTimer->Get() > 1.0)//1.9) 
		{
			if(!backintakeup)
			{
				backIntake->Stop();
				secondaryRollers->Undeploy();
				backIntake->UndeployIntake();
				backintakeup = true;
			}


			if(shotTimer->Get() > 1.5)
			{
				frontIntake->FrontPickup(driverStation);
			}
			else
			{
				//frontIntake->FrontRollerLoad();
				frontIntake->Stop();
			}


			if(shotTimer->Get() < 3.0)
			{
				drivetrain->TankDrive(0.1, 0.1);
			}
			else
			{
				drivetrain->TankDrive(0.0, 0.0);
			}
			
			//frontIntake->FrontRollerLoad();
			
			secondaryRollers->Pulse();
		}
		
		//FIRST. Driving
		else if((leftEncoder->Get() > -50 && randomNum) || rightEncoder->Get() > -50) {
			if (randomNum){
				drivetrain->TankDrive((0.7 * -k), 0.0);
			}
			else {
				drivetrain->TankDrive(0.0, (0.7 * -k));	
			}
		}
		else if(rightEncoder->Get() > -3300)//DriveForwardShootAutoConditions(timer, me, rightEncoder))
		{
			drivetrain->TankDrive(-0.8, -0.8); // Drive slightly to the right to start.
		}
		else if(!doneDriving)
		{
			DriveForwardAutoEnd(drivetrain);
			doneDriving = true;
		}


		if(shotTimer->Get() > 3.7)//3.0)//4.2)
		{
			printf("Shot timer > 4.2");
			secondaryRollers->Stop();
			DriveForwardAutoEnd(drivetrain);
			allDone = true;
			autoTimer->Start();
			autoTimer->Reset();
			break;
		}
	}

		//------------------------------------------------------



	frontIntake->DeployIntake();
	backIntake->DeployIntake();

	while ((ShootAutoConditions(shooter, me) || DriveForwardShootAutoConditions(autoTimer, me, rightEncoder) || !allDone2) && EnabledInAutonomous(me)){
		
		if (autoTimer->Get() < 0.4) { //TODO Time

			drivetrain->TankDrive(0.0, 0.0);
		}
		else if (autoTimer->Get() < 0.8) {
			drivetrain->TankDrive((0.75 * k), (0.75 * -k));
		}
		else if (autoTimer->Get() < 1.2) {
			drivetrain->TankDrive(0.0, 0.0);
		}
		else if (autoTimer->Get() < 1.6) {
			drivetrain->TankDrive((0.75 * -k), (0.75 * k));
		}
		else {
			drivetrain->TankDrive(0.0, 0.0);
		}
		
		if (autoTimer->Get() > 2.0){
			
			autoTimer->Stop();
			shotTimer->Stop();
			allDone2 = true;
		}
	}
	
	ShootShortAuto(frontIntake, backIntake, shooter, autoTimer, secondaryRollers, spitShortSwap, me);
	
}	




#endif	
