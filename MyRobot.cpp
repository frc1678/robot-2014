#include "WPILib.h" //Lots of convenient robot classes
#include "NetworkTables/NetworkTable.h" //Networktables used for vision
#include "Drivetrain.h"
#include "IntakeSystem.h" //System of pneumatics and motors involved in ball intake
#include "HumanLoad.h" //System of operations used for human loading
#include "MPU6050_I2C.h" //Non-KOP gyro
#include "ShooterSystem.h" //System of pneumatics and motors involved in shooting
#include "CitrusButton.h" //Custom button class used for manipulator and joystick buttons
#include "SecondaryRollerSystem.h" //System of pneumatics and motors involved in settling ball before shot
#include "AutonomousRoutines.h" //Collection of routines to chose from in autonomous
#include "AutonomousSubroutines.h" //Collection of operations to be used in AutonomousRoutines
#include "Definitions.h" //Used to simplify syntax for button operations
#include "CrioFile.h" //Used for logging live in-match data to files on the Crio

class Robot: public IterativeRobot {
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;

	CrioFile *currentSensor;
	AnalogChannel *a;
	AnalogChannel *b;
	
    //Joysticks
	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator; //gaming style controller

	//Network Tables
	NetworkTable *dataTable;
	NetworkTable *table;

	//Compressor
	Compressor *compressor;

	//Solenoids
	Solenoid *gearUp; //compressed air
	Solenoid *gearDown;
	Solenoid *frontIntakeDeploy;
	Solenoid *backIntakeDeploy;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;
	Solenoid *armPiston;
	Solenoid *spitShortSwap;

	//Sensors
	Encoder *leftEncoder; //wheel rotation clicks
	Encoder *rightEncoder;
	
	//Talons	
	RobotDrive *drivetrain; // robot drive system
	Talon *secondaryRollerA;
	Talon *secondaryRollerB;

	//Secondary Rollers
	SecondaryRollerSystem *secondaryRollers;

	//Timers.
	//For use in auto when timing routines
	Timer *autoTimer2; 
	Timer *autoTimer;
	Timer *turnTimer;
	Timer *shotTimer;

	//Intakes.
	IntakeSystem *frontIntake;
	IntakeSystem *backIntake;

	//Intake Toggle
	bool gearToggle;
	bool frontIntakeToggle;
	bool backIntakeToggle;

	//Arm piston.
	bool armPistonToggle;

	//Shooting
	ShooterSystem *shooter;
	bool shortShot;
	bool shotKillSwitch;

	//Buttons!
	CitrusButton *b_gearUp;
	CitrusButton *b_gearDown;
	CitrusButton *b_testGearToggle;
	CitrusButton *b_shotAlignLong;
	CitrusButton *b_shotAlignShort;
	CitrusButton *b_frontIntakePickup;
	CitrusButton *b_backIntakePickup;
	CitrusButton *b_frontIntakeDeployToggle;
	CitrusButton *b_backIntakeDeployToggle;
	CitrusButton *b_shortShoot;
	CitrusButton *b_longShoot;
	CitrusButton *b_shooterPrime;
	CitrusButton *b_pulseSecondary;
	CitrusButton *b_foldFlower;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_armPistonToggle;
	CitrusButton *b_runSecondary;
	CitrusButton *b_killShotL;
	CitrusButton *b_killShotR;
	CitrusButton *b_toggleShotAlign;
	CitrusButton *b_notShooting;
	CitrusButton *b_notShooting2;

public:
	Robot() {
		printf("Robot INITIALIZATION\n");

		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();

		driverL = new Joystick(1); //Joystick used to control left drivetrain
		driverR = new Joystick(2); //Joystick used to control right drivetrain
		manipulator = new Joystick(3); //Gamepad used to control most pneumatics and intakes

		currentSensor = new CrioFile();
		a = new AnalogChannel(3);
		b = new AnalogChannel(7);
        
		//Used in PID testing
		dataTable = NetworkTable::GetTable("TurnTable");
		dataTable->PutNumber("degreeOfTurn", 90.0);
		dataTable->PutNumber("kpError" , 2); //use 0.8 or 2
		dataTable->PutNumber("kiError", 0.023); //use 0.027 or 0.023
		dataTable->PutNumber("kdError", 0.5); //use 0.078 or 0.5

		table = NetworkTable::GetTable("datatable");
		table->PutNumber("Enabled", 0);

		compressor = new Compressor(1, 1);

		//Solenoids.
		gearUp = new Solenoid(8);
		gearDown = new Solenoid(7);
		frontIntakeDeploy = new Solenoid(3); //deploy = put intake down
		backIntakeDeploy = new Solenoid(4);
		shotAlignerUp = new Solenoid(5);
		shotAlignerDown = new Solenoid(6);
		armPiston = new Solenoid(2);
		spitShortSwap = new Solenoid(1);

		//Sensors 
		leftEncoder = new Encoder(4, 5); //values switched between comp and practice robot (7, 6)
		rightEncoder = new Encoder(7, 6); //(4, 5)

		//Begins counting of encoder clicks for both drivetrains
		leftEncoder->Start();
		rightEncoder->Start();

		
		drivetrain = new RobotDrive(3, 4);
		//SecondaryIntake
		secondaryRollerA = new Talon(7);
		secondaryRollerB = new Talon(8);

		secondaryRollers = new SecondaryRollerSystem(secondaryRollerA,
				secondaryRollerB, armPiston);

		//Autonomous Timer
		autoTimer2 = new Timer();
		autoTimer = new Timer();
		turnTimer = new Timer(); //starts && resets when we start gyroTurnAngle 
		shotTimer = new Timer();

		frontIntake = new IntakeSystem(6, secondaryRollerA, secondaryRollerB,
				3, frontIntakeDeploy, true);
		backIntake = new IntakeSystem(1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); 

		gearToggle = false;
		frontIntakeToggle = false;
		backIntakeToggle = false;

		armPistonToggle = false;

		//Shooter
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston);

		shortShot = false;
		shotKillSwitch = false;

		//Buttons 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons 
		b_gearUp = new CitrusButton(driverL, 2);
		b_gearDown = new CitrusButton(driverR, 2);
		b_testGearToggle = new CitrusButton(k_btestGearToggle);
		//Intake buttons
		b_frontIntakePickup = new CitrusButton(k_bfrontIntakePickup);
		b_backIntakePickup = new CitrusButton(k_bbackIntakePickup);
		b_frontIntakeDeployToggle
				= new CitrusButton(k_bfrontIntakeDeployToggle);
		b_backIntakeDeployToggle = new CitrusButton(k_bbackIntakeDeployToggle);
		b_reverseIntake = new CitrusButton(k_breverseIntake);
		b_humanLoad = new CitrusButton(k_bhumanLoad);
		b_runSecondary = new CitrusButton(k_brunSecondary);
		//Shooter buttons
		b_shotAlignLong = new CitrusButton(k_bshotAlignLong);
		b_shotAlignShort = new CitrusButton(k_bshotAlignShort);
		b_shortShoot = new CitrusButton(k_bshortShoot);
		b_longShoot = new CitrusButton(k_blongShoot);
		b_shooterPrime = new CitrusButton(k_bshooterPrime);
		b_pulseSecondary = new CitrusButton(k_bpulseSecondary);
		b_foldFlower = new CitrusButton(k_bfoldFlower);
		b_armPistonToggle = new CitrusButton(k_barmPistonToggle);
		b_killShotL = new CitrusButton(k_bkillShotL);
		b_killShotR = new CitrusButton(k_bkillShotR);
		b_toggleShotAlign = new CitrusButton(k_btoggleShotAlign);
		b_notShooting = new CitrusButton(driverR, 8);
		b_notShooting2 = new CitrusButton(driverR, 9);
	}

	void UpdateAllButtons() {
		//gearshift
		b_gearUp->Update();
		b_gearDown->Update();
		b_testGearToggle->Update();
		b_shotAlignLong->Update();
		b_shotAlignShort->Update();
		//intake
		b_frontIntakePickup->Update();
		b_backIntakePickup->Update();
		b_frontIntakeDeployToggle->Update();
		b_backIntakeDeployToggle->Update();
		b_reverseIntake->Update();
		b_humanLoad->Update();
		b_runSecondary->Update();
		//shooter
		b_shortShoot->Update();
		b_longShoot->Update();
		b_shooterPrime->Update();
		b_pulseSecondary->Update();
		b_foldFlower->Update();
		b_armPistonToggle->Update();
		b_killShotL->Update();
		b_killShotR->Update();
		b_toggleShotAlign->Update();
		b_notShooting->Update();
		b_notShooting2->Update();
	}

	void DisabledInit() {
		
		shooter->Reset();

		leftEncoder->Reset();
		rightEncoder->Reset();

		//gyro->Reset();
		
		//Stop Timers
		autoTimer->Stop();
		turnTimer->Stop();
		shotTimer->Stop();
		
		frontIntake->UndeployIntake();
		backIntake->UndeployIntake();
		secondaryRollers->Undeploy();
		
		table->PutNumber("Enabled", 0);
		driverStationLCD->Clear();
		drivetrain->TankDrive(0.0, 0.0); //TankDrive is a method of driving controling each drivetrain separately, parameter 1 is left drivetrain, 2 is right
		//TankDrive values range from -1 to 1
		//currentSensor->EndLog();
		
	}
	void DisabledPeriodic() {
		table->PutNumber("Enabled", 0); // 0 means not enabled, 1 means it is
		drivetrain->TankDrive(0.0, 0.0); //stop
	}
	void AutonomousInit() {
		spitShortSwap->Set(true); //fingers on the shooter used to controll the ball
		driverStationLCD->Clear();
		driverStationLCD->UpdateLCD();
		//table->PutNumber("Enabled", 1);
		
		table->PutString("Direction: ", "MERP"); //MERP so that we get an error if timing with network tables doesn't work
		//leftEncoder->Start();
		//rightEncoder->Start();
		gearDown->Set(true);
		gearUp->Set(false);
		bool allDone;
		
		rightEncoder->Reset();
		leftEncoder->Reset();
		//beginning of long chain to determine which auto we are running
		//for running motors: -1 to 1 input range
		//This applies to several of these: sorry for messy code, not all of them work
		if (driverStation->GetDigitalIn(1))
		{
			FunAuto(drivetrain, leftEncoder, rightEncoder);
			//ThreeShotGoalie3(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder, shotTimer, drivetrain, spitShortSwap); //goalie avoidance 3 ball
		} else if (driverStation->GetDigitalIn(2)) 
		{ /*Summary: 
		1. deploy intakes and prep to load top
		2. load top
		3. prime shooter and play with variables
		4. shoot
		5. load front
		6. shoot   (do driving during most of the routine)
		7. load back
		8. shoot */
			backIntake->DeployIntake();
			frontIntake->DeployIntake();
			LoadTopAutoPrep(autoTimer, shooter);
			while (LoadTopAutoConditions(autoTimer, this)) {

				backIntake->BackRollerAutoSlow();
				frontIntake->FrontRollerAutoSlow();
				secondaryRollers->Run();
			}
			LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
			shooter->ShooterPrime(true);
			backIntake->BackRollerAutoSlow();
			frontIntake->FrontRollerAutoSlow();
			bool shootPrep = false;
			bool doneShooting = false;
			bool stopSecondary = false;
			bool backintakeup = false;
			bool allDone = false;
			bool doneDriving = false;
			while ((ShootAutoConditions(shooter, this)
					|| DriveForwardShootAutoConditions(autoTimer, this,
							rightEncoder) || !allDone)&& IsAutonomous()) 
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
				if (shootPrep && ShootAutoConditions(shooter, this)) {
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
			while ((ShootAutoConditions(shooter, this) || autoTimer->Get()
					< 2.4) && IsAutonomous()) { //3.0
				if (ShootAutoConditions(shooter, this)) {
					ShootAutoInLoop(shooter);
				} else if (!shotDone) {
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
					secondaryRollers, spitShortSwap, this);
			autoTimer->Stop();
			
		} else if (driverStation->GetDigitalIn(3)) {
			ThreeShotGoalieRightLeft(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers,
					allDone, rightEncoder, shotTimer, drivetrain, spitShortSwap);
			
		} else if (driverStation->GetDigitalIn(4)) {
			printf("In GetDigitalIn 4");
			OneShotShort(frontIntake, backIntake, shooter, drivetrain,
					autoTimer, turnTimer, spitShortSwap, secondaryRollers,
					this, rightEncoder, driverStationLCD, table, 1.0);
			
		} else if (driverStation->GetDigitalIn(5)) { 
			table->PutNumber("Enabled", 1);
			backIntake->DeployIntake();
			frontIntake->DeployIntake();
			LoadTopAutoPrep(autoTimer, shooter);
			int encoderStartValue = 0;
			while (LoadTopAutoConditions(autoTimer, this)) {

				backIntake->BackRollerAutoSlow();
				frontIntake->FrontRollerAutoSlow();
				secondaryRollers->Run();
			}
			LoadTopAutoEnd(secondaryRollers, frontIntake, backIntake);
			float direction = ReceiveVisionProcessing(table);
			if(direction == 0.0)
			{
				driverStationLCD->Printf((DriverStationLCD::Line)3, 1, "VISION ERROR");
				direction = 1.0;
			}
			shooter->ShooterPrime(true);
			backIntake->BackRollerAutoSlow();
			frontIntake->FrontRollerAutoSlow();
			bool shootPrep = false;
			bool doneShooting = false;
			bool stopSecondary = false;
			bool backintakeup = false;
			bool allDone = false;
			bool doneDriving = false;
			bool startedSecondTurn = false;
			bool finishedSecondTurn = false;
			while (ShootAutoConditions(shooter, this)
					|| DriveForwardShootAutoConditions(autoTimer, this, rightEncoder) 
					|| !allDone) 
			{
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
				
				if (shootPrep && ShootAutoConditions(shooter, this)) {
					ShootAutoInLoop(shooter);
					printf("Shot");
				} else if (shootPrep && !doneShooting) {
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
						drivetrain->TankDrive(-0.6, -0.6);
					}
					else
					{
						drivetrain->TankDrive(0.0, 0.0);
					}
					//frontIntake->FrontRollerLoad();
					backIntake->BackRollerLoad();
					printf("load");
					
					secondaryRollers->Pulse();
				}
				else if(shotTimer->Get() > 0.3)
				{
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
							driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "Drive %d", leftEncoder->Get());
							drivetrain->TankDrive(0.0, -0.8);
						}
						else
						{
							drivetrain->TankDrive(-0.8, -0.8);
						}
					}
				} 
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
								
			}driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "  NOT!");
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
					else if (autoTimer->Get() > 0.9) 
					{
						if (autoTimer->Get() > 1.1) 
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
					secondaryRollers, spitShortSwap, this);
			autoTimer->Stop();
		} 
		else if (driverStation->GetDigitalIn(6)) {
			float direction = 1.0; //TODO Put back to receive vision processing and not manualy set the hot goal
			TwoShotShortVision(frontIntake, backIntake, shooter,
					drivetrain, autoTimer, autoTimer2,spitShortSwap,
					secondaryRollers, this,rightEncoder, leftEncoder, driverStation, direction, table);
					
			
		} 
		else if (driverStation->GetDigitalIn(7)) {
			printf("In GetDigitalIn 7");
			TwoShotShortShort(frontIntake, backIntake, shooter, drivetrain,
					autoTimer, shotTimer, spitShortSwap, secondaryRollers,
					this, rightEncoder, driverStation);
		} 		
		 else if (driverStation->GetDigitalIn(8))
		 {
			 ThreeShotGoalieLeftRight(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder, shotTimer, drivetrain, spitShortSwap);
		 }
	}
	void AutonomousPeriodic() {
		//printf("Gyro: %f, Gyro Rate: %f\n", gyro->GetAngle(), gyro->GetRate());
	}
	void TeleopInit() {
			
		spitShortSwap->Set(true);

		compressor->Start();


		gearToggle = false;
		gearUp->Set(!gearToggle);
		gearDown->Set(gearToggle);

		}
	void TeleopPeriodic() {
		currentSensor->LogCurrent(a);
		currentSensor->LogHeat(b);
		currentSensor->LogEncoders(leftEncoder, rightEncoder);
		/*for (int i = 0; i < 5; i++) { //for putting the current log on the driverstation.
			if (i == 4) {
				CurrentData[i] = a->GetVoltage();
			} else {
				CurrentData[i] = CurrentData[i + 1];
			}

			driverStationLCD->Printf((DriverStationLCD::Line) i, 1, "C:%f",
					CurrentData[i]);
		}*/
		///driverStationLCD->Printf((DriverStationLCD::Line)4, 1, "H:%f", currentSensor->CheckHeat(b));

		dataTable->PutNumber("Enabled", 1);
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());


		//Drive.
		//runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		runDrivetrainShift(driverL->GetY(), driverR->GetY(), drivetrain, 0.2, gearUp, gearDown, leftEncoder, rightEncoder);
		//currentSensor->VoltageMonitor(gearUp, gearDown, currentSensor, a, driverStationLCD);

		driverStationLCD->Printf((DriverStationLCD::Line) 3, 1,
				"Left Encoder: %f", leftEncoder->GetRate());
		driverStationLCD->Printf((DriverStationLCD::Line) 4, 1,
				"Right Encoder: %f", rightEncoder->GetRate());
		driverStationLCD->Printf((DriverStationLCD::Line) 2, 1, 
				"%f", manipulator->GetRawAxis(2));
		driverStationLCD->UpdateLCD();
		
		if (b_gearUp->ButtonClicked()) {
			gearUp->Set(false);
			gearDown->Set(true);
		}
		if (b_gearDown->ButtonClicked()) {
			gearUp->Set(true);
			gearDown->Set(false);
		}

		if (b_frontIntakePickup->ButtonPressed()) {
			//intakes to hold the ball using proxy sensors
			if (b_frontIntakePickup->ButtonClicked()
					&& manipulator->GetRawAxis(2) < 0.5) {
				frontIntake->DeployIntake();
			}
			if (b_frontIntakePickup->ButtonClicked()) {
				spitShortSwap->Set(true);
			}
			if (manipulator->GetRawAxis(2) > 0.5) {
				frontIntake->FrontRollerLoad();
			} else {
				frontIntake->FrontPickup(driverStation);
			}
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
		}
		//activates pickup for back intake
		else if (b_backIntakePickup->ButtonPressed()) {
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			if (b_backIntakePickup->ButtonClicked() && manipulator->GetRawAxis(
					2) < 0.5) {
				shortShot = true;
				shooter->ShooterPrime(shortShot);
				backIntake->DeployIntake();

				spitShortSwap->Set(true);
			}
			if (manipulator->GetRawAxis(2) > 0.5) {
				backIntake->BackRollerLoad();
			} else {
				backIntake->Pickup(manipulator, driverStation);
			}
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
		} else if (b_reverseIntake->ButtonPressed()) {
			//intake reverses
			if (b_reverseIntake->ButtonClicked()) {
				shortShot = true;
				shooter->ShooterPrime(shortShot);
				spitShortSwap->Set(false);
			}
			//secondaryRollers->ReverseSlow();
			//secondaryRollers->Run();
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(1.0);
			backIntake->Reverse();
		} else if (b_humanLoad->ButtonPressed()) {
			HPReceive(secondaryRollers, frontIntake, backIntake, shooter);
		} else if (b_runSecondary->ButtonPressed()) {
			secondaryRollers->Run();
		} else {
			frontIntake->Stop();
			backIntake->Stop();
			secondaryRollers->Stop();
		}

		if (b_reverseIntake->ButtonReleased()) {
			spitShortSwap->Set(true);
		}

		//toggles the front intake up and down
		if (b_frontIntakeDeployToggle->ButtonClicked()) {
			frontIntake->ToggleIntake();
			if (!frontIntake->DeployState()) {
				secondaryRollers->Undeploy();
			}
		}

		//toggles the back intake up and down
		if (b_backIntakeDeployToggle->ButtonClicked()) {
			backIntake->ToggleIntake();
			if (!frontIntake->DeployState()) {
				secondaryRollers->Undeploy();
			}
		}
		//Arm piston toggle
		if (b_armPistonToggle->ButtonClicked()) {
			secondaryRollers->ToggleArms();
		}

		if (b_longShoot->ButtonClicked()) {
			frontIntake->DeployIntake();
			backIntake->DeployIntake();
		}
		if (b_longShoot->ButtonReleased()) {
			if (!shotKillSwitch) {
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
		}
		if (b_shortShoot->ButtonClicked()) {
			frontIntake->DeployIntake();
			backIntake->DeployIntake();
		}
		if (b_shortShoot->ButtonReleased()) {
			if (!shotKillSwitch) {
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
		}

		if (b_shotAlignLong->ButtonClicked()) {
			shortShot = false;
			shooter->ShooterPrime(shortShot);
			spitShortSwap->Set(false);
		}
		if (b_shotAlignShort->ButtonClicked()) {
			shortShot = true;
			spitShortSwap->Set(true);
			shooter->ShooterPrime(shortShot);
		}

		if (b_killShotL->ButtonClicked() || b_killShotR->ButtonClicked()) {
			shotKillSwitch = true;
			secondaryRollers->Undeploy();
		}

		if (b_notShooting->ButtonPressed() || b_notShooting2->ButtonPressed()) {
			shooter->currentlyShooting = false;
		}

		shooter->ShooterFire();

		if (b_shooterPrime->ButtonPressed()) {
			shooter->ShooterReturn();
		}

		if (b_pulseSecondary->ButtonPressed()) {
			if (b_pulseSecondary->ButtonClicked()) {
				secondaryRollers->Undeploy();
			}
			secondaryRollers->Pulse();
		}

		if (b_foldFlower->ButtonClicked()) {
			secondaryRollers->Undeploy();
			backIntake->UndeployIntake();
			frontIntake->UndeployIntake();
		}
		
		
		UpdateAllButtons();
	}
	void TestInit() {
		//gyro->CalibrateRate();
		leftEncoder->Reset();
		rightEncoder->Reset();
		//leftEncoder->Start();
		//rightEncoder->Start();
	}
	void TestPeriodic() {
		driverStationLCD->Printf((DriverStationLCD::Line) 0, 1,
				"Front Prox: %d", frontIntake->ProximityTriggered());
		driverStationLCD->Printf((DriverStationLCD::Line) 1, 1,
				"Back Prox: %d", backIntake->ProximityTriggered());
		driverStationLCD->Printf((DriverStationLCD::Line) 2, 1,
				"Hall Sensor: %d", shooter->HallSensorTriggered());
		driverStationLCD->Printf((DriverStationLCD::Line) 3, 1,
				"Left Encoder: %d", leftEncoder->Get());
		driverStationLCD->Printf((DriverStationLCD::Line) 4, 1,
				"Right Encoder: %d", rightEncoder->Get());
		/*printf("gyro: %f, gyroRate: %f, gyroFiltRate: %f, gyroCalRate: %f\n",
		 gyro->GetCalibratedAngle(), gyro->GetRate(),
		 gyro->GetFilteredRate(), gyro->GetCalibratedRate());*/
		if (b_frontIntakeDeployToggle->ButtonClicked()) {
			frontIntake->ToggleIntake();
			secondaryRollers->Undeploy();
			printf("Front Intake: %d", frontIntake->intakeDeployed);
		}

		if (b_backIntakeDeployToggle->ButtonClicked()) {
			backIntake->ToggleIntake();
			secondaryRollers->Undeploy();
			printf("Back Intake: %d", backIntake->intakeDeployed);
		}
		if (b_armPistonToggle->ButtonClicked()) {
			secondaryRollers->ToggleArms();
			printf("Secondarys Deployed: %d", secondaryRollers->DeployState());
		}
		if (b_toggleShotAlign->ButtonClicked()) {
			if (shortShot) {
				shortShot = false;
				shooter->ShooterPrime(shortShot);
				driverStationLCD->Printf((DriverStationLCD::Line) 0, 10,
						"Shot Alignment: %f", shooter->shotAlignerUp);
			} else {
				shortShot = true;
				shooter->ShooterPrime(shortShot);
			}
		}
		driverStationLCD->UpdateLCD();
	}

};

START_ROBOT_CLASS(Robot)
;
