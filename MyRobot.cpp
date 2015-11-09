#include "WPILib.h" //Lots of convenient robot classes
#include "NetworkTables/NetworkTable.h" //Networktables used for vision
#include "Drivetrain.h" //Drivetrain Header File
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
	
	
	///////////////////////////
	//
	//	  ROBOT DECLARATION
	//
	///////////////////////////
	
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
	
	DigitalOutput *sensor;

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
	bool bumperShot;

	//Random number generator 
	bool randomNum;
	
	//PID
	CitrusPID *PID;

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
	CitrusButton *b_reverseHumanLoad;

public:
	
	
	///////////////////////////
	//
	//	ROBOT INITIALIZATION
	//
	///////////////////////////
	
	Robot() {
		printf("Robot INITIALIZATION\n");

		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();

		sensor = new DigitalOutput(3);
		
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

		//Solenoids. Compressed air.
		gearUp  		  = new Solenoid(8);
		gearDown          = new Solenoid(7);
		frontIntakeDeploy = new Solenoid(3); //deploy = put intake down
		backIntakeDeploy  = new Solenoid(4);
		shotAlignerUp     = new Solenoid(5);
		shotAlignerDown   = new Solenoid(6);
		armPiston         = new Solenoid(2); // Controls both secondary roller pistons. Toggles them up and down
		spitShortSwap     = new Solenoid(1);

		//Sensors 
		/*
		 * Ports for the encoders (in relation to the drivetrain motors)
		 * Left- 4,5 for competition robot, 7,6 for the practice robot
		 * Right- 7,6 for competition robot, 4,5 for the practice robot
		 */
		leftEncoder  = new Encoder(4, 5); 
		rightEncoder = new Encoder(7, 6);

		//Begins counting of encoder clicks for both drivetrains
		leftEncoder->Start();
		rightEncoder->Start();

		
		drivetrain = new RobotDrive(3, 4); //Ports for the motors on the drivetrain
		//SecondaryIntake
		secondaryRollerA = new Talon(7); //Port for (most likely) the left secondary
		secondaryRollerB = new Talon(8); //Port for (most likely) the right secondary

		// Creating the class object for the secondary rollers. 
		secondaryRollers = new SecondaryRollerSystem(secondaryRollerA,
				secondaryRollerB, armPiston);

		//Autonomous Timer
		autoTimer2 = new Timer();
		autoTimer  = new Timer();
		turnTimer  = new Timer(); //starts && resets when we start gyroTurnAngle 
		shotTimer  = new Timer();

		// The class object for the front intake. 
		frontIntake = new IntakeSystem(6, secondaryRollerA, secondaryRollerB,
				14, frontIntakeDeploy, true);
		// Class object for the front intake.
		backIntake  = new IntakeSystem(1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); 

		// Set all the solenoids 
		gearToggle = false;
		frontIntakeToggle = false;
		backIntakeToggle = false;
		armPistonToggle = false;

		//Shooter
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston);

		// Booleans 
		shortShot      = false;
		shotKillSwitch = false;
		bumperShot     = false;
		randomNum      = false;
		
		//PID
		PID = new CitrusPID();

		//Buttons 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons 
		b_gearUp				  = new CitrusButton(driverL, 2);
		b_gearDown				  = new CitrusButton(driverR, 2);
		b_testGearToggle		  = new CitrusButton(k_btestGearToggle);
		//Intake buttons
		b_frontIntakePickup 	  = new CitrusButton(k_bfrontIntakePickup);
		b_backIntakePickup		  = new CitrusButton(k_bbackIntakePickup);
		b_frontIntakeDeployToggle = new CitrusButton(k_bfrontIntakeDeployToggle);
		b_backIntakeDeployToggle  = new CitrusButton(k_bbackIntakeDeployToggle);
		b_reverseIntake    		  = new CitrusButton(k_breverseIntake);
		b_humanLoad 			  = new CitrusButton(k_bhumanLoad);
		b_runSecondary 			  = new CitrusButton(k_brunSecondary);
		//Shooter buttons
		b_shotAlignLong			  = new CitrusButton(k_bshotAlignLong);
		b_shotAlignShort		  = new CitrusButton(k_bshotAlignShort);
		b_shortShoot 			  = new CitrusButton(k_bshortShoot);
		b_longShoot 			  = new CitrusButton(k_blongShoot);
		b_shooterPrime 			  = new CitrusButton(k_bshooterPrime);
		b_pulseSecondary 		  = new CitrusButton(k_bpulseSecondary);
		b_foldFlower 			  = new CitrusButton(k_bfoldFlower);
		b_armPistonToggle 		  = new CitrusButton(k_barmPistonToggle);
		b_killShotL				  = new CitrusButton(k_bkillShotL);
		b_killShotR 		 	  = new CitrusButton(k_bkillShotR);
		b_toggleShotAlign		  = new CitrusButton(k_btoggleShotAlign);
		b_notShooting 			  = new CitrusButton(driverR, 8);
		b_notShooting2 			  = new CitrusButton(driverR, 9);
		b_reverseHumanLoad 	 	  = new CitrusButton(k_breverseHumanLoad);
	}

	
	
	///////////////////////////
	//
	//	UPDATE ALL BUTTONS
	//
	///////////////////////////
	
	
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
		b_reverseHumanLoad->Update();
		//i hate everything
		b_frontIntakePickup->Update(manipulator->GetRawAxis(3) > 0.5);
		b_backIntakePickup->Update(manipulator->GetRawAxis(3) < -0.5);
	}

	
	
	///////////////////////////
	//
	//	DISABLED MODE
	//
	///////////////////////////
	
	
	void DisabledInit() {
		
		bumperShot = false;
		
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
		bumperShot = false; 
	}
	
	
	
	///////////////////////////
	//
	//	AUTONOMOUS MODE
	//
	///////////////////////////
	
	
	void AutonomousInit() {
		
		spitShortSwap->Set(true); //fingers on the shooter used to controll the ball
		driverStationLCD->Clear(); // Clears all user messages from the driver console
		driverStationLCD->UpdateLCD(); // Must update for changes to the driver console to apply
		//table->PutNumber("Enabled", 1); //TODO needs to be added back in for vision processing
		randomNum = true;
		
		//Random Number generator!
		//if ((1 + (rand() % 20)) % 2 ) { //TODO add in == 0 to make sure that it actually varies
			randomNum = true; // If it's an even number, the routine will go right, then left
		//}
		//else {
			//randomNum = false; // If it's an odd number, the routine will gp left, then right
		//}
		
		// Clear the driver station before autonomous
		driverStationLCD->Clear();
		driverStationLCD->Printf((DriverStationLCD::Line) 1, 2, "%d", randomNum);
		driverStationLCD->UpdateLCD();

		
		//MERP so that we get an error if timing with network tables doesn't work
		table->PutString("Direction: ", "MERP"); 
		
		gearDown->Set(true); // Start in low gear
		gearUp->Set(false);
		
		bool allDone; // Loop control for the auto loops. 
		
		// Start encoders at 0.0
		rightEncoder->Reset();
		leftEncoder->Reset();
		
		
		//beginning of long chain to determine which auto we are running
		//for running motors: -1 to 1 input range
		//This applies to several of these: sorry for messy code, not all of them work
		/*
		 * 
		 * Auto 1: Long shot from distance, short shot. A goalie avoidance for 2 ball. Random.
		 * 		Start on 
		 * Auto 2: 3 ball auto. Needs to be tested. 
		 * Auto 3: Long shot close up, and short shot. Goalie 2 ball avoidance. Random.
		 * Auto 4: One ball vision, straight. 
		 * Auto 5: 3 ball auto, working. Testing?
		 * Auto 6: 2 ball vision. Start on the left side of the field.
		 * Auto 7: Straight 2 ball. Starting point doesn't matter.
		 * Auto 8: Random juking 2 shot. Starts in the middle of the field, chooses a direction.
		 * 
		 * 
		 */
		// TODO needs to be tested
		if (driverStation->GetDigitalIn(1)) // AUTO NUMBER 1
		{
			printf("in DIO 1\n");
			RightLeftShot(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder,
					leftEncoder, shotTimer, drivetrain, spitShortSwap, driverStation, randomNum);
			
//			LongShortRandomGoalie (backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers,
//					allDone, rightEncoder, leftEncoder, shotTimer, drivetrain, driverStation, 
//					randomNum, spitShortSwap); //goalie avoidance 2 ball
		} 
		
		// TODO Test
		else if (driverStation->GetDigitalIn(2)) // AUTO NUMBER 2
		{ 
//			ThreeBallStraight(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, 
//					rightEncoder, leftEncoder, shotTimer, drivetrain, driverStation, spitShortSwap);
			LeftRightShot(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder,
					leftEncoder, shotTimer, drivetrain, driverStation, randomNum, spitShortSwap);
						
		} 
		
		// TODO needs to be tested
		else if (driverStation->GetDigitalIn(3)) { // AUTO NUMBER 3
			
			//LeftRightLeftShot(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder,
				//	leftEncoder, shotTimer, drivetrain, driverStation, randomNum, spitShortSwap);
			LeftRightLeftShot(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder,
					leftEncoder, shotTimer, drivetrain, spitShortSwap, driverStation, randomNum);
									
			
			//CloseLongShortRandomGoalie(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone,
			//		rightEncoder, leftEncoder, shotTimer, drivetrain, spitShortSwap, driverStation, randomNum);
			
			
		} 
		// TODO Reliable. One shot hot. 
		else if (driverStation->GetDigitalIn(4)) { // AUTO NUMBER 4
			printf("In GetDigitalIn 4");
			OneShotShort(frontIntake, backIntake, shooter, drivetrain,
					autoTimer, turnTimer, spitShortSwap, secondaryRollers,
					this, rightEncoder, driverStationLCD, table, 1.0);
			
		} 
		
		// TODO Test
		else if (driverStation->GetDigitalIn(5)) { // AUTO NUMBER 5
			DriveForward(frontIntake, backIntake, shooter, drivetrain, autoTimer, shotTimer, spitShortSwap, 
					secondaryRollers, this, rightEncoder, driverStationLCD, table, 1.0);
//			ThreeBallVision(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone,
//					rightEncoder, leftEncoder, shotTimer, drivetrain, driverStation, spitShortSwap, table, 
//					driverStationLCD);
		} 
		
		// TODO Test
		else if (driverStation->GetDigitalIn(6)) { // AUTO NUMBER 6
			float direction = 1.0; //TODO Put back to receive vision processing and not manualy set the hot goal
			TwoShotShortVision(frontIntake, backIntake, shooter,
					drivetrain, autoTimer, autoTimer2,spitShortSwap,
					secondaryRollers, this,rightEncoder, leftEncoder, driverStation, direction, table);
		} 
		
		// TODO Reliable
		else if (driverStation->GetDigitalIn(7)) { // AUTO NUMBER 7
			printf("In GetDigitalIn 7");
			TwoShotShortShort(frontIntake, backIntake, shooter, drivetrain,
					autoTimer, shotTimer, spitShortSwap, secondaryRollers,
					this, rightEncoder, driverStation);
		} 		
		
		// TODO needs to be tested
		 else if (driverStation->GetDigitalIn(8)) // AUTO NUMBER 8
		 {
			 RightLeftRightShot (backIntake, frontIntake, autoTimer, shooter, this,
					 secondaryRollers, allDone, rightEncoder, leftEncoder, shotTimer, drivetrain,
					 spitShortSwap, randomNum, driverStation);
			 //ThreeShotGoalieLeftRight(backIntake, frontIntake, autoTimer, shooter, this, secondaryRollers, allDone, rightEncoder, shotTimer, drivetrain, spitShortSwap);
		 }
	}
	
	
	
	// Runs all throughout autonomous
	void AutonomousPeriodic() {
		//print this for troubleshooting
		//printf("Gyro: %f, Gyro Rate: %f\n", gyro->GetAngle(), gyro->GetRate());
	}
	
	
	
	///////////////////////////
	//
	//	TELEOPERTAED MODE
	//
	///////////////////////////
	
	
	
	// Runs once at the beginning of teleop
	void TeleopInit() {
			
		spitShortSwap->Set(true);
		bumperShot = false;
		
		compressor->Start();


		gearToggle = false;
		gearUp->Set(!gearToggle);
		gearDown->Set(gearToggle);

	}
	
	
	// Runs all throughout teleop
	void TeleopPeriodic() {
		
		// For the thermal sensors (not in use at the moment)
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
		//For troubleshooting
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());


		//Drive.
		
		// Normal drive code, ramps up and down accordingly
		//runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		
		// The shift code allows for the drivetrain to auto shift to avoid blowing the main breakrer
		runDrivetrainShift(driverL->GetY(), driverR->GetY(), drivetrain, 0.2, gearUp, gearDown, leftEncoder, rightEncoder);
		
		// Pring the voltage for testing the shifting code
		//currentSensor->VoltageMonitor(gearUp, gearDown, currentSensor, a, driverStationLCD);

		
		
		// Prints the encoder values so Brycen can't say it's a programming issue. 
		driverStationLCD->Printf((DriverStationLCD::Line) 3, 1,
				"Left Encoder: %f", leftEncoder->GetRate());
		driverStationLCD->Printf((DriverStationLCD::Line) 4, 1,
				"Right Encoder: %f", rightEncoder->GetRate());
		driverStationLCD->Printf((DriverStationLCD::Line) 2, 1, 
				"%f", manipulator->GetRawAxis(3));
		driverStationLCD->UpdateLCD();
		
		
		
		// BUTTONS! 
		// Controls everything else besides the drivetrain. 
		
		
		// GEARS
		// Gearshifting buttons. Are named accordingly
		if (b_gearUp->ButtonClicked()) {
			// Gear up
			gearUp->Set(false);
			gearDown->Set(true);
		}
		if (b_gearDown->ButtonClicked()) {
			// Gear down
			gearUp->Set(true);
			gearDown->Set(false);
		}

		
		// INTAKES RUNNING
		// Intaking and reversing, including human loading
		// Includes secondary rollers
		
		// While the button is held down, load through the front intake
		if (b_frontIntakePickup->ButtonPressed(manipulator->GetRawAxis(3) > 0.5)) {
			
			// If the override isn't being held down, deploy the intake
			if (b_frontIntakePickup->ButtonClicked(manipulator->GetRawAxis(3) > 0.5)
					&& manipulator->GetRawAxis(6) < 0.5) {
				frontIntake->DeployIntake();
			}
			// Set the fingers so the ball can settle correctly
			if (b_frontIntakePickup->ButtonClicked(manipulator->GetRawAxis(3) > 0.5)) {
				spitShortSwap->Set(true);
			}
			// If the override is being used, ingore the proxy and manually toggle the intake up
			if (manipulator->GetRawAxis(6) > 0.5) {
				frontIntake->FrontRollerLoad();
			} 
			// If no override, automatically bring intake up once proxy is triggered
			else {
				frontIntake->FrontPickup(driverStation);
			}
			
			// Bring secondary rollers in, and run them to suck down the ball
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
			
			bumperShot = false;
		}
		// While this button is held down, load through the back intake
		else if (b_backIntakePickup->ButtonPressed(manipulator->GetRawAxis(3) < -0.5)) {
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			
			// If the override isn't being held down, deploy the intake 
			if (b_backIntakePickup->ButtonClicked(manipulator->GetRawAxis(3) < -0.5) && manipulator->GetRawAxis(6) < 0.5) {
				// This so the stops go down and the ball can load
				shortShot = true;
				shooter->ShooterPrime(shortShot);

				backIntake->DeployIntake();

				// Also set so the ball can load over the catapult
				spitShortSwap->Set(true);
			}
			
			// If the override is being held, ignore the proxy and manually toggle the intake up
			if (manipulator->GetRawAxis(6) > 0.5) {
				backIntake->BackRollerLoad();
			} 
			// If no override, automatically bring the intake up once proxy is triggered
			else {
				backIntake->Pickup(manipulator, driverStation);
			}
			// Bring the secondary rollers in and run them to suck the ball down
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
			bumperShot = false;
		} 
		// Reverses the back intake to spit the ball back
		else if (b_reverseIntake->ButtonPressed()) 
		{
			
			if (b_reverseIntake->ButtonClicked()) {
				// Set the fingers down so the ball can roll over the catapult
				shortShot = true;
				shooter->ShooterPrime(shortShot);
				spitShortSwap->Set(false);
			}
			// Run the secondary rollers opposite to spin the ball
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(1.0);
			// Run the back intake in reverse
			backIntake->Reverse();
			bumperShot = false;
		} 
		else if (b_humanLoad->ButtonPressed()) 
		{
			if(b_reverseHumanLoad->ButtonPressed())
			{
				// If HPReverse has been triggered, pop the ball out the top
				HPReverse(secondaryRollers, frontIntake, backIntake, shooter);
				// Set true for the bank/bumper shot
				bumperShot = true;
			}
			else
			{
				// Otherwise, just suck the ball down from the top
				HPReceive(secondaryRollers, frontIntake, backIntake, shooter);
				bumperShot = false;
			}
		}
		else if (b_runSecondary->ButtonPressed()) 
		{
			// Suck the ball down
			secondaryRollers->Run();
			bumperShot = false;
		} 
		else 
		{
			// Don't run anything at all!
			frontIntake->Stop();
			backIntake->Stop();
			secondaryRollers->Stop();
		}

		
		// Set the long fingers back up
		if (b_reverseIntake->ButtonReleased()) {
			spitShortSwap->Set(true);
		}

		
		
		// INTAKES TOGGLING
		// Controls teh intakes moving up and down
		
		// Toggles the intakes up and down
		if (b_frontIntakeDeployToggle->ButtonClicked()) 
		{
			bumperShot = false;
			frontIntake->ToggleIntake();
			if (!frontIntake->DeployState()) 
			{
				secondaryRollers->Undeploy();
			}
		}

		//toggles the back intake up and down
		if (b_backIntakeDeployToggle->ButtonClicked()) 
		{
			bumperShot = false;
			backIntake->ToggleIntake();
			if (!frontIntake->DeployState()) // TODO check to see if this should be back intake
			{
				secondaryRollers->Undeploy();
			}
		}
		
		
		// SECONDARY ROLLERS
		
		// Toggle the secondary rollers
		if (b_armPistonToggle->ButtonClicked()) 
		{
			
			secondaryRollers->ToggleArms();
		}
		
		// Pulse the rollers to properly settle the ball
		if (b_pulseSecondary->ButtonPressed()) {
			if (b_pulseSecondary->ButtonClicked()) {
				// Bring the intakes in if they aren't already
				secondaryRollers->Undeploy();
			}
			// Pulse the intakes, basicaly running them on at off at 0.2 sec intervals
			secondaryRollers->Pulse();
			bumperShot = false;
		}

		
		// SHOOTER
		
		// For the long shot
		if (b_longShoot->ButtonClicked()) 
		{
			// If true, prep for bumper shot and DON'T set the intakes down. They hold the ball
			if(bumperShot)
			{
				//spitShortSwap->Set(false);
				frontIntake->UndeployIntake();
				backIntake->UndeployIntake();
			}
			// Otherwise set the down because the ball is settled
			else
			{
				frontIntake->DeployIntake();
				backIntake->DeployIntake();
			}
		}
		
		// Once released, begin to shoot
		if (b_longShoot->ButtonReleased()) 
		{
			if (bumperShot)
			{
				secondaryRollers->Undeploy();
				backIntake->Reverse();
				//frontIntake->FrontRollerLoad();
				//spitShortSwap->Set(false);
			}
			if (!shotKillSwitch) 
			{
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			
			shotKillSwitch = false;
			bumperShot = false;
		}
		
		// Set down the intakes for the short shot
		if (b_shortShoot->ButtonClicked()) 
		{	
			frontIntake->DeployIntake();
			backIntake->DeployIntake();	
			bumperShot = false;
		}
		
		// Once released, begin to shoot
		if (b_shortShoot->ButtonReleased()) 
		{
			if (!shotKillSwitch) 
			{
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
			bumperShot = false;
		}

		// Prepping the catapult for a long shot
		if (b_shotAlignLong->ButtonClicked()) {
			// For the bumper shot
			if(bumperShot){
				// Set all of the fingers up 
				shooter->ShooterPrime(false);
				spitShortSwap->Set(true);
			}
			else {
				// Set the longest fingers up, shorter ones down
				shortShot = false;
				shooter->ShooterPrime(shortShot);
				spitShortSwap->Set(false);
			}	
		}
		
		// Prepping the catapult for a short shot
		if (b_shotAlignShort->ButtonClicked()) {
			shortShot = true;
			spitShortSwap->Set(true);
			shooter->ShooterPrime(shortShot);
		}

		// If the kill shot is pressed, CANCEL THE SHOT!
		if (b_killShotL->ButtonClicked() || b_killShotR->ButtonClicked()) {
			shotKillSwitch = true;
			bumperShot = false;
			secondaryRollers->Undeploy();
		}

		// Set this so the catapult stays up and doesn't retract automatically
		// aka, the "fire and disable"
		if (b_notShooting->ButtonPressed() || b_notShooting2->ButtonPressed()) {
			shooter->currentlyShooting = false;
		}

		
		
		// ACTUAL SHOT
		shooter->ShooterFire();

		
		
		// Retract shooter
		// Run the talons so the shooter is brought back automatically
		// Use after on re-enabling after shooting at disabling 
		if (b_shooterPrime->ButtonPressed()) {
			shooter->ShooterReturn();
		}

		
		// Bring everything in 
		if (b_foldFlower->ButtonClicked()) {
			secondaryRollers->Undeploy();
			backIntake->UndeployIntake();
			frontIntake->UndeployIntake();
		}
		
		// UPDATE ALL THE BUTTONS
		UpdateAllButtons();
	}
	
	///////////////////////////
	//
	//	TEST MODE
	//
	///////////////////////////
	
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
