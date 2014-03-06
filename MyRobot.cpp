#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "MPU6050_I2C.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"
//#include "Autonomous.h" 
#include "SecondaryRollerSystem.h"
#include "AutonomousRoutines.h"
#include "AutonomousSubroutines.h"
#include "Definitions.h"
//#include "CurrentSensor.h"

class Robot : public IterativeRobot
{
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;

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

	//Sensors
	Encoder *leftEncoder; //wheel rotation clicks
	Encoder *rightEncoder;
	//Gyro
	MPU6050_I2C *gyro;

	//Talons	
	RobotDrive *drivetrain; // robot drive system
	Talon *secondaryRollerA;
	Talon *secondaryRollerB;

	//Secondary Rollers
	SecondaryRollerSystem *secondaryRollers;

	//Timers.
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
	CitrusButton *b_gearToggle;
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
	CitrusButton *b_unfoldFlower;
	CitrusButton *b_foldFlower;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_armPistonToggle;
	CitrusButton *b_runSecondary;
	CitrusButton *b_killShotL;
	CitrusButton *b_killShotR;
	

	//Network Tables
	float Hthresh;
	float Sthresh;
	float Vthresh;
	float Hthreshlow;
	float Sthreshlow;
	float Vthreshlow;
	float startSide;
	float autoDirection;

public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");

		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();

		driverL = new Joystick(1);
		driverR = new Joystick(2);
		manipulator = new Joystick(3);

		dataTable = NetworkTable::GetTable("TurnTable");
		dataTable->PutNumber("degreeOfTurn", 90.0);
		dataTable->PutNumber("kpError", 2); //use 0.8 or 2
		dataTable->PutNumber("kiError", 0.023); //use 0.027 or 0.023
		dataTable->PutNumber("kdError", 0.5); //use 0.078 or 0.5
		
		table = NetworkTable::GetTable("datatable");
		table->PutNumber("Enabled", 0);

		compressor = new Compressor(1,1);

		//Solenoids.
		gearUp = new Solenoid(8); //TODO which is which?
		gearDown = new Solenoid (7);
		frontIntakeDeploy = new Solenoid (3); //deploy = putdown
		backIntakeDeploy = new Solenoid(4);
		shotAlignerUp = new Solenoid (5);
		shotAlignerDown = new Solenoid(6);
		armPiston = new Solenoid (2);
		//Sensors
		leftEncoder = new Encoder(6,7);
		rightEncoder = new Encoder(4,5);

		//gyro
		gyro = new MPU6050_I2C();
		drivetrain = new RobotDrive(3, 4);
		//Intake
		secondaryRollerA = new Talon (7);
		secondaryRollerB = new Talon (8);

		secondaryRollers = new SecondaryRollerSystem(secondaryRollerA, secondaryRollerB, armPiston);

		//Autonomous Timer
		autoTimer = new Timer();
		turnTimer = new Timer(); //starts && resets when we start gyroTurnAngle 
		shotTimer = new Timer();
		//Motors are on the wrong side of the intakes than we were expecting. 
		//They're on the opposite sides than before, thus everything was backwards
		//Ask Mike about it, but it is different from the other robot
		frontIntake = new IntakeSystem (6, secondaryRollerA, secondaryRollerB, 3,
				frontIntakeDeploy, true); //TODO was talon port 6. Swicthed for Fembots
		backIntake = new IntakeSystem (1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); //TODO was talon port 1. Switched for fembots
		
		gearToggle = false;
		frontIntakeToggle = false;
		backIntakeToggle = false;

		armPistonToggle = false;
		
		//Shooter
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston);

		shortShot = false;
		shotKillSwitch = false;

		//Buttons! 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons 
		b_gearToggle = new CitrusButton(k_bGearToggle);
		//Intake buttons
		b_frontIntakePickup = new CitrusButton (k_bfrontIntakePickup);
		b_backIntakePickup = new CitrusButton (k_bbackIntakePickup);
		b_frontIntakeDeployToggle = new CitrusButton (k_bfrontIntakeDeployToggle);
		b_backIntakeDeployToggle = new CitrusButton (k_bbackIntakeDeployToggle);
		b_reverseIntake = new CitrusButton (k_breverseIntake);
		b_humanLoad = new CitrusButton (k_bhumanLoad);
		b_runSecondary = new CitrusButton (k_brunSecondary);
		//Shooter buttons
		b_shortShoot = new CitrusButton (k_bshortShoot);
		b_longShoot = new CitrusButton (k_blongShoot);
		b_shotAlignLong = new CitrusButton(k_bshotAlignLong);
		b_shotAlignShort = new CitrusButton(k_bshotAlignShort);
		b_shooterPrime = new CitrusButton (k_bshooterPrime);
		b_pulseSecondary = new CitrusButton (k_bpulseSecondary);
		b_unfoldFlower = new CitrusButton (k_bunfoldFlower);
		b_foldFlower = new CitrusButton (k_bfoldFlower);
		b_armPistonToggle = new CitrusButton (k_barmPistonToggle);
		b_killShotL = new CitrusButton (k_bkillShotL);
		b_killShotR = new CitrusButton (k_bkillShotR);
	}

	void UpdateAllButtons()
	{
		//gearshift
		b_gearToggle->Update();
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
		b_unfoldFlower->Update();
		b_foldFlower->Update();
		b_armPistonToggle->Update();
		b_killShotL->Update();
		b_killShotR->Update();
	}

	void DisabledInit()
	{
		//TODO return for camera work
		//table->PutNumber("Enabled", 0);
		driverStationLCD->Clear();
		//leftEncoder->Stop();
		//rightEncoder->Stop();
		drivetrain->TankDrive(0.0, 0.0);
		frontIntakeDeploy->Set(frontIntake->DeployState());
		backIntakeDeploy->Set(backIntake->DeployState());
		armPiston->Set(secondaryRollers->DeployState());
	}
	void DisabledPeriodic()
	{
		//table->PutNumber("Enabled", 0);
		drivetrain->TankDrive(0.0, 0.0);
	}
	void AutonomousInit()
	{
		table->PutNumber("Enabled", 1);
		table->PutString("Direction: ", "MERP");
		leftEncoder->Start();
		rightEncoder->Start();
		gearDown->Set(true);
 
		//wisteria(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
		//	secondaryRollers, gyro, this);
		if (driverStation->GetDigitalIn(1))//Three ball auto starting on the left
		{	//TODO REDUCE TIME
			startSide = 1;
			table->PutNumber("Start Side", startSide);
			table->PutNumber("Enabled", 1);
			//autoDirection = ReceiveVisionProcessing(table);
			ThreeBallVisionRight(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
					turnTimer, secondaryRollers, gyro, this, table);
		}   
		else if (driverStation->GetDigitalIn(2))
		{
			//startSide = 2;
			//table->PutNumber("Start Side", startSide);
			//table->PutNumber("Enabled", 1);
			//autoDirection = ReceiveVisionProcessing(table);
			//TODO Right then left
			heliotrope(frontIntake, backIntake, shooter, drivetrain, autoTimer,
					turnTimer, secondaryRollers, gyro, 1.0, this);
		}
		else if (driverStation->GetDigitalIn(3))
		{
			//TODO Left then right
			heliotrope(frontIntake, backIntake, shooter, drivetrain, autoTimer,
					turnTimer, secondaryRollers, gyro, 2.0, this);
		}
		else if (driverStation->GetDigitalIn(4))
		{
			ShootLoadFrontAuto(frontIntake, backIntake, shooter, autoTimer,
					secondaryRollers, this, drivetrain);
			ShootAuto(frontIntake, backIntake, shooter, autoTimer,
					secondaryRollers, this);
		}
		else if (driverStation->GetDigitalIn(5))//Three ball auto starting on a side
		{
			ShootThreeAndDrive(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
					secondaryRollers, this);
		}
		else if (driverStation->GetDigitalIn(6))
		{
		 
			wisteria(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
					secondaryRollers, gyro, this);
		}
		else if (driverStation->GetDigitalIn(7))
		{
			ShootTwoThenOne(frontIntake, backIntake, shooter, drivetrain, autoTimer, turnTimer,
					secondaryRollers, this);
		}
		//TODO mobility function
		  
		//turnTimer->Start();
		//turnTimer->Reset(); 
		//frontIntake->DeployIntake();
		//backIntake->DeployIntake();
		OpenFlower(frontIntake, backIntake, secondaryRollers);
		Wait(0.5);
		frontIntake->FrontRollerAutoSlow();
		backIntake->BackRollerAutoSlow();
		GyroTurnAngle(this, gyro, drivetrain, -10.0, 2.0, 0.5, 0.585);
		Wait(2.0);
//		frontIntake->FrontRollerAutoSlow();
	//	backIntake->BackRollerAutoSlow();
		GyroTurnAngle(this, gyro, drivetrain, 20.0, 2.0, 0.355, 0.55);
		
		//GyroTurnAngle(this, gyro, drivetrain, dataTable);
		//printf("*********    Time: %f    ********    ", turnTimer->Get());
		
	}
	void AutonomousPeriodic()
	{
		printf("Gyro: %f, Gyro Rate: %f\n", gyro->GetAngle(), gyro->GetRate());
	}
	void TeleopInit()
	{
		
		compressor->Start();

		leftEncoder->Reset();
		rightEncoder->Reset();

		leftEncoder->Start();
		rightEncoder->Start();

		gyro->Reset();

		gearToggle = false;
		gearUp->Set(!gearToggle);
		gearDown->Set(gearToggle);
		
		//Photo to compare
		table->PutNumber("Enabled", 1);
	}
	void TeleopPeriodic()
	{
		dataTable->PutNumber("Enabled", 1);
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());

		//Drive.
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		/*if(manipulator->GetRawButton(12))
		{
			shotAligner
		}*/
		//Shift.
		if (b_gearToggle->ButtonClicked())
		{
			gearToggle = !gearToggle;
			gearUp->Set(!gearToggle);
			gearDown->Set(gearToggle);
		}
 
		if (b_frontIntakePickup->ButtonPressed())
		{
			//intakes to hold the ball using proxy sensors
			//frontIntake->Hold(manipulator);
			if (b_frontIntakePickup->ButtonClicked()&& manipulator->GetRawAxis(5) == 0.0)
			{
				frontIntake->DeployIntake();
			}
			//secondaryRollerA->Set(1.0);
			//secondaryRollerB->Set(-1.0);
			if (manipulator->GetRawAxis(5)!=0.0)
			{
				frontIntake->FrontRollerLoad();
			}
			else
			{
				frontIntake->FrontPickup(manipulator, driverStation);
			}
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
		}
		//activates pickup for back intake
		else if (b_backIntakePickup->ButtonPressed())
		{
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			if (b_backIntakePickup->ButtonClicked() && manipulator->GetRawAxis(5) == 0.0)
			{
				shortShot = true;
				shooter->ShooterPrime(shortShot);
				backIntake->DeployIntake();
			}
			if (manipulator->GetRawAxis(5)!=0.0)
			{
				backIntake->BackRollerLoad();
			}
			else
			{
				backIntake->Pickup(manipulator, driverStation);
			}
			secondaryRollers->Undeploy();
			secondaryRollers->Run();
		}
		else if (b_reverseIntake->ButtonPressed())
		{
			//intake reverses
			//frontIntake->Reverse();
			if (b_reverseIntake->ButtonClicked())
			{
				shortShot = true;
				shooter->ShooterPrime(shortShot);
			}
			secondaryRollers->Reverse();
			backIntake->Reverse();
		}
		else if (b_humanLoad->ButtonPressed())
		{
			HPReceive(secondaryRollers, frontIntake, backIntake);
		}
		else if (b_runSecondary->ButtonPressed())
		{
			secondaryRollers->Run();
		}
		else
		{
			//TODO: how will this affect the shooter?
			frontIntake->Stop();
			backIntake->Stop();
			secondaryRollers->Stop();
		}

		//toggles the front intake up and down
		if (b_frontIntakeDeployToggle->ButtonClicked())
		{
			frontIntake->ToggleIntake();
			secondaryRollers->Undeploy();
		}

		//toggles the back intake up and down
		if (b_backIntakeDeployToggle->ButtonClicked())
		{
			backIntake->ToggleIntake();
			secondaryRollers->Undeploy();
		}

		//armPistonToggle = Toggle(b_ArmPistonToggle, armPistonToggle);
		//Arm piston toggle
		if (b_armPistonToggle->ButtonClicked())
		{
			secondaryRollers->ToggleArms();
		} 

		if (b_shooterPrime->ButtonPressed())
		{
			shooter->ShooterReturn();
		}
		if (b_longShoot->ButtonClicked())
		{
			frontIntake->DeployIntake();
			backIntake->DeployIntake();
		}
		if (b_longShoot->ButtonReleased())
		{
			if (!shotKillSwitch)
			{
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
		}
		if (b_shortShoot->ButtonClicked())
		{
			frontIntake->DeployIntake();
			backIntake->DeployIntake();
		}
		if (b_shortShoot->ButtonReleased())
		{
			if (!shotKillSwitch)
			{
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
		}

		if (b_shotAlignLong->ButtonClicked())
		{
			shortShot = false;
			shooter->ShooterPrime(shortShot);
		}
		if (b_shotAlignShort->ButtonClicked())
		{
			shortShot = true;
			shooter->ShooterPrime(shortShot);
		}

		if (b_killShotL->ButtonClicked() || b_killShotR->ButtonClicked())
		{
			shotKillSwitch = true;
			frontIntake->UndeployIntake();
			backIntake->UndeployIntake();
		}

		shooter->ShooterFire();

		if (b_pulseSecondary->ButtonPressed())
		{
			if(b_pulseSecondary->ButtonClicked())
			{
				secondaryRollers->Undeploy();
			}
			secondaryRollers->Pulse();
		}

		if (b_unfoldFlower->ButtonClicked()) //All solenoids down
		{
			//Deploy side arms
			secondaryRollers->Deploy();
			backIntake->DeployIntake();
			frontIntake->DeployIntake();
		}
		if (b_foldFlower->ButtonClicked())
		{
			secondaryRollers->Undeploy();
			backIntake->UndeployIntake();
			frontIntake->UndeployIntake();
		}

		UpdateAllButtons();
	}
	void TestInit()
	{
		gyro->CalibrateRate();
	}
	void TestPeriodic()
	{
		printf("gyro: %f, gyroRate: %f, gyroFiltRate: %f, gyroCalRate: %f\n",
				gyro->GetCalibratedAngle(), gyro->GetRate(),
				gyro->GetFilteredRate(), gyro->GetCalibratedRate());
		drivetrain->TankDrive(0.0, 0.0); // robot drive system
		secondaryRollerA->Set(1.0);
		secondaryRollerB->Set(1.0);
		frontIntake->FrontRollerLoad();
		backIntake->BackRollerLoad();
		shooter->StopTalons();
	}

};

START_ROBOT_CLASS(Robot)
;
