#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "MPU6050_I2C.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"
#include "Autonomous.h" 
#include "SecondaryRollerSystem.h"
//#include "CurrentSensor.h"

class Robot : public IterativeRobot
{
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;

	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator; //gaming style controller
	
	NetworkTable *dataTable;

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
	CitrusButton *b_intakePickup;
	CitrusButton *b_frontIntakeDeployToggle;
	CitrusButton *b_backIntakeDeployToggle;
	CitrusButton *b_shoot;
	CitrusButton *b_shooterPrime;
	CitrusButton *b_pulseSecondary;
	CitrusButton *b_unfoldFlower;
	CitrusButton *b_foldFlower;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_holdOrPickup;
	CitrusButton *b_ArmPistonToggle;
	CitrusButton *b_runSecondary;
	CitrusButton *b_killShot;
	
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
		secondaryRollerB = new Talon(8);
		
		secondaryRollers = new SecondaryRollerSystem
			(secondaryRollerA, secondaryRollerB, armPiston);
		
		//Autonomous Timer
		autoTimer = new Timer(); 
		turnTimer = new Timer(); //starts && resets when we start gyroTurnAngle 
		shotTimer = new Timer();
		
		frontIntake = new IntakeSystem (6, secondaryRollerA, secondaryRollerB, 3, 
				frontIntakeDeploy, true); 
		backIntake = new IntakeSystem (1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); 
		
		//Shooter
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston);
		
		shortShot = false;
		shotKillSwitch = false;
		
		gearToggle = false;		
		frontIntakeToggle = false;
		backIntakeToggle = false;
		armPistonToggle = false;

		//Buttons. 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons
		b_gearToggle = new CitrusButton(driverL, 5);
		//Intake buttons
		b_intakePickup = new CitrusButton (manipulator, 6);
		b_frontIntakeDeployToggle = new CitrusButton (manipulator, 1);
		b_backIntakeDeployToggle = new CitrusButton (manipulator, 2);
		b_reverseIntake = new CitrusButton (manipulator, 3);
		b_holdOrPickup = new CitrusButton (manipulator, 8);
		b_humanLoad = new CitrusButton (manipulator, 4);
		//Shooter buttons
		b_shoot = new CitrusButton (driverR, 1);
		b_shotAlignLong = new CitrusButton(driverL, 2);
		b_shotAlignShort = new CitrusButton(driverL, 1);
		b_shooterPrime = new CitrusButton (manipulator, 11);
		b_pulseSecondary = new CitrusButton (manipulator, 5);
		b_unfoldFlower = new CitrusButton (driverR, 2);
		b_foldFlower = new CitrusButton (manipulator, 7);
		b_runSecondary = new CitrusButton (manipulator, 9);
		b_ArmPistonToggle = new CitrusButton (manipulator, 10);
		b_killShot = new CitrusButton (driverL, 3);
	}

	void UpdateAllButtons()
	{
		//gearshift
		b_gearToggle->Update();
		b_shotAlignLong->Update();
		b_shotAlignShort->Update();
		//intake
		b_intakePickup->Update();
		b_frontIntakeDeployToggle->Update();
		b_backIntakeDeployToggle->Update();
		b_reverseIntake->Update();
		b_holdOrPickup->Update();
		b_humanLoad->Update();
		//shooter
		b_shoot->Update();
		b_shooterPrime->Update();
		b_pulseSecondary->Update();
		b_unfoldFlower->Update();
		b_foldFlower->Update();
		b_ArmPistonToggle->Update();
		b_killShot->Update();
	}

	void DisabledInit()
	{
		driverStationLCD->Clear();
		//leftEncoder->Stop();
		//rightEncoder->Stop();
		drivetrain->TankDrive(0.0, 0.0);
		frontIntakeDeploy->Set(false);
		backIntakeDeploy->Set(false);
	}
	void DisabledPeriodic()
	{
		drivetrain->TankDrive(0.0, 0.0);
	}
	void AutonomousInit()
	{
		leftEncoder->Start();
		rightEncoder->Start();
		gearDown->Set(true);
		
		//wisteria(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
			//	secondaryRollers, gyro, this);
		if(driverStation->GetDigitalIn(1))
		{
			aubergine(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
					secondaryRollers, this);
		}
		else if(driverStation->GetDigitalIn(2))
		{
			heliotrope(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
					turnTimer, secondaryRollers, gyro, this); 
		}
		else if(driverStation->GetDigitalIn(3))
		{
			ShootAuto(frontIntake, backIntake, shooter, autoTimer, 
					secondaryRollers, this);
		}
		else if(driverStation->GetDigitalIn(4))
		{
			ShootLoadFrontAuto(frontIntake, backIntake, shooter, autoTimer, 
					secondaryRollers, this);
			ShootAuto(frontIntake, backIntake, shooter, autoTimer, 
					secondaryRollers, this);
		}
		//TODO mobility function
		//turnTimer->Start();
		//turnTimer->Reset();
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
	}
	void TeleopPeriodic()
	{
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());
		 
		//Drive.
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		//Shift.
		if (b_gearToggle->ButtonClicked())
		{
			gearToggle = !gearToggle;
			gearUp->Set(!gearToggle);
			gearDown->Set(gearToggle);
		}

		if (b_holdOrPickup->ButtonPressed())
		{
			//intakes to hold the ball using proxy sensors
			//frontIntake->Hold(manipulator);
			if(b_holdOrPickup->ButtonPressed())
			{
				frontIntake->DeployIntake();
			}
			frontIntake->FrontPickup(manipulator, driverStation); //TODO reorg
			//secondaryRollerA->Set(1.0);
			//secondaryRollerB->Set(-1.0);
			secondaryRollers->Run();
		}
		//activates pickup for back intake
		else if(b_intakePickup->ButtonPressed())  
		{
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			if(b_intakePickup->ButtonClicked())
			{
				shortShot = true;
				shooter->ShooterPrime(shortShot);
				backIntake->DeployIntake();
			}
			backIntake->Pickup(manipulator, driverStation);
			secondaryRollers->Run();
		}
		else if (b_reverseIntake->ButtonPressed())
		{
			//intake reverses
			//frontIntake->Reverse();
			secondaryRollers->Stop();
			backIntake->Reverse();
		}
		else if(b_humanLoad->ButtonPressed())
		{
			HPReceive(secondaryRollers, frontIntake, backIntake); 
		}
		else if(b_runSecondary->ButtonPressed())
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
		}

		//toggles the back intake up and down
		if (b_backIntakeDeployToggle->ButtonClicked()) 
		{
			backIntake->ToggleIntake();
		}
		
		armPistonToggle = Toggle(b_ArmPistonToggle, armPistonToggle);
		//Arm piston toggle
		if(b_ArmPistonToggle->ButtonClicked())
		{
			if(armPistonToggle)
			{
				armPiston->Set(true);
			}
			else if(!armPistonToggle)
			{
				armPiston->Set(false);
			}
		}
		
		if(b_shooterPrime->ButtonPressed())
		{
			shooter->ShooterReturn();
		}
		if(b_shoot->ButtonClicked())
		{
			frontIntake->DeployIntake();
			backIntake->DeployIntake();
		}
		if(b_shoot->ButtonReleased())
		{
			if(!shotKillSwitch)
			{
				secondaryRollers->Deploy();
				shooter->BeginShooterFire();
			}
			shotKillSwitch = false;
		}
		
		if(b_shotAlignLong->ButtonClicked())
		{
			shortShot = false;
			shooter->ShooterPrime(shortShot);
		}
		if(b_shotAlignShort->ButtonClicked())
		{
			shortShot = true;
			shooter->ShooterPrime(shortShot);
		}
		
		if(b_killShot->ButtonClicked())
		{
			shotKillSwitch = true;
			frontIntake->UndeployIntake();
			backIntake->UndeployIntake();
		}
		
		shooter->ShooterFire();
		
		if(b_pulseSecondary->ButtonPressed())
		{
			secondaryRollers->Pulse();
		}
		
		if(b_unfoldFlower->ButtonClicked()) //All solenoids down
		{
			//Deploy side arms
			armPiston->Set(true);
			backIntake->DeployIntake();
			frontIntake->DeployIntake();
		}
		if(b_foldFlower->ButtonClicked())
		{
			armPiston->Set(false);
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
				gyro->GetCalibratedAngle(), gyro->GetRate(), gyro->GetFilteredRate(), gyro->GetCalibratedRate());
	}
	
};
 
START_ROBOT_CLASS(Robot);
