#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "MPU6050_I2C.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"
#include "Autonomous.h" 
//#include "CurrentSensor.h"

class Robot : public IterativeRobot
{
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
	
	NetworkTable *dataTable;
	
	RobotDrive *drivetrain; // robot drive system
	Solenoid *gearUp; //compressed air
	Solenoid *gearDown;
	Encoder *leftEncoder; //wheel rotation clicks
	Encoder *rightEncoder;

	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator; //gaming style controller

	//Intakes.
	Solenoid *frontIntakeDeploy;
	Solenoid *backIntakeDeploy;
	Talon *secondaryRollerA;
	Talon *secondaryRollerB;
	IntakeSystem *frontIntake;
	IntakeSystem *backIntake;

	//Autonomous Timer
	Timer *autoTimer;
	
	//Gyro
	MPU6050_I2C *gyro;
	
	//Intake Toggle
	bool frontIntakeToggle;
	bool backIntakeToggle;
	//Arm piston.
	bool armPistonToggle;
	
	Timer *turnTimer;
	//Shooting
	ShooterSystem *shooter;
	Compressor *compressor;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;
	Solenoid *armPiston;
	bool shortShot;

	//Buttons!
	CitrusButton *b_gearUp;
	CitrusButton *b_gearDown;
	CitrusButton *b_IntakePickup;
	CitrusButton *b_frontIntakeDeployToggle;
	CitrusButton *b_backIntakeDeployToggle;
	CitrusButton *b_shoot;
	CitrusButton *b_shooterPrime;
	CitrusButton *b_shotAlignerToggle;
	CitrusButton *b_unfoldFlower;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_holdOrPickup;
	CitrusButton *b_ArmPistonToggle;
	CitrusButton *b_runSecondary;
	
public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");
		
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		
		dataTable = NetworkTable::GetTable("TurnTable");
		dataTable->PutNumber("degreeOfTurn", 90.0);
		dataTable->PutNumber("kpError", 2); //use 0.8 or 2
		dataTable->PutNumber("kiError", 0.023); //use 0.027 or 0.023
		dataTable->PutNumber("kdError", 0.5); //use 0.078 or 0.5
		
		//remove todos as the correct values are found
		drivetrain = new RobotDrive(3, 4);
		gearUp = new Solenoid(8); //TODO which is which?
		gearDown = new Solenoid (7);

		driverL = new Joystick(1);
		driverR = new Joystick(2);
		manipulator = new Joystick(3);

		//gyro
		gyro = new MPU6050_I2C();

		//Autonomous Timer
		autoTimer = new Timer(); 
		
		//Solenoids.
		frontIntakeDeploy = new Solenoid (3); //deploy = putdown
		backIntakeDeploy = new Solenoid(4);

		//Intake
		secondaryRollerA = new Talon (7); 
		secondaryRollerB = new Talon(8);
		frontIntake = new IntakeSystem (6, secondaryRollerA, secondaryRollerB, 3, 
				frontIntakeDeploy, true); 
		backIntake = new IntakeSystem (1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); 

		compressor = new Compressor(1,1);
		
		turnTimer = new Timer(); //starts && resets when we start gyroTurnAngle 

		//TODO fix intake pointers in shooter?
		leftEncoder = new Encoder(6,7);
		rightEncoder = new Encoder(4,5);

		armPiston = new Solenoid (2); //TODO number?
		
		//Shooter
		shotAlignerUp = new Solenoid (5);
		shotAlignerDown = new Solenoid(6);
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston); //TODO numbers
		shortShot = false;

		frontIntakeToggle = false;
		backIntakeToggle = false;
		armPistonToggle = false;
				
		//Buttons. 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons
		b_gearUp = new CitrusButton(driverL, 1);
		b_gearDown = new CitrusButton(driverL, 2);
		//Intake buttons
		b_IntakePickup = new CitrusButton (manipulator, 6);//TODO numbers
		b_frontIntakeDeployToggle = new CitrusButton (manipulator, 1);
		b_backIntakeDeployToggle = new CitrusButton (manipulator, 2);
		b_reverseIntake = new CitrusButton (manipulator, 3);
		b_holdOrPickup = new CitrusButton (manipulator, 8);
		b_humanLoad = new CitrusButton (manipulator, 4);
		//Shooter buttons
		b_shoot = new CitrusButton (driverR, 1);
		b_shooterPrime = new CitrusButton (manipulator, 11);
		b_shotAlignerToggle = new CitrusButton (manipulator, 5);
		b_unfoldFlower = new CitrusButton (driverR, 2);
		b_runSecondary = new CitrusButton (manipulator, 9);
		//Temporary.
		b_ArmPistonToggle = new CitrusButton (manipulator, 10);
	}

	void UpdateAllButtons()
	{
		//gearshift
		b_gearUp->Update();
		b_gearDown->Update();
		//intake
		b_IntakePickup->Update();
		b_frontIntakeDeployToggle->Update();
		b_backIntakeDeployToggle->Update();
		b_reverseIntake->Update();
		b_holdOrPickup->Update();
		b_humanLoad->Update();
		//shooter
		b_shoot->Update();
		b_shooterPrime->Update();
		b_shotAlignerToggle->Update();
		b_unfoldFlower->Update();
		//temp
		b_ArmPistonToggle->Update();
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
		//frontIntakeDeploy->Set(true);
		//backIntakeDeploy->Set(true);

		aubergine(frontIntake, backIntake, shooter, drivetrain, autoTimer, 
				armPiston, this);

		//turnTimer->Start();
		//turnTimer->Reset();
		//GyroTurnAngle(this, gyro, drivetrain, leftEncoder, rightEncoder, dataTable);
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
	}
	void TeleopPeriodic()
	{
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());
		 
		//Drive.
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		//Shift.
		if (b_gearUp->ButtonClicked())
		{
			gearUp->Set(true);
			gearDown->Set(false);
			driverStationLCD->PrintfLine((DriverStationLCD::Line)0, "High gear");
		}
		else if (b_gearDown->ButtonClicked())
		{
			gearUp->Set(false);
			gearDown->Set(true);
			driverStationLCD->PrintfLine((DriverStationLCD::Line)0, "Low gear ");
		}

		if (b_holdOrPickup->ButtonPressed())
		{
			//intakes to hold the ball using proxy sensors
			//frontIntake->Hold(manipulator);
			frontIntake->FrontPickup(manipulator, driverStation); //TODO reorg
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(-1.0);
			//backIntake->Hold(manipulator);
		}
		//activates pickup for back intake
		else if(b_IntakePickup->ButtonPressed())  
		{
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			backIntake->Pickup(manipulator, driverStation);
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(-1.0);
		}
		else if (b_reverseIntake->ButtonPressed())
		{
			//intake reverses
			//frontIntake->Reverse();
			secondaryRollerA->Set(0.0);
			secondaryRollerB->Set(0.0);
			backIntake->Reverse();
		}
		else if(b_humanLoad->ButtonPressed())
		{
			HPReceive(secondaryRollerA, secondaryRollerB, frontIntake, backIntake); 
		}
		else if(manipulator->GetRawButton(9))
		{
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(-1.0);
		}
		else
		{
			//TODO: how will this affect the shooter?
			frontIntake->Stop();
			backIntake->Stop();
			frontIntake->StopSecondaryRollers();
			secondaryRollerA->Set(0.0);
			secondaryRollerB->Set(0.0);
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
			shooter->BeginShooterFire(); //if called then Fire runs
		}
		shooter->ShooterFire();
		
		if(b_shotAlignerToggle->ButtonClicked())
		{
			shortShot = !shortShot;
			shooter->ShooterPrime(shortShot);
			driverStationLCD->PrintfLine((DriverStationLCD::Line)1, "short: %d", shortShot);
		}
		
		if(b_unfoldFlower->ButtonClicked()) //All solenoids down
		{
			//Deploy side arms
			armPiston->Set(true);
			backIntake->DeployIntake();
			frontIntake->DeployIntake();
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
