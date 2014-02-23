#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "MPU6050_I2C.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"
#include "Autonomous.h" 

class Robot : public IterativeRobot
{
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

	//Gyro
	MPU6050_I2C *gyro;
	
	//Intake Toggle
	bool frontIntakeToggle;
	bool backIntakeToggle;
	//Arm piston. Temporary.
	bool tarmPistonToggle;
	
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
	CitrusButton *b_return;
	CitrusButton *b_primeShortShot;
	CitrusButton *b_primeLongShot;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_hold;
	CitrusButton *b_shooterSystemReset;
	CitrusButton *b_tArmPistonToggle;

public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");
		
		dataTable = NetworkTable::GetTable("TurnTable");
		dataTable->PutNumber("degreeOfTurn", 90.0);
		dataTable->PutNumber("kpError", 2);
		dataTable->PutNumber("kiError", 0.05);
		dataTable->PutNumber("kdError", 0.001);
		
		//remove todos as the correct values are found
		drivetrain = new RobotDrive(3, 4);
		gearUp = new Solenoid(8); //TODO which is which?
		gearDown = new Solenoid (7);

		driverL = new Joystick(1);
		driverR = new Joystick(2);
		manipulator = new Joystick(3);

		//gyro
		gyro = new MPU6050_I2C();

		//Solenoids.
		frontIntakeDeploy = new Solenoid (3); //deploy = putdown
		backIntakeDeploy = new Solenoid(4);

		//Intake
		secondaryRollerA = new Talon (7); //TODO value?>
		secondaryRollerB = new Talon(8);
		frontIntake = new IntakeSystem (6, secondaryRollerA, secondaryRollerB, 3, 
				frontIntakeDeploy, true); //TODO numbers
		backIntake = new IntakeSystem (1, secondaryRollerA, secondaryRollerB, 2,
				backIntakeDeploy, false); //TODO numbers

		compressor = new Compressor(1,1);

		//TODO fix intake pointers in shooter?
		leftEncoder = new Encoder(6,7);
		rightEncoder = new Encoder(4,5);

		armPiston = new Solenoid (3); //TODO number?
		//Shooter
		shooter = new ShooterSystem(2, 5, 8, shotAlignerUp, shotAlignerDown,
				armPiston, frontIntakeDeploy,backIntakeDeploy); //TODO numbers
		shortShot = false;

		frontIntakeToggle = false;
		backIntakeToggle = false;
		tarmPistonToggle = false;
		
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
		b_hold = new CitrusButton (manipulator, 8);
		b_humanLoad = new CitrusButton (manipulator, 4);
		//Shooter buttons
		b_shoot = new CitrusButton (driverR, 1);
		b_return = new CitrusButton (driverR, 3);
		b_primeShortShot = new CitrusButton (manipulator, 5);
		b_primeLongShot = new CitrusButton (manipulator, 7);
		b_shooterSystemReset = new CitrusButton(manipulator, 9);
		//Temporary.
		b_tArmPistonToggle = new CitrusButton (manipulator, 10);
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
		b_hold->Update();
		b_humanLoad->Update();
		//shooter
		b_shoot->Update();
		b_return->Update();
		b_primeShortShot->Update();
		b_primeLongShot->Update();
		b_shooterSystemReset->Update();
		//temp
		b_tArmPistonToggle->Update();
	}

	void DisabledInit()
	{
		//leftEncoder->Stop();
		//rightEncoder->Stop();

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
		
		//printf("AUTO INIT\n");
		GyroTurnAngle(this, gyro, 90.0, drivetrain, leftEncoder, rightEncoder, dataTable);
	}
	void AutonomousPeriodic()
	{
		printf("Gyro: %f Gyro Rate: %f\n", gyro->GetAngle(), gyro->GetRate());

	}
	void TeleopInit()
	{
		compressor->Start();

		leftEncoder->Reset();
		rightEncoder->Reset();

		leftEncoder->Start();
		rightEncoder->Start();

		gyro->Reset();
		//printf("TELEOP INIT\n");
	}
	void TeleopPeriodic()
	{
		//printf("Left Encoder: %d Right Encoder: %d", leftEncoder->Get(), rightEncoder->Get());
		//printf("Front prox: %d, back prox: %d\n", frontIntake->ProximityTriggered(), backIntake->ProximityTriggered());
		
		//Drive.
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		//Shift.
		if (b_gearUp->ButtonClicked())
		{
			gearUp->Set(true);
			gearDown->Set(false);
		}
		else if (b_gearDown->ButtonClicked())
		{
			gearUp->Set(false);
			gearDown->Set(true);
		}

		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());
 
		if (b_hold->ButtonPressed())
		{
			//intakes to hold the ball using proxy sensors
			frontIntake->Hold();
			backIntake->Hold();
		}
		else if (b_reverseIntake->ButtonPressed())
		{
			//intake reverses
			frontIntake->Reverse();
			backIntake->Reverse();
		}
		//activates pickup for both intakes
		else if(b_IntakePickup->ButtonPressed())  
		{
			//Pickup is here b/c we are unsure of how it'll interact w/ the stops
			frontIntake->Pickup();
			backIntake->Pickup();
		}
		else
		{
			frontIntake->Stop();
			backIntake->Stop();
		}

		frontIntakeToggle = Toggle(b_frontIntakeDeployToggle, frontIntakeToggle);
		backIntakeToggle = Toggle(b_backIntakeDeployToggle, backIntakeToggle);
		
		//toggles the front intake up and down
		if (b_frontIntakeDeployToggle->ButtonClicked()) 
		{
			if (frontIntakeToggle)
			{
				frontIntakeDeploy->Set(true);
			}
			else if (!frontIntakeToggle)
			{
				frontIntakeDeploy->Set(false);
			}
		}

		//toggles the back intake up and down
		if (b_backIntakeDeployToggle->ButtonClicked()) 
		{
			if (backIntakeToggle)
			{
				backIntakeDeploy->Set(true);
			}
			else if (!backIntakeToggle)
			{
				backIntakeDeploy->Set(false);
			}
		}
		
		tarmPistonToggle = Toggle(b_tArmPistonToggle, tarmPistonToggle);
		//Arm piston toggle
		if(b_tArmPistonToggle->ButtonClicked())
		{
			if(tarmPistonToggle)
			{
				armPiston->Set(true);
			}
			else if(!tarmPistonToggle)
			{
				armPiston->Set(false);
			}
		}

		if(b_return->ButtonPressed())
		{
			shooter->ShooterReturn();
		}
		if(b_shoot->ButtonClicked())
		{
			shooter->BeginShooterFire(); //if called then Fire runs
		}
		shooter->ShooterFire();
		
		if(b_primeShortShot->ButtonClicked()) //primes for the short shot
		{
			shortShot = true; 
			shooter->ShooterPrime(shortShot);
		}
		
		if(b_primeLongShot->ButtonClicked()) //primes for the long shot
		{
			shortShot = false; //So short shot isn't primed for
			shooter->ShooterPrime(shortShot);
		}
		
		//HPReceive(b_humanLoad, armPiston, secondaryRollerA, secondaryRollerB); 
		//opens side arms for
		//hp intake
		
		
		if(driverR->GetRawButton(9))
		{
			secondaryRollerA->Set(1.0);
			secondaryRollerB->Set(-1.0);
		}
		else if(driverR->GetRawButton(10))
		{
			secondaryRollerA->Set(-1.0);
			secondaryRollerB->Set(1.0);
		}
		else
		{
			secondaryRollerA->Set(0.0);
			secondaryRollerB->Set(0.0);
		} 

		UpdateAllButtons();
	}
	void TestInit()
	{
		
	}
	void TestPeriodic()
	{
		printf("gyro: %f, gyroRate: %f\n", gyro->GetAngle(), gyro->GetRate());
	}
	
};
 
START_ROBOT_CLASS(Robot);
