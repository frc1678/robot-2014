#include "WPILib.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "MPU6050_I2C.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"
#include "Autonomous.h" 

class Robot : public IterativeRobot
{
	RobotDrive *drivetrain; // robot drive system
	Solenoid *gearUp;
	Solenoid *gearDown;
	Encoder *leftEncoder;
	Encoder *rightEncoder;

	Joystick *driverL;
	Joystick *driverR;
	Joystick *manipulator;

	//Intakes.
	Solenoid *frontIntakeDeploy;
	Solenoid *backIntakeDeploy;
	Talon *secondaryIntake;
	IntakeSystem *frontIntake;
	IntakeSystem *backIntake;
	IntakeSystem *secondaryFrontIntake;
	IntakeSystem *secondaryBackIntake;

	//Gyro
	MPU6050_I2C *gyro;
	
	//Intake Toggle
	bool frontIntakeToggle;
	bool backIntakeToggle;
	
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
	CitrusButton *b_secondaryFrontIntake;
	CitrusButton *b_secondaryBackIntake;
	CitrusButton *b_shot;
	CitrusButton *b_primeShortShot;
	CitrusButton *b_primeLongShot;
	CitrusButton *b_reverseIntake;
	CitrusButton *b_humanLoad;
	CitrusButton *b_hold;

public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");
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
		frontIntakeDeploy = new Solenoid (3);
		backIntakeDeploy = new Solenoid(4);

		//Intake
		secondaryIntake = new Talon (2); //TODO value?>
		frontIntake = new IntakeSystem (6, secondaryIntake, 3, frontIntakeDeploy, true); //TODO numbers
		backIntake = new IntakeSystem (1, secondaryIntake, 2, backIntakeDeploy, false); //TODO numbers

		compressor = new Compressor(1,1);

		leftEncoder = new Encoder(6,7);
		rightEncoder = new Encoder(4,5);

		//Shooter
		shooter = new ShooterSystem(2, 8, shotAlignerUp, shotAlignerDown, armPiston, frontIntakeDeploy,backIntakeDeploy);
		//TODO numbers
		shortShot = false;
		armPiston = new Solenoid (1); //TODO number?

		frontIntakeToggle = false;
		backIntakeToggle = false;
		
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
		b_shot = new CitrusButton (driverR, 1);
		b_primeShortShot = new CitrusButton (manipulator, 5);
		b_primeLongShot = new CitrusButton (manipulator, 7);

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
		b_shot->Update();
		b_primeShortShot->Update();
		b_primeLongShot->Update();
		
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
		//printf("AUTO INIT\n");
		GyroTurn(gyro, 90.0, drivetrain);
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
		for (int i = 0; i<60; i++)
		{
			if (manipulator->GetRawButton(i))
			{
				printf("%d", i);
			}
		}
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
		
		if(b_shot->ButtonClicked())
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
		
		HPReceive(b_humanLoad, armPiston, secondaryIntake); //opens side arms for
		//hp intake

		UpdateAllButtons();
	}

};
 
START_ROBOT_CLASS(Robot)
;
