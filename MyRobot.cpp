#include "WPILib.h"
#include "Drivetrain.h"
#include "IntakeSystem.h"
#include "HumanLoad.h"
#include "ShooterSystem.h"
#include "CitrusButton.h"

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
	IntakeSystem *frontIntake;
	IntakeSystem *backIntake;
	
	
	//Shooting
	
	Compressor *compressor;
	
	//Buttons!
	CitrusButton *b_gearUp;
	CitrusButton *b_gearDown;
	CitrusButton *b_frontIntakePickup;
	CitrusButton *b_backIntakePickup;
	CitrusButton *b_frontIntakeDeploy;
	CitrusButton *b_frontIntakeUndeploy;
	CitrusButton *b_backIntakeDeploy;
	CitrusButton *b_backIntakeUndeploy;
public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");
		//remove todos as the correct values are found
		drivetrain = new RobotDrive(3, 4);
		gearUp = new Solenoid(7); //TODO which is which?
		gearDown = new Solenoid (8);
		
		driverL = new Joystick(1);
		driverR = new Joystick(2);
		manipulator = new Joystick(3);
		
		//Solenoids.
		frontIntakeDeploy = new Solenoid (3);
		backIntakeDeploy = new Solenoid(4);
		frontIntake = new IntakeSystem (6, 3, frontIntakeDeploy, true);
		backIntake = new IntakeSystem (1, 2, backIntakeDeploy, false);
		
		compressor = new Compressor(1,1);
		
		leftEncoder = new Encoder(6,7);
		rightEncoder = new Encoder(4,5);
		 
		//Buttons. 
		//NOTE: ALWAYS ADD NEW BUTTON TO UpdateAllButtons()
		//Gearshift buttons
		b_gearUp = new CitrusButton(driverL, 1);
		b_gearDown = new CitrusButton(driverL, 2);
		//Intake buttons
		b_frontIntakePickup = new CitrusButton (manipulator, 8);
		b_backIntakePickup = new CitrusButton (manipulator, 6);
		b_frontIntakeDeploy = new CitrusButton (manipulator, 1);
		b_frontIntakeUndeploy = new CitrusButton (manipulator, 2);
		b_backIntakeDeploy = new CitrusButton (manipulator, 3);
		b_backIntakeUndeploy = new CitrusButton (manipulator, 4);
	}
	
	void UpdateAllButtons()
	{
		b_gearUp->Update();
		b_gearDown->Update();
		b_frontIntakePickup->Update();
		b_backIntakePickup->Update();
		b_frontIntakeDeploy->Update();
		b_frontIntakeUndeploy->Update();
		b_backIntakeDeploy->Update();
		b_backIntakeUndeploy->Update();
	}
	
	void DisabledInit(){
		//leftEncoder->Stop();
		//rightEncoder->Stop();
		
		frontIntakeDeploy->Set(false);
		backIntakeDeploy->Set(false);
	}
	void DisabledPeriodic(){
		
	}
	void AutonomousInit(){
		//printf("AUTO INIT\n");
		
	} 
	void AutonomousPeriodic(){
		
	}
	void TeleopInit(){
		compressor->Start();
		
		leftEncoder->Reset();
		rightEncoder->Reset();

		leftEncoder->Start();
		rightEncoder->Start();
		
		//printf("TELEOP INIT\n");
		
	}
	void TeleopPeriodic(){
		//drivetrain->TankDrive(0.0, 0.0);
		//talon1->Set(0.0);
		//intakeRollerBack->Set(0.0);
		//camTalon->Set(0.0);
		//talon6->Set(0.0);
		//Drive.
		printf("Left Encoder: %d Right Encoder: %d\n", leftEncoder->Get(), rightEncoder->Get());
		runDrivetrain(driverL->GetY(),driverR->GetTwist(), drivetrain);
		if(b_gearUp->ButtonClicked())
		{
			gearUp->Set(true);
			gearDown->Set(false);
		}
		else if (b_gearDown->ButtonClicked())
		{
			gearUp->Set(false);
			gearDown->Set(true);
		}
		//Shift.
		
		//printf("2 Proximity sensor: %d\n", frontIntake->ProximityTriggered());
		
		if(b_frontIntakePickup->ButtonPressed())
		{ 
			frontIntake->Hold();
		}
		else if(manipulator->GetRawButton(7))
		{
			frontIntake->Reverse();
		}
		else
		{
			frontIntake->Stop();
		}
		
		if(b_backIntakePickup->ButtonPressed())
		{
			backIntake->Hold();
		}
		else if(manipulator->GetRawButton(5))
		{
			backIntake->Reverse();
		}
		else
		{
			backIntake->Stop();
		}
		
		//intakes up/down
		if(b_frontIntakeDeploy->ButtonClicked())
		{
			frontIntakeDeploy->Set(true);
		}
		else if(b_frontIntakeUndeploy->ButtonClicked())
		{
			frontIntakeDeploy->Set(false);
		}
		if(b_backIntakeDeploy->ButtonClicked())
		{
			backIntakeDeploy->Set(true);
		}
		else if(b_backIntakeUndeploy->ButtonClicked())
		{
			backIntakeDeploy->Set(false);
		}
		
		UpdateAllButtons();
	}
	
	
};

START_ROBOT_CLASS(Robot);

