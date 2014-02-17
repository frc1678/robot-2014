#include "WPILib.h"
#include "IntakeSystem.h"
#include "Drivetrain.h"

class Robot : public IterativeRobot
{
	RobotDrive *drivetrain; // robot drive system
	Joystick *driverL;
	Joystick *driverR;
	Solenoid *gearUp;
	Solenoid *gearDown;
	
	/*Talon *intakeRollerFront;
	Talon *intakeRollerBack;
	DigitalInput *intakeSensorFront;
	DigitalInput *intakeSensorBack;
	Solenoid *intakeFrontUp;
	Solenoid *intakeFrontDown;
	Solenoid *intakeBackUp;
	Solenoid *intakeBackDown;
	*/
	IntakeSystem *frontIntake;
	IntakeSystem *backIntake;
	
	//Skid.
	Solenoid *skidUp;
	Solenoid *skidDown;
	
	//Shooting
	Talon *camTalon;
	DigitalInput *camSensor;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;
	
	Talon *talon6;
	
	Compressor *compressor;
	
	
	//Gyro. http://playground.arduino.cc/Main/MPU-6050
	I2C *testGyro;
	DigitalModule *dModule;
	//I2C *testGyro2;
	
	
	//Gyro output array
	uint16_t gyroOut;
	//uint8_t gyroOut2;
public:
	Robot()
	{
		printf("Robot INITIALIZATION\n");
		//remove todos as the correct values are found
		drivetrain = new RobotDrive(3, 4); //TODO
		gearUp = new Solenoid(9); //TODO how many solenoids?
		gearDown = new Solenoid (10); //TODO how many solenoids?
		
		driverL = new Joystick(1); //TODO
		driverR = new Joystick(2);
		
		/*intakeRollerFront = new Talon(1); //TODO
		intakeRollerBack = new Talon(2); //TODO
		intakeSensorFront = new DigitalInput(1); //TODO
		intakeSensorBack = new DigitalInput(2); //TODO
		intakeFrontUp = new Solenoid (1);
		intakeFrontDown = new Solenoid (2);
		intakeBackUp = new Solenoid (3);
		intakeBackDown = new Solenoid(4);
		*/
		frontIntake = new IntakeSystem (1, 1, 1, 2); //TODO
		
		//Skid.
		skidUp = new Solenoid(5);
		skidDown = new Solenoid(6);
		
		camTalon = new Talon(5);
		camSensor = new DigitalInput(3); //TODO
		shotAlignerUp = new Solenoid(7);
		shotAlignerDown = new Solenoid(8);
		
		talon6 = new Talon(6);
		
		compressor = new Compressor(1,1);
		
		dModule = DigitalModule::GetInstance(1);
		testGyro = dModule->GetI2C(0x68<<1);
		testGyro->Write(0x6B, 0); //PWR_MGMT_1:=0
		
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
	
	void DisabledInit(){
		
	}
	void DisabledPeriodic(){
		
	}
	void AutonomousInit(){
		
		
		
	}
	void AutonomousPeriodic(){
		
	}
	void TeleopInit(){
		compressor->Start();

		
		printf("TELEOP INIT\n");
		
	}
	void TeleopPeriodic(){
		drivetrain->TankDrive(0.0, 0.0);
		//talon1->Set(0.0);
		//intakeRollerBack->Set(0.0);
		camTalon->Set(0.0);
		talon6->Set(0.0);
		
		//Drive.
		runDrivetrain(driverL->GetY(),driverR->GetY(), drivetrain);
		//Shift.
		
		
		/*intakeRollerFront->Set(stick->GetY());
		if(stick->GetRawButton(1))
		{
			intakeFrontUp->Set(true);
			intakeFrontDown->Set(false);
		}
		else if(stick->GetRawButton(2))
		{
			intakeFrontUp->Set(false);
			intakeFrontDown->Set(true);
		}*/
		
		uint8_t gyroValue = 7;
		
		//Pseudocode for robot
		//Type out in outline form in comments to insert code later
		//NB:Minimize air pressure for everything
				
		//Intake in the front and back of robot. Hold in ball until input signal 
		//then stop or slow depending on instance.
		//Based on button
		
		
		
		//Pickup (possibly seperate sequence for each side of robot) 
		//Roll in until input signal. Using pneumatic trigger
		
		//Shut off roller, but pneumatic stays up. Save air. Not a lot of time either
		
		
		//Run-into-wall skid using pneumatics, toggle/ish sequence
		
		
		//Shooting
		//Auto prime to fire. Fire moving CAM in one direction only
		
		//Sensor possibly to register degree of turn before a full revolution of the CAM
		
		//Fender shot with back intake down (maybe)
		
		//Range pneumatics for determining length of shot, possibly fingers up
		
		//Long shot with front intake down and range pneumatics in non-fender-shot-position
		
		
		//Infrared and ball sensors could be used. Magnetic sensor of CAM 
		
		
		
	}
	
	void intakeHold(bool buttonInput, Talon *roller, DigitalInput *sensor, Solenoid *up, Solenoid *down)
	{
		if(buttonInput) //TODO change to toggle or click-to-run
		{
			if(!sensor->Get()) //TODO deal with sensor inputs: 0 or 1?
			{
				roller->Set(1.0); //TODO direction?
			}
			else
			{
				roller->Set(0.5); //TODO split this up. May be different b/t front/back
			}
		}
	}
	void intakeReverse(bool buttonInput, Talon *roller)
	{
		if(buttonInput)
		{
			roller->Set(-1.0); //TODO direction?
		} //TODO stop y/n?
	}
	void intakePickup(bool buttonInput, Talon *roller)
	{
		if(buttonInput) //TODO's are the same as intakeHold
		{
			//TODO timing: how to determine if sensor has already been triggered
		}
	}
};

START_ROBOT_CLASS(Robot);

