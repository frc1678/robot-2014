#include "WPILib.h"
#include "Buttons.h"
#include "Drivetrain.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class Robot : public IterativeRobot
{
	RobotDrive *drivetrain; // robot drive system
	
	Joystick *driverL; //left driver joystick
	Joystick *driverR;
	
	//Talons.
	Talon *talon1;
	Talon *talon2;
	Talon *talon5;
	Talon *talon6;
	
	Compressor *compressor;
	
	//Solenoids.
	Solenoid *shiftUp;
	Solenoid *shiftDown;
	
	//Gyro. http://playground.arduino.cc/Main/MPU-6050
	
public:
	Robot()
	{
		drivetrain = new RobotDrive(3, 4);
		
		driverL = new Joystick(LEFT_JOYSTICK);
		driverR = new Joystick(RIGHT_JOYSTICK);
		
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon5 = new Talon(5);
		talon6 = new Talon(6);
		
		compressor = new Compressor(1,1);
		shiftUp = new Solenoid(1); //TODO see which solenoids are actually up/down?
		shiftDown = new Solenoid(2); //see previous TODO
		
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
	}
	void TeleopPeriodic(){
		//Run drivetrain; drivetrain.h
		runDrivetrain(driverL->GetY(), driverR->GetY(), drivetrain);
		shiftGears(shiftUp, shiftDown, driverL, 1, driverL, 2); 
		
		talon1->Set(0.0);
		talon2->Set(0.0);
		talon5->Set(0.0);
		talon6->Set(0.0);
	}
	
};

START_ROBOT_CLASS(Robot);

