#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class Robot : public IterativeRobot
{
	RobotDrive *drivetrain; // robot drive system
	Joystick *stick; // only joystick
	
	//Talons.
	Talon *talon1;
	Talon *talon2;
	Talon *talon5;
	Talon *talon6;
	
	Compressor *compressor;
	
	//Gyro. http://playground.arduino.cc/Main/MPU-6050
	
public:
	Robot()
	{
		drivetrain = new RobotDrive(3, 4);
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon5 = new Talon(5);
		talon6 = new Talon(6);
		
		compressor = new Compressor(1,1);
		
		
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
		drivetrain->TankDrive(0.0, 0.0);
		talon1->Set(0.0);
		talon2->Set(0.0);
		talon5->Set(0.0);
		talon6->Set(0.0);
	}
	
};

START_ROBOT_CLASS(Robot);

