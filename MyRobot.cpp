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
	I2C *testGyro;
	DigitalModule *dModule;
	//I2C *testGyro2;
	
	
	//Gyro output array
	uint16_t gyroOut;
	//uint8_t gyroOut2;
public:
	Robot()
	{
		drivetrain = new RobotDrive(3, 4);
		talon1 = new Talon(1);
		talon2 = new Talon(2);
		talon5 = new Talon(5);
		talon6 = new Talon(6);
		
		compressor = new Compressor(1,1);
		dModule = DigitalModule::GetInstance(1);
		testGyro = dModule->GetI2C(0xd0); //d0, 68
		testGyro->Write(0x6B, 0);

		printf("begin");
		/*for(uint8_t i = 0x00; i< 0xFF; i++)
		{
			testGyro = dModule->GetI2C(i);
			if(testGyro->AddressOnly() == 0)
			{
				printf("success! %x", i);
			}
		}*/
		
		//testGyro2 = dModule->GetI2C(2);
		gyroOut = 0;
		
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
	
		uint8_t gyroValue;
		
		printf("end");
		printf("HELLO WORLD");
		/*for(int addr=0;addr<=0xff;addr++) {
			testGyro = dModule->GetI2C(0xd0); //d0 or 0x68
		}*/
		
		testGyro->Write(0x6B, 0); //PWR_MGMT_1:=0
		printf("%d:\n", gyroValue);

		for(int i = 0x00; i < 0x80; i++)
		{
			testGyro->Read(i, sizeof(gyroValue),&gyroValue);
			printf("%02X ", gyroValue); 
			if (!(i%0x10))
				printf("\n");
			
		}
	}
	void TeleopPeriodic(){
		drivetrain->TankDrive(0.0, 0.0);
		talon1->Set(0.0);
		talon2->Set(0.0);
		talon5->Set(0.0);
		talon6->Set(0.0);
		
		//printf("%f, %f, %f\n",testGyro->GetAcceleration(ADXL345_I2C::kAxis_X),testGyro->GetAcceleration(ADXL345_I2C::kAxis_Y),testGyro->GetAcceleration(ADXL345_I2C::kAxis_Z));
		uint8_t gyroValue = 7;
		//testGyro->Read(0x44, sizeof(gyroValue), &gyroValue);
		//printf("%f\n",  (float)gyroValue);
				
		
		for(int i = 0x00; i < 0x75; i++)
		{
			uint8_t gyroValue = 7;
			//printf("%d %d\n", sizeof(gyroValue), &gyroValue);
			testGyro->Read(i, sizeof(gyroValue), &gyroValue);
			printf("%d ", gyroValue); 
			if (i % 16 == 15) {
				printf("\n");
			}
		}	
		//printf("%d\n", testGyro->AddressOnly());
		//testGyro2->Read(67, 1, &gyroOut2);
		//printf("Gyro2: %x\n", gyroOut2);	
	}
};

START_ROBOT_CLASS(Robot);

