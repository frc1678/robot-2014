#include "WPILib.h"

class ShooterSystem
{
public:

	Talon *camTalon;
	DigitalInput *camSensor;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;
	Solenoid *armPiston;
	Solenoid *frontIntakeDeploy;
	Solenoid *backIntakeDeploy;
	bool listenForSensor;
	bool camPrimedToShoot;
	bool currentlyShooting;
	
	ShooterSystem(int camTalonPort, int camSensorPort, Solenoid *tshotAlignerUp, 
			Solenoid *tshotAlignerDown, Solenoid *tarmPiston,
			Solenoid *tfrontIntakeDeploy, Solenoid *tbackIntakeDeploy)
	{
		camTalon = new Talon(camTalonPort);
		camSensor = new DigitalInput(camSensorPort);
		shotAlignerUp = tshotAlignerUp;
		shotAlignerDown = tshotAlignerDown;
		armPiston = tarmPiston;
		frontIntakeDeploy = tfrontIntakeDeploy;
		backIntakeDeploy = tbackIntakeDeploy;
		listenForSensor = false;
		camPrimedToShoot = false;
	}

	bool HallSensorTriggered()
	{
		if(camSensor->Get()==1)
		{
			return false;
		}
		return true;
	}
	
	void ShooterPrime(bool shortShot) //on manipulator
	{
		//all the relevant solenoids
		//Open up "arms"
		armPiston->Set(true); //TODO what direction is which? am I double solenoid?
		//Long/short toggle
		if(shortShot)
		{
			shotAlignerUp->Set(true); //TODO which is short and which is long?
			shotAlignerDown->Set(false);
		}
		else
		{
			shotAlignerUp->Set(false); //TODO which is short and which is long?
			shotAlignerDown->Set(true);
			
			//and front intake down
			frontIntakeDeploy->Set(true);
		}
		//Back intake down
		backIntakeDeploy->Set(true);
	}
	
	//call beginShooterFire on an if statement of buttonclicked
	//then call shooterfire outside the if statement.
	void BeginShooterFire() //on driver
	{
		camTalon->Set(1.0); //TODO direction?
		camPrimedToShoot = false;
		currentlyShooting = true;
	}
	void ShooterFire()
	{
		if(currentlyShooting)
		{
			if(!camPrimedToShoot)
			{
				if(listenForSensor && HallSensorTriggered())
				{
					camTalon->Set(0.0);
					listenForSensor = false;
					camPrimedToShoot = true;
					currentlyShooting = false;
				}
				if(HallSensorTriggered()) 
				{
					listenForSensor = false;
				}
				else
				{
					listenForSensor = true;
				}
				camTalon->Set(1.0);
			}
			else
			{
				camTalon->Set(0.0);				
			}
		}
		else
		{
			camTalon->Set(0.0);
		}
	}
};
