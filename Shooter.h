#include "WPILib.h"

class ShooterSystem
{
public:

	Talon *camTalon;
	DigitalInput *camSensor;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;

	ShooterSystem(int camTalonPort, int camSensorPort, int shotAlignerUpPort, int shotAlignerDownPort)
	{
		camTalon = new Talon(camTalonPort);
		camSensor = new DigitalInput(camSensorPort);
		shotAlignerUp = new Solenoid(shotAlignerUpPort);
		shotAlignerDown = new Solenoid(shotAlignerDownPort);
	}
	void ShooterPrime()
	{
		while(camSensor->Get() == 0)
		{
			camTalon->Set(1.0);
		}
		camTalon->Set(0.0);
	}

	void ShooterFire()
	{
		camTalon->Set(1.0);
		Wait(0.2); //TODO numbers
		camTalon->Set(0.0);
	}

	void FenderShot()
	{
		shotAlignerUp->Set(true);

	}
};
