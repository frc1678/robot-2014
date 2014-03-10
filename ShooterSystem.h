#include "WPILib.h"

#ifndef SHOOTERSYSTEM_H
#define SHOOTERSYSTEM_H

class ShooterSystem
{
public:

	Talon *camTalonA;
	Talon *camTalonB;
	DigitalInput *camSensor;
	Solenoid *shotAlignerUp;
	Solenoid *shotAlignerDown;
	Solenoid *armPiston;
	bool listenForSensor;
	bool camPrimedToShoot;
	bool currentlyShooting;
	int numTimesSensorTriggered; //within one shooting cycle
	Timer *camDelayTimer;
	Timer *deadzoneTimer;
	
	//ANY TIME YOU WANT TO DEPLOY INTAKES, MAKE A PARALLEL METHOD IN
	//INTAKESYSTEM

	ShooterSystem(int camTalonPortA, int camTalonPortB, int camSensorPort,
			Solenoid *tshotAlignerUp, Solenoid *tshotAlignerDown,
			Solenoid *tarmPiston)
	{
		camTalonA = new Talon(camTalonPortA);
		camTalonB = new Talon(camTalonPortB);
		camSensor = new DigitalInput(camSensorPort);
		shotAlignerUp = tshotAlignerUp;
		shotAlignerDown = tshotAlignerDown;
		armPiston = tarmPiston;
		listenForSensor = false;
		camPrimedToShoot = false;
		currentlyShooting = false;
		numTimesSensorTriggered = 0;
		camDelayTimer = new Timer();
		deadzoneTimer = new Timer();
	}

	bool HallSensorTriggered()
	{
		if (camSensor->Get()==1) //inverted
		{
			return false;
		}
		return true;
	}

	void DeadzoneDelayRun()
	{
		if (deadzoneTimer->Get() < 0.7 && !HallSensorTriggered())//(camDelayTimer->Get() < 0.5)
		{
			StopTalons();
		}
		else
		{
			RunTalons();
		}
	}

	void ShooterPrime(bool shortShot) //on manipulator
	{
		//all the relevant solenoids
		//Open up "arms"
		//armPiston->Set(true); //TODO what direction is which? am I double solenoid?
		//Long/short toggle
		if (shortShot)
		{
			shotAlignerUp->Set(false);
			shotAlignerDown->Set(true);
		}
		else
		{
			shotAlignerUp->Set(true); 
			shotAlignerDown->Set(false);

			//and front intake down
			//frontIntakeDeploy->Set(true); //make room for shot
		}
		//Back intake down
		//backIntakeDeploy->Set(true);
	}

	void RunTalons()
	{
		camTalonA->Set(.75); //facing opposite directions
		camTalonB->Set(-.75);
	}
	void ReverseTalons()
	{
		camTalonA->Set(-1.0);
		camTalonB->Set(1.0);
	}
	void StopTalons()
	{
		camTalonA->Set(0.0);
		camTalonB->Set(0.0);
	}

	void Reset()
	{
		listenForSensor = false;
		camPrimedToShoot = false;
		currentlyShooting = false;
		numTimesSensorTriggered = 0;
		camDelayTimer->Reset();
	}

	//call beginShooterFire on an if statement of buttonclicked
	//then call shooterfire outside the if statement.
	void BeginShooterFire() //on driver
	{
		RunTalons(); 
		camPrimedToShoot = false;
		currentlyShooting = true;
		listenForSensor = false;
		numTimesSensorTriggered = 0;
		deadzoneTimer->Reset();
		deadzoneTimer->Start();
	}
	void ShooterFire()
	{
		/*printf(
				"Hall sensor: %d, currentlyShooting: %d, Timer: %f, NumTimes: %d\n",
				HallSensorTriggered(), currentlyShooting, camDelayTimer->Get(),
				numTimesSensorTriggered);*/
		if (currentlyShooting)
		{
			if (!camPrimedToShoot)
			{
				if(deadzoneTimer->Get() < 0.35)// ||deadzoneTimer->Get() > 0.75)
				{
					RunTalons();
				}
				else
				{
					DeadzoneDelayRun();
					//RunTalons();
				}
				//If we've gotten all the way around
				if (listenForSensor && HallSensorTriggered() && deadzoneTimer->Get() > 0.35) //when does listenForSensor become true
				{
					StopTalons();
					listenForSensor = false;
					camPrimedToShoot = true;
					currentlyShooting = false;
					camDelayTimer->Stop();
					camDelayTimer->Reset();
				}
				if (HallSensorTriggered())
				{
					/*if (numTimesSensorTriggered == 0)
					{
						RunTalons();
					}
					else if (numTimesSensorTriggered == 1)
					{
						camDelayTimer->Start();
						DeadzoneDelayRun();
					}
					else
					{
						StopTalons();
					}*/
					listenForSensor = false;
				}
				else
				{
					listenForSensor = true;
					/*if (numTimesSensorTriggered == 1)
					{
						DeadzoneDelayRun();
					}
					else if(numTimesSensorTriggered == 2)
					{
						StopTalons();
					}*/
				}
			}
			else
			{
				StopTalons();
			}
		}
		else
		{
			StopTalons();
		}
	}
	//ONLY WHEN A BUTTON IS PRESSED.
	void ShooterReturn()
	{
		if(HallSensorTriggered())
		{
			StopTalons();
		}
		else
		{
			RunTalons();
		}
	}
	
	bool CurrentlyShooting()
	{
		return currentlyShooting;
	}
};

#endif

