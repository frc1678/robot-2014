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
	float shooterAK;
	float shooterBK;
	Timer *deadzoneTimer;
	//Timer *camDelayTimer;
	double shooterWait;
	
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
		deadzoneTimer = new Timer();
		shooterAK = 1.0;
		shooterBK = 1.0;
		shooterWait = 0.15; 
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
		if (deadzoneTimer->Get() < shooterWait + 0.4 && !HallSensorTriggered())//(camDelayTimer->Get() < 0.5)
		{
			StopTalons();
		}
		else if(!HallSensorTriggered())
		{
			RunTalons();
		}
		else
		{
			StopTalons();
 		}
	}

	void ShooterPrime(bool shortShot) 
	{
		//all the relevant solenoids
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
		camTalonA->Set(.75 * shooterAK);
		camTalonB->Set(.75 * shooterBK); 
	}
	void ReverseTalons()
	{
		camTalonA->Set(-1.0 * shooterAK);
		camTalonB->Set(-1.0 * shooterBK);
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
		//camDelayTimer->Reset();
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
				if(deadzoneTimer->Get() < shooterWait)// ||deadzoneTimer->Get() > 0.75)
				{
					RunTalons();
				}
				else
				{
					DeadzoneDelayRun();
					//RunTalons();
				}
				//If we've gotten all the way around
				if (listenForSensor && HallSensorTriggered() && deadzoneTimer->Get() > shooterWait) //when does listenForSensor become true
				{
					StopTalons();
					listenForSensor = false;
					camPrimedToShoot = true;
					currentlyShooting = false;
					//camDelayTimer->Stop();
					//camDelayTimer->Reset();
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

