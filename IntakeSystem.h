#include "WPILib.h"
#ifndef INTAKESYSTEM_H
#define INTAKESYSTEM_H //include protection

class IntakeSystem
{
public:
	/*
	 bool ProximityTriggered();
	 void BackRollerInitialPickup();
	 void FrontRollerInitialPickup();
	 void FrontRollerHold();
	 void BackRollerHold();
	 */
	Talon *intakeRoller;
	DigitalInput *intakeSensor;
	Solenoid *intakeUp;
	Talon *secondaryIntakeRollerA;
	Talon *secondaryIntakeRollerB;

	//booleans and such for void Pickup()
	bool readyToPickup;
	bool sensorTriggered;
	bool front;
	Timer *pickupTimer;
	bool intakePulled;

	IntakeSystem(int rollerTalonPort, Talon *tsecondaryIntakeRollerA, 
			Talon *tsecondaryIntakeRollerB, int sensorPort, Solenoid *tIntakeUp, bool front)
	{
		intakeRoller = new Talon(rollerTalonPort);
		secondaryIntakeRollerA = tsecondaryIntakeRollerA;
		secondaryIntakeRollerB = tsecondaryIntakeRollerB;
		intakeSensor = new DigitalInput(sensorPort);
		intakeUp = tIntakeUp;

		front = true;
		readyToPickup = true;
		sensorTriggered = false;
		pickupTimer = new Timer();
		intakePulled = false;
	}
	//TODO add secondary rollers to methods. Have some internal ones
	//then add to Pickup.
	void Reverse() //Only call inside an if statement.
	{
		intakeRoller->Set(-1.0);
	}
	void Stop()
	{
		intakeRoller->Set(0.0);
		sensorTriggered = false;
		readyToPickup = true;
		pickupTimer->Stop();
		pickupTimer->Reset();
		intakePulled = false;
	}
	
	void RunSecondaryRollers()
	{
		secondaryIntakeRollerA->Set(1.0);
		secondaryIntakeRollerB->Set(-1.0);
	}
	void ReverseSecondaryRollers()
	{
		secondaryIntakeRollerA->Set(-1.0);
		secondaryIntakeRollerB->Set(1.0);
	}
	void StopSecondaryRollers()
	{
		secondaryIntakeRollerA->Set(0.0);
		secondaryIntakeRollerB->Set(0.0);
	}
	
	void DeployIntake ()
	{
		intakeUp->Set(true);
	}
	void UndeployIntake()
	{
		intakePulled = true;
		intakeUp->Set(false);
	}

	//Internal: front roller grabbing onto ball, front roller holding ball, back roller grabbing onto ball, back roller holding ball, front roller reverse, back roller reverse
	void FrontRollerLoad(Joystick * stick)
	{
		intakeRoller->Set(1.0); //TODO numbers
		//intakeRoller->Set(stick->GetTwist());
	}

	void BackRollerLoad(Joystick * stick)
	{
		intakeRoller->Set(1.0); //TODO numbers
		//intakeRoller->Set(stick->GetY());
	}

	void BackBumperHold()
	{
		intakeRoller->Set(0.0); //TODO numbers
	}
	
	void FrontBumperHold()
	{
		intakeRoller->Set(0.0); //TODO numbers.
	}
	
	
	bool ProximityTriggered()
	{
		if (intakeSensor->Get() == 1)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	void Hold(Joystick *stick)
	{
		if (!ProximityTriggered())
		{
			if (front)
			{
				FrontRollerLoad(stick);
			}
			else
			{
				BackRollerLoad(stick);
			}
		}
		else
		{
			printf("Proxy!\n");
			if(front)
			{
				FrontBumperHold();
			}
			else
			{
				BackBumperHold();
			}
		}
	}
	void FrontRollerSlow(DriverStation *m_ds)
	{
		intakeRoller->Set(m_ds->GetAnalogIn(3));
	}
	void BackRollerSlow()
	{
		intakeRoller->Set(0.5);
	}
	
	void Pickup(Joystick * stick, DriverStation *m_ds)
	{
		if(ProximityTriggered())
		{
			printf("sensor");
			sensorTriggered = true;
		}
		if(!sensorTriggered)
		{
			printf("running");
			BackRollerLoad(stick);
		}
		else
		{
			pickupTimer->Start();
			if(pickupTimer->Get() < m_ds->GetAnalogIn(1))
			{
				BackRollerLoad(stick);
			}
			else
			{
				if(!intakePulled)
				{
					printf("undeploy");
					UndeployIntake();
				}
				printf("slow");
				BackRollerSlow();
			}
		}
	}
	//TODO make them in the same function.
	void FrontPickup(Joystick * stick, DriverStation *m_ds)
	{
		if(ProximityTriggered())
		{
			printf("sensor");
			sensorTriggered = true;
		}
		if(!sensorTriggered)
		{
			printf("running");
			FrontRollerLoad(stick);
		}
		else
		{
			pickupTimer->Start();
			if(pickupTimer->Get() < m_ds->GetAnalogIn(2))
			{
				FrontRollerLoad(stick);
			}
			else
			{
				if(!intakePulled)
				{
					printf("undeploy");
					UndeployIntake();
				}
				printf("slow");
				FrontRollerSlow(m_ds);
			}
		}
	}
	
	/*void Pickup(Joystick *stick, DriverStation * m_ds) //Only call these inside an if statement. This one currently assumes
	//that if it's called, the first run is where to start.
	{
		if (!readyToPickup) //aka, I'm currently picking something up
		{
			if (ProximityTriggered())
			{
				sensorTriggered = true;
			}
			if (!sensorTriggered)
			{
				if (front) //if front is false than we are using the back
				{
					FrontRollerLoad(stick);
				}
				else
				{
					BackRollerLoad(stick);
				}
			}
			//The below may be changed if another proximity sensor is added.
			else if (sensorTriggered)
			{
				pickupTimer->Start(); //TODO eventually don't keep starting it.
				if(pickupTimer->Get() < m_ds->GetAnalogIn(1))
				{
					if (front)
					{
						FrontRollerLoad(stick);
					}
					else
					{
						BackRollerLoad(stick);
					}
				}
				else
				{
					if(!intakePulled)
					{
						UndeployIntake();
					}
					if (front)
					{
						FrontRollerSlow();
					}
					else
					{
						BackRollerSlow();
					}
				}
				//RunSecondaryRollers();
			}
			//To end the whole round.
			if (pickupTimer->Get() > 2.0) //TODO change to correct amount of time
			{
				readyToPickup = true;
				sensorTriggered = false;
				//StopSecondaryRollers(); //TODO numbers
				pickupTimer->Stop();
			}
		}
		//To start the whole round.
		if (readyToPickup)
		{
			pickupTimer->Reset();
			readyToPickup = false;
		}
	}*/
	
};

#endif
