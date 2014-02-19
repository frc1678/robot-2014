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

	//booleans and such for void Pickup()
	bool readyToPickup;
	bool sensorTriggered;
	bool front;
	Timer *pickupTimer;

	IntakeSystem(int talonPort, int sensorPort, Solenoid *tIntakeUp, bool front)
	{
		intakeRoller = new Talon(talonPort);
		intakeSensor = new DigitalInput(sensorPort);
		intakeUp = tIntakeUp;

		front = true;
		readyToPickup = true;
		sensorTriggered = false;
		pickupTimer = new Timer();
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
	}

	//Internal: front roller grabbing onto ball, front roller holding ball, back roller grabbing onto ball, back roller holding ball, front roller reverse, back roller reverse
	void FrontRollerLoad()
	{
		intakeRoller->Set(1.0); //TODO numbers
	}

	void BackRollerLoad()
	{
		intakeRoller->Set(1.0); //TODO numbers
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

	void Hold()
	{
		if (!ProximityTriggered())
		{
			if (front)
			{
				FrontRollerLoad();
			}
			else
			{
				BackRollerLoad();
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
	void Pickup() //Only call these inside an if statement. This one currently assumes
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
				if (front)
				{
					FrontRollerLoad();
				}
				else
				{
					BackRollerLoad();
				}
			}
			//The below may be changed if another proximity sensor is added.
			else if (sensorTriggered)
			{
				pickupTimer->Start(); //TODO eventually don't keep starting it.
				if (front)
				{
					FrontRollerLoad();
				}
				else
				{
					BackRollerLoad();
				}
			}
			//To end the whole round.
			if (pickupTimer->Get() > 1.0) //TODO change to correct amount of time
			{
				readyToPickup = true;
				sensorTriggered = false;
				pickupTimer->Stop();
			}
		}
		//To start the whole round.
		if (readyToPickup)
		{
			pickupTimer->Reset();
			readyToPickup = false;
		}
	}
};
