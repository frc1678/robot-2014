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
	Solenoid *skid;

	//booleans and such for void Pickup()
	bool readyToPickup;
	bool sensorTriggered;
	bool front;
	Timer *pickupTimer;

	IntakeSystem(int talonPort, int sensorPort, int upPort, bool front)
	{
		//Remove the correct inputs are found
		intakeRoller = new Talon(talonPort); //TODO
		intakeSensor = new DigitalInput(sensorPort); //TODO
		intakeUp = new Solenoid (upPort);

		front = true;
		readyToPickup = true;
		sensorTriggered = false;
		pickupTimer = new Timer();
	}

	void Reverse() //Only call inside an if statement.
	{
		intakeRoller->Set(-1.0);
	}
	void Stop()
	{
		intakeRoller->Set(0.0);
	}

	//Internal: front roller grabbing onto ball, front roller holding ball, back roller grabbing onto ball, back roller holding ball, front roller reverse, back roller reverse
	void FrontRollerInitialPickup()
	{
		intakeRoller->Set(1.0); //TODO numbers
	}

	void FrontRollerHold()
	{
		intakeRoller->Set(0.0); //TODO numbers
	}

	void BackRollerHold()
	{
		intakeRoller->Set(0.0); //TODO numbers
	}

	void BackRollerInitialPickup()
	{
		intakeRoller->Set(1.0); //TODO numbers
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

	void Hold() //TODO front versus back, change later
	{
		if (!ProximityTriggered())
		{
			if (front)
			{
				FrontRollerInitialPickup();
			}
			else
			{
				BackRollerInitialPickup();
			}
		}
		else
		{
			if (front)
			{
				FrontRollerHold();
			}
			else
			{
				BackRollerHold();
			}
		}
	}
	void Pickup() //Only call these inside an if statement. This one currently assumes
	//that if it's called, the first run is where to start.
	{
		if (!readyToPickup) //aka, I'm currently picking something up
		{
			if (ProximityTriggered()) //TODO which direction is which?
			{
				sensorTriggered = true;
			}
			if (!sensorTriggered)
			{
				if (front)
				{
					FrontRollerInitialPickup();
				}
				else
				{
					BackRollerInitialPickup();
				}
			}
			else if (sensorTriggered)
			{
				pickupTimer->Start(); //TODO eventually don't keep starting it.
				if (front)
				{
					FrontRollerInitialPickup();
				}
				else
				{
					BackRollerInitialPickup();
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
