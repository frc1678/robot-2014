class IntakeSystem
{
	Talon *intakeRoller;
	DigitalInput *intakeSensor;
	Solenoid *intakeUp;
	Solenoid *intakeDown;
public:
	IntakeSystem(int talonPort, int sensorPort, int upPort, int downPort)
	{
		//Remove the correct inputs are found
		intakeRoller = new Talon(talonPort); //TODO
		intakeSensor = new DigitalInput(sensorPort); //TODO
		intakeUp = new Solenoid (upPort);
		intakeDown = new Solenoid (downPort);
	}
	
	void Hold() //TODO front versus back, change later
	{
		
	}
	void Pickup()
	{
		
	}
	void Reverse()
	{
		
	}
};
