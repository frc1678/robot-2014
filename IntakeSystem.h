#include "WPILib.h"
#ifndef INTAKESYSTEM_H
#define INTAKESYSTEM_H //include protection
class IntakeSystem {
public:
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

	//boolean for is intake deployed?
	bool intakeDeployed;

	float frontIntakeK;
	float backIntakeK;

	IntakeSystem(int rollerTalonPort, Talon *tsecondaryIntakeRollerA,
			Talon *tsecondaryIntakeRollerB, int sensorPort,
			Solenoid *tIntakeUp, bool tfront) {
		intakeRoller = new Talon(rollerTalonPort);
		secondaryIntakeRollerA = tsecondaryIntakeRollerA;
		secondaryIntakeRollerB = tsecondaryIntakeRollerB;
		intakeSensor = new DigitalInput(sensorPort);
		intakeUp = tIntakeUp;

		front = tfront;
		readyToPickup = true;
		sensorTriggered = false;
		pickupTimer = new Timer();
		intakeDeployed = false;

		frontIntakeK = 1.0;
		backIntakeK = 1.0;
	}
	//then add to Pickup.
	void Reverse() //Only call inside an if statement.
	{
		//roller motors are set backward (rotate as if sucking in from on top)
		if (front) {
			intakeRoller->Set(-1.0 * frontIntakeK);
		} else {
			intakeRoller->Set(-1.0 * backIntakeK);
		}

	}
	void ReverseSlow()
	{
		if (front)
		{
			intakeRoller->Set(-0.5 * frontIntakeK);
		}
		else
		{
			intakeRoller->Set(-0.5 * backIntakeK);
		}
	}
	void Stop() {
		intakeRoller->Set(0.0);
		sensorTriggered = false;
		readyToPickup = true;
		pickupTimer->Stop();
		pickupTimer->Reset();
	}

	//USE THESE WHENEVER YOU WANT TO DEPLOY THE INTAKES.
	void DeployIntake() {
		intakeDeployed = true;
		intakeUp->Set(true);
	}
	void UndeployIntake() {
		intakeDeployed = false;
		intakeUp->Set(false);
	}
	void ToggleIntake() {
		intakeDeployed = !intakeDeployed;
		intakeUp->Set(intakeDeployed);
	}

	bool DeployState() { 
		return intakeDeployed;
	}

	//Internal: front roller grabbing onto ball, front roller holding ball, back roller grabbing onto ball, back roller holding ball, front roller reverse, back roller reverse
	void FrontRollerLoad() {
		if (front) {
			intakeRoller->Set(1.0 * frontIntakeK);
		} else {
			intakeRoller->Set(1.0 * backIntakeK);
		}
	}

	void BackRollerLoad() {
		if (front) {
			intakeRoller->Set(1.0 * frontIntakeK);
		} else {
			intakeRoller->Set(1.0 * backIntakeK);
		} 
	}

	void BackBumperHold() {
		intakeRoller->Set(0.0); 
	}

	void FrontBumperHold() {
		intakeRoller->Set(0.0); 
	}

	void FrontRollerAutoSlow() { //used during auto
		if (front) {
			intakeRoller->Set(0.25 * frontIntakeK);
		} else {
			intakeRoller->Set(0.5 * backIntakeK); 
		}
	}
	void BackRollerAutoSlow() {
		if (front) {
		intakeRoller->Set(0.25 * frontIntakeK);
		} else {
			intakeRoller->Set(0.5 * backIntakeK);
		}
	}
	
	void RunAt(float x)
	{
		if(front)
		{
			intakeRoller->Set(x * frontIntakeK);	
		}
		else
		{
			intakeRoller->Set(x * backIntakeK); 
		}
		
	}

	bool ProximityTriggered() { //TODO, add link to picture of where this sensor is
		if (intakeSensor->Get() == 1) {
			return false;
		} else {
			return true;
		}
	}

	void FrontRollerSlow(DriverStation *m_ds) {
		if (front) {
			intakeRoller->Set(0.5 * frontIntakeK);
		} else {
			intakeRoller->Set(0.5 * backIntakeK);
		}
	}
	void BackRollerSlow() {
		if (front) {
			intakeRoller->Set(0.5 * frontIntakeK);
		} else {
			intakeRoller->Set(0.5 * backIntakeK);
		}
	}

	void Hold() {
		//tell us when to stop rolling in order to hold the ball between the bumpers and the intakes.
		if (!ProximityTriggered()) {
			if (front) {
				FrontRollerLoad();
			} else {
				BackRollerLoad();
			}
		} else {
			if (front) {
				FrontBumperHold();
			} else {
				BackBumperHold();
			}
		}
	}
	void Pickup(Joystick * stick, DriverStation *m_ds) {
		if (ProximityTriggered()) {
			sensorTriggered = true;
		}
		if (!sensorTriggered) {
			BackRollerLoad();
		} else {
			pickupTimer->Start();
			if (pickupTimer->Get() < m_ds->GetAnalogIn(1)) {
				BackRollerLoad();
			} else {
				if (intakeDeployed) {
					UndeployIntake();
				}
				BackRollerSlow();
			}
		}
	}
	
	void FrontPickup( DriverStation *m_ds) {
		if (ProximityTriggered()) {
			sensorTriggered = true;
		}
		if (!sensorTriggered) {
			FrontRollerLoad();
		} 
		else {
			pickupTimer->Start();
			if (pickupTimer->Get() < m_ds->GetAnalogIn(2)) {
				FrontRollerLoad();
			} 
			else {
				if (intakeDeployed) {
					UndeployIntake();
				}
				FrontRollerSlow(m_ds);
			}
		}
	}

};

#endif
