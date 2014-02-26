#include "WPILib.h"

#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

class CurrentSensor
{
	AnalogChannel * a;
public:
	CurrentSensor(int port)
	{
		a = new AnalogChannel(port);
	}
	
	float Get()
	{
		float returnMe = a->GetVoltage();
		return returnMe;
	}
};

#endif //CURRENTSENSOR_H