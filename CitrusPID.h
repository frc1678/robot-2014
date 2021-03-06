#include "WPIlib.h"
#include "NetworkTables/NetworkTable.h"

#ifndef CITRUSPID_H
#define CITRUSPID_H

class CitrusPID 
{
	/*
	do not attempt to read and understand if you do not have a basic understanding of what PID loops are.
	This file was used for experimenting with PID and contains the network table stuff as well as attempted
	"drive straight" code with encoders
	ALSO: if you have experience with PID and have suggesgions for how to do it better, we would love to hear them at:
	1678programming@gmail.com
	*/
	
	float perror;
	float ierror;
	float derror;
	
	float target;
	float lOutput;
	float targetDistance;
	Timer *timer;
	
	NetworkTable *dataTable;
	
	
public:
	CitrusPID()
	{
		perror = 0.0; //proportional error 
		ierror = 0.0; 
		derror = 0.0; 
		
		target = 0.0;
		lOutput = 0.0;
		targetDistance = 0.0;
		
		timer = new Timer();
		timer->Start();
		
		//Network Tables
		dataTable = NetworkTable::GetTable("PIDTable");
		dataTable->PutNumber("targetppsL", 1500);
		dataTable->PutNumber("targetDistance", 100);
		dataTable->PutNumber("kp", 0.00005); //constant
		dataTable->PutNumber("ki", 0.000016); //constant
		dataTable->PutNumber("kd", 0.0); //constant
	}
	
	void Update(Encoder *encoder)
	{ //WOOOOOOOOO fun PID!
		target = dataTable->GetNumber("targetppsL");
		targetDistance = dataTable->GetNumber("targetDistance");
		
		derror = ((target - (-encoder->GetRate()))-perror)/timer->Get();
		perror = target - (-encoder->GetRate());
		ierror = perror * timer->Get();
					
		lOutput = perror * dataTable->GetNumber("kp")
			+ ierror * dataTable->GetNumber("ki")
			+ derror * 0.0001 * dataTable->GetNumber("kd")
			+ 1.0 * target/2000.0; //2000 ~ 1.0 power free speed
		if((-1*encoder->Get()) >= targetDistance) {
			dataTable->PutNumber("targetppsL", 0.0);
			printf("Out!\n");
		}
		
		timer->Reset();
	}
	
	float GetOutput()
	{
		return lOutput;
	}
	
	float GetTargetDistance()
	{
		return targetDistance;
	}
	
};
#endif //CITRUSPID_H
