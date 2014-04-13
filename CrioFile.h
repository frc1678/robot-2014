#include "WPILib.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <ctime>

#ifndef CRIOFILE_H
#define CRIOFILE_H

using namespace std;
class CrioFile {
	//AnalogChannel *a;
	Timer *currentTimer;
	//istream *reader;
	ofstream fileSave; //create new output file 
	bool currentlyLogging;
	float voltageCounter;
	
public:
	CrioFile()
	{
		//a = new AnalogChannel(3);
		//fileSave.open("File.txt");
		currentTimer = new Timer();
		currentlyLogging = false;
		voltageCounter = 0;
	}
		
	/*float GetCurrent()
	{
		float returnMe = a->GetVoltage();
		return returnMe;
	}*/
	void StartLog()
	{
		/*string currentLogNum = "";
		fileSave.open("LogNum.txt");
		istream& reader>>(fileSave,currentLogNum);
		(int)currentLogNum;
		fileSave.close();
		string newFileName = (currentLogNum + ".txt");*/
		fileSave.open("File.txt"); //opening the file
		//reseting and starting the timer
		currentTimer->Reset(); 
		currentTimer->Start();
		
		currentlyLogging = true;
		fileSave<<"Time: "<<"Current:\n\r"; //writing to the file
		fileSave.close();
		fileSave.open("Heat.txt");
		currentlyLogging = true;
		fileSave<<"Time: "<<"Heat:\n\r";
		fileSave.close();
		
		fileSave.open("Encoder.txt");
		currentlyLogging = true;
		fileSave<<"Time: Rate: Clicks: \n\r";
		fileSave.close();
		//Two seperate files, one for heat and one for current.
	}
	void LogCurrent(AnalogChannel *a)
	{
		//static ifstream fileIn; //create file object 
		//fileIn.open("File.txt"); //read in file
		
		/*bool timing = currentTimer->m_running;
		
		if(timing == false)
		{
			currentTimer->Start();
			currentTimer->Reset();			
		}*/

		if(currentlyLogging) //only run if we are logging
		{
			fileSave.open("File.txt", ios::app); //temp

			float current = a->GetVoltage();
			current = fabs(5-current); //log
			//inverse chart, therefore absolute value
			//5 b/c 0 = 5 on the inverted scale		
			double time = currentTimer->Get();
			fileSave<<time<<" "<<current<<"\r\n";
			
			if(fileSave.fail())//test to see if file opened
			{
				printf("File did not open.\n");
			}
		}

		fileSave.close();
	}

	void LogHeat(AnalogChannel *b)
		{

			

			if(currentlyLogging) //only if we are currently logging
			{
				fileSave.open("Heat.txt", ios::app); //temp
				//app appends the writing, so it writes to the end of the file
				
				float Value = b->GetValue();
				double time = currentTimer->Get();
				fileSave<<time<<" "<<Value<<"\r\n";
				
				if(fileSave.fail())//test to see if file opened
				{
					printf("File did not open.\n");
				}
			}

			fileSave.close();
		}
	
	void LogEncoders(Encoder *leftEncoder, Encoder *rightEncoder)
	{
		
				
		if(currentlyLogging)
		{
			fileSave.open("Encoders.txt", ios::app);
			
			double time = currentTimer->Get();
			float rightRate = rightEncoder->GetRate();
			float leftRate = leftEncoder->GetRate();
			float rightValue = rightEncoder->Get();
			float leftValue = leftEncoder->Get();
			
			fileSave<<time<<" "<<leftRate<< " "<<rightRate<<"  "<<leftValue<< " "
					<<rightValue<<"\r\n";
			
			if(fileSave.fail())
			{
				printf("File did not open.\n");
			}
		}
		
		fileSave.close();
	}
		

	void EndLog()
	//flushing and closing the file
	{
		if(currentlyLogging)
		{
			currentlyLogging = false;
			fileSave.close();
			currentTimer->Stop();
		}
	}
	
	float CheckVoltage(AnalogChannel *a) //checking the voltage of the sensors
	{
		float current = a->GetVoltage();
		current = fabs(5.0-current); //check
		//inverse chart, therefore absolute value
		//5 b/c 0 = 5 on the inverted scale
		
		return current;
	}
	
	float CheckHeat(AnalogChannel *b) //checking the temperatue of the sensors
	{
		float Value = b->GetValue(); //This get function may end up being a different one.
		
		return Value;
	}
	
	//TODO USE DONALD'S NEW FORMULA AND INCORPERATE HEAT FOR SHIFTING
	void VoltageMonitor(Solenoid *gearUp, Solenoid *gearDown, CrioFile *currentSensor, 
			AnalogChannel *a, DriverStationLCD *driverStationLCD)
	//monitors the amount of voltage being drawn 
	{
		if(currentSensor->CheckVoltage(a) >= 3.5)
		{
			voltageCounter++;
			
			if(voltageCounter >= 15)
				//if it gets to high, auto shift down
				
			{
				gearUp->Set(false);
				gearDown->Set(true);
				voltageCounter = 0;
			}
		}
		else if(currentSensor->CheckVoltage(a) < 3.5)
		{
			voltageCounter = 0;
		}
		
		driverStationLCD->Printf((DriverStationLCD::Line) 5, 1, "Counter: %f", 
				voltageCounter);

		driverStationLCD->UpdateLCD();
		//printf("Counter: %f \n", voltageCounter);
	}
	/*
	void printFile()
	{
		string line;
		ifstream fileSave;
		if (fileSave.is_open())
		  {
		    while ( getline (fileSave,line) )
		    {
		      printf("%s\n",line);
		    }
		    fileSave.close();
		  }
		  else printf("Unable to open file"); 
	}
	*/
};

#endif
