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
	ofstream fileSave; //create new output file 
	bool currentlyLogging;
	
public:
	CrioFile()
	{
		//a = new AnalogChannel(3);
		//fileSave.open("File.txt");
		currentTimer = new Timer();
		currentlyLogging = false;
	}
		
	/*float GetCurrent()
	{
		float returnMe = a->GetVoltage();
		return returnMe;
	}*/
	void StartLog()
	{
		fileSave.open("File.txt");
		currentTimer->Reset();
		currentTimer->Start();
		currentlyLogging = true;
		fileSave<<"Time: "<<"Current:\n\r";
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
		if(currentlyLogging)
		{
		float current = a->GetVoltage();
		double time = currentTimer->Get();
		fileSave<<time<<" "<<current<<"\r\n";
		
		if(fileSave.fail())//test to see if file opened
		{
			printf("File did not open.\n");
		}
		}
	}
	
	void EndLog()
	{
		if(currentlyLogging)
		{
			currentlyLogging = false;
			fileSave.close();
			currentTimer->Stop();
		}
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
