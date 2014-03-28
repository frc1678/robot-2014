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
		/*string currentLogNum = "";
		fileSave.open("LogNum.txt");
		istream& reader>>(fileSave,currentLogNum);
		(int)currentLogNum;
		fileSave.close();
		string newFileName = (currentLogNum + ".txt");*/
		fileSave.open("File.txt");
		currentTimer->Reset();
		currentTimer->Start();
		currentlyLogging = true;
		fileSave<<"Time: "<<"Current:\n\r";
		fileSave.close();
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

		fileSave.open("File.txt", ios::app); //temp

		if(currentlyLogging)
		{
		float current = a->GetVoltage();
		current = fabs(2.5-current);
		double time = currentTimer->Get();
		fileSave<<time<<" "<<current<<"\r\n";
		
		if(fileSave.fail())//test to see if file opened
		{
			printf("File did not open.\n");
		}
		}

		fileSave.close();
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
