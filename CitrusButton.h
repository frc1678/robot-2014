#include "WPILib.h"

//Designed to get input from joysticks.
class CitrusButton
{
	bool output;
	bool oldInput;
	
	//Optional.
	Joystick *stick;
	int button;
public:
	CitrusButton(Joystick *tstick, int tbutton)
	{
		output = false;
		oldInput = false;
		stick = tstick;
		button = tbutton;
	}
	
	//All of these have clones with generic input. If we ever end up needing a button that's not from a joystick.
	
	//Call at the end of every loop (once per loop)!
	void Update(bool input)
	{
		oldInput = input;
	}
	void Update()
	{
		Update(stick->GetRawButton(button));
	}
	bool ButtonClicked(bool input)
	{
		//Return true the first time input's true after being false
		//false otherwise.
		bool returnMe = false;
		if(input != oldInput && input == true)
		{
			returnMe = true;
		}
		else
		{
			returnMe = false;
		}
		return returnMe;
	}
	bool ButtonClicked()
	{
		if(button != 0)
		{
			printf("00680069");
			return ButtonClicked(stick->GetRawButton(button));
		}
		return false;
	}
	bool ButtonReleased(bool input)
	{
		bool returnMe = false;
		if(input != oldInput && input == false)
		{
			returnMe = true;
		}
		else
		{
			returnMe = false;
		}
		return returnMe;
	}
	bool ButtonReleased()
	{
		if(button!= 0)
		{
			return ButtonReleased(stick->GetRawButton(button));
		}
		return false;
	}
	bool ButtonPressed(bool input)
	{
		return input;
	}
	bool ButtonPressed()
	{
		if(button!= 0)
		{
			return ButtonPressed(stick->GetRawButton(button));
		}
		return false;
	}
	
	//Reset to factory settings!
	void Reset()
	{
		output = false;
		oldInput = false;
	}
};

//Use the following like: input = TurnOn(myButton); input = Toggle(myButton, input);
bool TurnOn(CitrusButton *button)
{
	if(button->ButtonClicked())
	{
		return true;
	}
	return false;
}
bool TurnOff(CitrusButton *button)
{
	if(button->ButtonClicked())
	{
		return false;
	}
	return true;
}
bool Toggle(CitrusButton *button, bool input)
{
	if(button->ButtonClicked())
	{
		return !input;
	}
	return input;
}
