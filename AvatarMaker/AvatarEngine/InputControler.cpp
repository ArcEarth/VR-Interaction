////////////////////////////////////////////////////////////////////////////////
// Filename: InputControler.cpp
////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "InputControler.h"
#include <iostream>
#include <fstream>
#define FILTERTIMER 0x1001
//The Filter-timer's ID //Added by Yupeng

#define CUTTIME 150 
//The sync time for multi-sensor gesture
//Or the latency for eliminate jitter
//Added by Yupeng

//#include "raw_mouse.h"
const static InputControler::Gestures RightHandGestureCode[8]={InputControler::FREE,InputControler::SLAP,InputControler::PINCH,InputControler::UNKNOWN,InputControler::FINGERGUN,InputControler::FIST,InputControler::FIST,InputControler::FIST};
const static InputControler::Gestures LeftHandGestureCode[8]={InputControler::FREE,InputControler::FINGERGUN,InputControler::PINCH,InputControler::FIST,InputControler::SLAP,InputControler::FIST,InputControler::UNKNOWN,InputControler::FIST};

//const static std::string ConfigFileName = "InputMouseConfig.ini";

InputControler::InputControler(HWND hWnd)
	: m_MouseControler(false,hWnd)
{
	for(int i=0; i<256; i++)
	{
		m_keys[i] = false;
	}
}

InputControler::~InputControler()
{
}

void InputControler::KeyDown(unsigned int input)
{
	// If a key is pressed then save that state in the key array.
	m_keys[input] = true;
	return;
}


void InputControler::KeyUp(unsigned int input)
{
	// If a key is released then clear that state in the key array.
	m_keys[input] = false;
	return;
}


bool InputControler::IsKeyDown(unsigned int key)
{
	// Return what state the key is in (pressed/not pressed).
	return m_keys[key];
}

bool InputControler::IsKeyHit(unsigned int key)
{
	bool hr = m_keys[key];
	m_keys[key] = false;
	return hr;
}

bool InputControler::ReadMouse(HWND hWnd,HRAWINPUT input){
	auto index = m_MouseControler.MouseInputDispicher(hWnd,input);
	return false;
}

