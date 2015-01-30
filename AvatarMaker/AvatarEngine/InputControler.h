////////////////////////////////////////////////////////////////////////////////
// Filename: InputControler.h
////////////////////////////////////////////////////////////////////////////////
#ifndef _InputControler_H_
#define _InputControler_H_

#include "KinectControler.h"

#include "stdafx.h"
#include "resource.h"
#include "TIMER.h"
#include "MultiMouseControler.h"
//#include "NuiSkeletonStream.h"
//#include "SkeletonStreamViewer.h"
#define N_MOUSE_MAX 5
//char GestureName[9][8]={"Free","Pinch","Sculpt","Null","Skech","Null","Null","Null","Push"};
//int GestureBuffer[N_MOUSE_MAX] = {0};

////////////////////////////////////////////////////////////////////////////////
// Class name: InputControler
////////////////////////////////////////////////////////////////////////////////
class InputControler
	: public KinectControler
{
public:
	//enum Gestures
	//{
	//	GESTURE_FREE=0x000,
	//	GESTURE_NULL=0x000,
	//	GESTURE_PINCH=0x001,
	//	GESTURE_SCULPT=0x002,
	//	GESTURE_SKECH=0x004,
	//	GESTURE_PUSH=0x008,
	//};
	enum Gestures
	{
		FREE		=0,
		UNKNOWN		=0,
		PINCH		=1,
		SLAP		=2,
		FINGERGUN	=3,
		FIST		=4,
	};

	InputControler(HWND hWnd = nullptr);
	~InputControler();

	void KeyDown(unsigned int);
	void KeyUp(unsigned int);

	// Query the key 's state
	bool IsKeyDown(unsigned int);
	// Query if the key is pressed and reset the key's state if it's pressed
	bool IsKeyHit(unsigned int);

	bool ReadMouse(HWND hWnd,HRAWINPUT input);
	// The function returns the value if the gestures changed

	//Do not change this values outside the input class
	const bool IsGestureEvent();
	const MultiMouseControler& Mice() const
		{return m_MouseControler;}
	MultiMouseControler& Mice()
		{return m_MouseControler;}

public:
	// =3,=2
	int MouseIndex_LeftHand,MouseIndex_RightHand; 
private:
	InputControler(const InputControler&);
	
	MultiMouseControler			m_MouseControler;
	bool						m_keys[256];

	//INuiSensor* m_pNuiSensor;
	//std::unique_ptr<Nui::NuiSkeletonStream> m_pSkeletonStream;
	//std::unique_ptr<SkeletonStreamViewer> m_pSkeletonStreamViewer;
//	 m_Kinect;
};
#endif