//#ifdef SYTEM_CONTROLER_CPP
#pragma once
#include "stdafx.h"
#include "systemclass.h"
#include "DragManager.h"
#include "FreeCursor.h"
#include "GrowManager.h"
#include "SweepManager.h"
#include "VolumeSculptManager.h"
#include "CutManager.h"
#include "DriveManager.h"
#include "PaintManager.h"
//#include "ManipulationManager.h"
//#include "NaiveWarpper.h"
#include "PlayerSkeletonUpdater.h"
#include <DirectXColors.h>
#include <iostream>
#include <DDSTextureLoader.h>
#include <ScreenGrab.h>
//#include <amp.h>

#define ToString(name) #name 
static SystemControler* ApplicationHandle = 0;
static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

static const wchar_t* ToolsName[]={L"Cursor",L"Drag",L"Grow",L"Sweep",L"Sculpt",L"Cut",L"Sketch",L"Drive",L"Cursor",L"Eyedropper",L"Paint",L"Fill Color",L"Manipulation",L"NULL_1",L"NULL_2"};

std::ostream& operator << (std::ostream& lhs,SystemControler::ToolsSet rhs)
{
	static const char* ToolsName[]={"Cursor","Drag","Grow","Sweep","Sculpt","Cut","Sketch","Drive","Cursor","Eyedropper","Paint","Fill Color","Manipulation","NULL_1","NULL_2"};
	if ((unsigned int)rhs>=SystemControler::ToolsSet::Count)
		return lhs<<"IllegalEnum("<<(unsigned int)rhs<<')'<<std::endl;
	return lhs<<ToolsName[rhs];
}

static const wchar_t* HandSwitchName[]={L"Left  Hand",L"Right Hand",L"DoubleHand"};

std::ostream& operator << (std::ostream& lhs,SystemControler::HandSwitch rhs)
{
	static const char* HandSwitchName[]={"Left  Hand","Right Hand","DoubleHand"};
	if ((unsigned int)rhs>SystemControler::DoubleHand)
		return lhs<<"IllegalEnum("<<(unsigned int)rhs<<')'<<std::endl;
	return lhs<<HandSwitchName[rhs];
}

static const wchar_t* ModeName[]={
	L"Paused" ,
	L"Scanning",
	L"",
	L"",
	L"",
	L"",
	L"",
};

static const DirectX::XMVECTORF32 ModeColorSchedle[]=
{
	DirectX::Colors::White,			//"PAUSE"
	DirectX::Colors::AliceBlue,		//Scanning
	DirectX::Colors::LightGreen,	//Attaching
	DirectX::Colors::LightGreen,	//SpitalEditing
	DirectX::Colors::Gold,			//ReferenceEditng
	DirectX::Colors::MediumPurple,	//ReferencePainting
	DirectX::Colors::MediumPurple,  //SpatialPainting
	DirectX::Colors::Azure,
};


std::ostream& operator << (std::ostream& lhs,const SystemControler::SystemMode &rhs)
{
	static const char* ModeName[]={
		"PAUSE" ,
		"SCANING",
		"ATTACHING",
		"SPECIAL_EDTING",
		"REFERENCE_EDING",
		"PAINTING",
		"SPATIAL_PAINT",
		"ANMATING",
	};

	if ((unsigned int)rhs>SystemControler::MODE_ANIMATE)
		return lhs<<"IllegalEnum("<<(unsigned int)rhs<<')'<<std::endl;
	return lhs<<ModeName[rhs];
}

HWND CreateDebugConsoleWindow()
{	
	if( ! AllocConsole() )
		return NULL;
	HWND hConsole = GetConsoleWindow();
	MoveWindow(hConsole,-800,0,800,500,1);

#pragma warning( push )
#pragma warning( disable: 4996 )
	// Reopen file handles for stdin,stdout and sterr to point to
	// the newly created console
	freopen( "CONIN$", "r", stdin );
	freopen( "CONOUT$", "w", stdout );
	freopen( "CONOUT$", "w", stderr );

#pragma warning( pop )

	std::cout << "DEBUG console has been created" << std::endl;

	return hConsole;
}

// Methods for Initialize APP Window
LRESULT CALLBACK SystemControler::MessageHandler(HWND hwnd, UINT umsg, WPARAM wparam, LPARAM lparam)
{
	switch(umsg)
	{
		// Check if a key has been pressed on the keyboard.
	case WM_KEYDOWN:
		{
			// If a key is pressed send it to the input object so it can record that state.
			m_pInput->KeyDown((unsigned int)wparam);
			return 0;
		}

		// Check if a key has been released on the keyboard.
	case WM_KEYUP:
		{
			// If a key is released then send it to the input object so it can unset the state for that key.
			m_pInput->KeyUp((unsigned int)wparam);
			return 0;
		}

		//Handel the mouse(hand gesture glove) input
	case WM_INPUT:
		{
			m_pInput->ReadMouse(hwnd,(HRAWINPUT)lparam);
			return 0;
		}
		// Any other messages send to the default message handler as our application won't make use of them.
	default:
		{
			return DefWindowProc(hwnd, umsg, wparam, lparam);
		}
	}
}


void SystemControler::InitializeWindows(unsigned int& screenWidth, unsigned int& screenHeight)
{
	WNDCLASSEX wc;
	DEVMODE dmScreenSettings;
	int posX, posY;


	// Get an external pointer to this object.	
	ApplicationHandle = this;

	// Get the instance of this application.
	m_hinstance = GetModuleHandle(NULL);

	// Give the application a name.
	m_applicationName = L"BodyAvatar";

	// Setup the windows class with default settings.
	wc.style         = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc   = WndProc;
	wc.cbClsExtra    = 0;
	wc.cbWndExtra    = 0;
	wc.hInstance     = m_hinstance;
	wc.hIcon		 = LoadIcon(NULL, IDI_WINLOGO);
	wc.hIconSm       = wc.hIcon;
	wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszMenuName  = NULL;
	wc.lpszClassName = m_applicationName;
	wc.cbSize        = sizeof(WNDCLASSEX);

	// Register the window class.
	RegisterClassEx(&wc);

	// Determine the resolution of the clients desktop screen.
	screenHeight = GetSystemMetrics(SM_CYSCREEN);
	screenWidth  = GetSystemMetrics(SM_CXSCREEN);

	// Setup the screen settings depending on whether it is running in full screen or in windowed mode.
	if(FULL_SCREEN)
	{
		// If full screen set the screen to maximum size of the users desktop and 32bit.
		memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
		dmScreenSettings.dmSize       = sizeof(dmScreenSettings);
		dmScreenSettings.dmPelsWidth  = (unsigned long)screenWidth;
		dmScreenSettings.dmPelsHeight = (unsigned long)screenHeight;
		dmScreenSettings.dmBitsPerPel = 32;			
		dmScreenSettings.dmFields     = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

		// Change the display settings to full screen.
		ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN);

		// Set the position of the window to the top left corner.
		posX = posY = 0;
	}
	else
	{
		// If windowed then set it to 800x600 resolution.
		//screenWidth  = 1680;
		//screenHeight = 1050;
		//screenWidth  = 1024;
		//screenHeight = 768;

		// Place the window in the middle of the screen.
		posX = (GetSystemMetrics(SM_CXSCREEN) - screenWidth)  / 2;
		posY = (GetSystemMetrics(SM_CYSCREEN) - screenHeight) / 2;
	}

	// Create the window with the screen settings and get the handle to it.
	m_hwnd = CreateWindowEx(WS_EX_APPWINDOW, m_applicationName, m_applicationName, 
		WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_POPUP,
		posX, posY, screenWidth, screenHeight, NULL, NULL, m_hinstance, NULL);

	// Bring the window up on the screen and set it as main focus.
	ShowWindow(m_hwnd, SW_SHOW);
	SetForegroundWindow(m_hwnd);
	SetFocus(m_hwnd);

	// Hide the mouse cursor.
	ShowCursor(false);

	return;
}


void SystemControler::ShutdownWindows()
{
	// Show the mouse cursor.
	ShowCursor(true);

	// Fix the display settings if leaving full screen mode.
	if(FULL_SCREEN)
	{
		ChangeDisplaySettings(NULL, 0);
	}

	// Remove the window.
	DestroyWindow(m_hwnd);
	m_hwnd = NULL;

	// Remove the application instance.
	UnregisterClass(m_applicationName, m_hinstance);
	m_hinstance = NULL;

	// Release the pointer to this class.
	ApplicationHandle = NULL;

	return;
}


LRESULT CALLBACK WndProc(HWND hwnd, UINT umessage, WPARAM wparam, LPARAM lparam)
{
	switch(umessage)
	{
		// Check if the window is being destroyed.
	case WM_DESTROY:
		{
			PostQuitMessage(0);
			return 0;
		}

		// Check if the window is being closed.
	case WM_CLOSE:
		{
			PostQuitMessage(0);		
			return 0;
		}

		// All other messages pass to the message handler in the system class.
	default:
		{
			return ApplicationHandle->MessageHandler(hwnd, umessage, wparam, lparam);
		}
	}
}
//#endif // SYTEM_CONTROLER_CPP
