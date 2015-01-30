#pragma once
#include <Windows.h>
//#include "Player.h"
#include "TIMER.h"
#include <map>
#include <vector>
#include <string>

class MultiMouseControler
{
public:
	enum Event
	{
		WheelEvent = 0x1,
		ButtonEvent= 0x2,
	};
	struct Device
	{
		std::string		Name;
		HANDLE			hDevice;
		unsigned short	Priority;
		unsigned short	EventFlag;
		unsigned short	State;
		short			WheelDelta;
		TIMER			SteadyTimer;
	};
public:
	MultiMouseControler(bool IncludeRomoteDesktopMouse = false , HWND hWnd = nullptr);
	~MultiMouseControler(void);

	int MouseInputDispicher(HWND hWnd,HRAWINPUT input);

	inline const Device& operator[] (unsigned int index) const
		{ return m_Devices[index]; }
	inline Device& operator[] (unsigned int index)
		{ return m_Devices[index]; }
	inline size_t size() const 
		{return m_Devices.size();}

	inline double GetTimeFromLastUpdate() const
	{
		return m_Syncronizer.GetTime();
	}

	unsigned int GetDeviceIndexByName(const std::string & deviceName) const;

private:
	bool Initialize(bool IncludeRomoteDesktopMouse , HWND hWnd);
	std::pair<short,short> TranslateWINPINT(PRAWINPUT raw);

protected:
	bool					m_Initialized;
	std::map<HANDLE,short>	m_DeviceToIndexMap;
	std::vector<Device>		m_Devices;
	TIMER					m_Syncronizer;
};

