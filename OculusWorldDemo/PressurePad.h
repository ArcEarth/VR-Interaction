#pragma once
#include <map>
#include <memory>
#include <Tactonic.h>
#include <TactonicTouch.h>
#include <boost/signals2.hpp>

class PressurePad
{
public:
	enum PressurePadUsageType : UINT
	{
		Usage_ForceMap = 1,
		Usage_TouchPoints = 2,
	};

	~PressurePad();

	static std::shared_ptr<PressurePad> Create(const TactonicDevice &device,UINT usageFlag);
	static std::shared_ptr<PressurePad> Create(UINT deviceIndex, UINT usageFlag);
	static const TactonicDeviceList* GetDeviceList();
	static void DestroyDeviceList();

	boost::signals2::signal<void(TactonicTouchFrame* e)> TouchFrameReady;
	boost::signals2::signal<void(TactonicFrame* e)>      ForecFrameReady;

	TactonicFrame* CurrentForceFrame();
	TactonicTouchFrame* CurrentTouchFrame();

	// Do not use it!!!
	PressurePad(const TactonicDevice &device, UINT usage);


private:

	//friend std::shared_ptr < PressurePad > ;
	//template <class _Types...>
	//friend std::shared_ptr < PressurePad > std::make_shared<PressurePad>(_Types&&... _Args);

	void Start();
	void Stop();

	UINT                 m_Usage;
	TactonicDevice       m_Device;              // The device that we are monitoring
	TactonicFrame	   * m_ForceFrame;          // An Anti-Aliased Frame of data from the device
	TactonicTouchFrame * m_TouchFrame;          // A list of touches that were detected for a given TactonicFrame

	static std::map<unsigned long, std::weak_ptr<PressurePad>> DeviceTable;
	static std::map<unsigned long, unsigned long> TouchToDeviceSerialMap;
	static TactonicDeviceList *DeviceList;
	static void frameCallback(TactonicFrameEvent* e);

	static void touchCallback(TactonicTouchEvent* e);

	static struct Initializer
	{
		Initializer();
	} _initializer;

	friend struct Initializer;
};

typedef std::shared_ptr<PressurePad> SPPresurePad;
