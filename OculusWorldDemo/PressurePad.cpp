#include "PressurePad.h"
#include <functional>

PressurePad::PressurePad(const TactonicDevice &device, UINT usage)
	: m_Device(device), m_Usage(usage)
{
	m_ForceFrame = Tactonic_CreateFrame(m_Device);                   // Create a TactonicFrame for this m_Device
	m_TouchFrame = TactonicTouch_CreateFrame(m_Device);              // Create a TactonicTouchFrame for this m_Device
	TactonicTouch_CreateDetector(m_Device);                          // Create the touch detector for the m_Device
	TactonicTouch_AddTouchCallback(m_Device, touchCallback);         // Add a TactonicTouchFrame callback method
	TactonicTouch_StartDetector(m_Device);                           // Start the touch detector
	Tactonic_AddFrameCallback(m_Device, frameCallback);              // Add a TactonicFrame callback method
	Tactonic_StartDevice(m_Device);                                  // Start the m_Device
}

PressurePad::~PressurePad()
{
	TouchFrameReady.disconnect_all_slots();
	ForecFrameReady.disconnect_all_slots();
	Tactonic_StopDevice(m_Device);
	TactonicTouch_StopDetector(m_Device);
	Tactonic_RemoveFrameCallback(m_Device, frameCallback);
	TactonicTouch_RemoveTouchCallback(m_Device, touchCallback);
	//TactonicTouch_DestroyDetector(m_Device);
	//TactonicTouch_DestroyFrame(m_TouchFrame);
	//Tactonic_DestroyFrame(m_ForceFrame);
}

std::shared_ptr<PressurePad> PressurePad::Create(const TactonicDevice &device, UINT usage)
{
	auto pThis = std::make_shared<PressurePad>(device, usage);
	DeviceTable[device.serialNumber] = pThis;
	return pThis;
}

std::shared_ptr<PressurePad> PressurePad::Create(UINT deviceIndex, UINT usageFlag)
{
	if (GetDeviceList()->numDevices < deviceIndex + 1)
		return nullptr;
	else
	{
		return Create(GetDeviceList()->devices[deviceIndex], usageFlag);
	}
}

void PressurePad::frameCallback(TactonicFrameEvent* e)
{
	if (DeviceTable.find(e->device.serialNumber) == DeviceTable.end())
		return;
	auto pad = DeviceTable[e->device.serialNumber].lock();
	if (pad && (pad->m_Usage & Usage_ForceMap))
	{
		Tactonic_CopyFrame(e->frame, pad->m_ForceFrame);
		pad->ForecFrameReady(pad->m_ForceFrame);
	}
}

void PressurePad::touchCallback(TactonicTouchEvent* e)
{
	unsigned long serial = TouchToDeviceSerialMap[e->device.serialNumber];
	if (DeviceTable.find(serial) == DeviceTable.end())
		return;
	auto pad = DeviceTable[serial].lock();
	if (pad && (pad->m_Usage & Usage_TouchPoints))
	{
		TactonicTouch_CopyFrame(e->frame, pad->m_TouchFrame);
		pad->TouchFrameReady(pad->m_TouchFrame);
	}
}

void PressurePad::Start()
{
	Tactonic_StartDevice(m_Device); // Start the device
}

void PressurePad::Stop()
{
	Tactonic_StopDevice(m_Device);	// Stop the device
}

const TactonicDeviceList* PressurePad::GetDeviceList()
{
	if (!DeviceList)
		DeviceList = Tactonic_GetDeviceList();
	return DeviceList;//Tactonic_GetDeviceList();
}

void PressurePad::DestroyDeviceList()
{
	//DeviceList.reset();
}


std::map<unsigned long, unsigned long> PressurePad::TouchToDeviceSerialMap;
std::map<unsigned long, std::weak_ptr<PressurePad>> PressurePad::DeviceTable;
PressurePad::Initializer PressurePad::_initializer;
TactonicDeviceList* PressurePad::DeviceList;

PressurePad::Initializer::Initializer()
{
	TouchToDeviceSerialMap[3131961357U] = 17498432U;
}
