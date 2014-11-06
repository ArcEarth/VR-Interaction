#include "TouchPad.h"
#include <functional>
#include <Tactonic.h>
#include <TactonicTouch.h>
#include "Util\Filter.h"
#include <SimpleMath.h>

using namespace Platform::Input;
using namespace DirectX::SimpleMath;
using namespace std;

struct VectorNormalizer : public unary_function<Vector2,double>
{
	double operator()(const Vector2& v)
	{
		return (double) v.Length();
	}
};

typedef LowPassDynamicFilter<Vector2,double , VectorNormalizer> TraceFilter;

class TouchPad::Impl
{
public:
	unsigned             m_Usage;
	TactonicDevice       m_Device;              // The device that we are monitoring
	TactonicFrame	   * m_ForceFrame;          // An Anti-Aliased Frame of data from the device
	TactonicTouchFrame * m_TouchFrame;          // A list of touches that were detected for a given TactonicFrame
	TouchPointsFrame	 m_TouchPointsFrame;
	PressureMapFrame	 m_PressureMapFrame;
	double				 m_Frequency;

	map<int, TraceFilter> m_Filters;

	Platform::Event<TouchPointsFrame*> TouchFrameReady;
	Platform::Event<PressureMapFrame*> ForceFrameReady;

	Impl(const TactonicDevice &device, unsigned usage)
		: m_Device(device), m_Usage(usage), m_Frequency(60)
	{
		// Add the filter callback
		TouchFrameReady += std::bind(&TouchPad::Impl::ApplyTouchFilter, this, placeholders::_1);
		m_ForceFrame = Tactonic_CreateFrame(m_Device);                   // Create a TactonicFrame for this m_Device
		m_TouchFrame = TactonicTouch_CreateFrame(m_Device);              // Create a TactonicTouchFrame for this m_Device
		TactonicTouch_CreateDetector(m_Device);                         // Create the touch detector for the m_Device
		TactonicTouch_AddTouchCallback(m_Device, touchCallback);         // Add a TactonicTouchFrame callback method
		TactonicTouch_StartDetector(m_Device);                           // Start the touch detector
		Tactonic_AddFrameCallback(m_Device, frameCallback);              // Add a TactonicFrame callback method
		Tactonic_StartDevice(m_Device);                               // Start the m_Device
	}

	~Impl()
	{
		TouchFrameReady.disconnect_all_slots();
		ForceFrameReady.disconnect_all_slots();
		Tactonic_RemoveFrameCallback(m_Device, frameCallback);
		TactonicTouch_RemoveTouchCallback(m_Device, touchCallback);
		Tactonic_StopDevice(m_Device);
		TactonicTouch_StopDetector(m_Device);
	}

	void ApplyTouchFilter(TouchPointsFrame* frame)
	{
		for (size_t i = 0; i < frame->numTouches; i++)
		{
			auto& t = frame->touches[i];
			auto& filter = m_Filters[t.id];
			filter.SetUpdateFrequency(&m_Frequency);
			if (t.touchtype == TouchType::TOUCH_DOWN)
			{
				filter.SetVelocityLow(0);
				filter.SetVelocityHigh( 1.0);
				filter.SetCutoffFrequencyLow(0.01);
				filter.SetCutoffFrequencyHigh(1.0);
				filter.Clear();
			}
			auto v = filter.Apply(Vector2(t.x, t.y));
			auto dv = filter.Delta();
			t.x = v.x; t.y = v.y;
			t.dx = dv.x; t.dy = dv.y;
			if (t.touchtype == TouchType::TOUCH_UP)
			{
				m_Filters.erase(t.id); //delete the filter
			}
		}
	}

	static std::shared_ptr<TouchPad::Impl> Create(const TactonicDevice &device, unsigned usageFlag);
	static std::shared_ptr<TouchPad::Impl> Create(unsigned deviceIndex, unsigned usageFlag);
	static const TactonicDeviceList* GetDeviceList();

	static std::map<unsigned long, std::weak_ptr<TouchPad::Impl>> DeviceTable;
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

TouchPad::~TouchPad()
{
}

void TouchPad::Initialize()
{
	TouchPad::Impl::GetDeviceList();
}

template <class T>
bool operator==(const std::weak_ptr<T> &lhs, nullptr_t)
{
	return lhs.expired();
}

std::shared_ptr<TouchPad::Impl> TouchPad::Impl::Create(const TactonicDevice &device, unsigned usage)
{
	if (DeviceTable[device.serialNumber] == nullptr)
	{
		auto pThis = std::make_shared<TouchPad::Impl>(device, usage);
		DeviceTable[device.serialNumber] = pThis;
		return std::move(pThis);
	}
	else
	{
		return DeviceTable[device.serialNumber].lock();
	}
}

std::shared_ptr<TouchPad::Impl> TouchPad::Impl::Create(unsigned deviceIndex, unsigned usageFlag)
{
	if (Impl::GetDeviceList()->numDevices < (int)deviceIndex + 1)
	{
		return nullptr;
	}
	else
	{
		return Impl::Create(Impl::GetDeviceList()->devices[deviceIndex], usageFlag);
	}
}


TouchPad::TouchPad(unsigned deviceIndex, unsigned usageFlag)
	: pImpl(Impl::Create(deviceIndex,usageFlag))
{
}

TouchPad::TouchPad(nullptr_t)
	: pImpl(nullptr)
{}

TouchPad::TouchPad()
	: pImpl(nullptr)
{}

unsigned Platform::Input::TouchPad::Rows() const
{
	return pImpl->m_Device.rows;
}

unsigned Platform::Input::TouchPad::Cols() const
{
	return pImpl->m_Device.cols;
}

PressureMapFrame* TouchPad::CurrentForceFrame()
{
	return &pImpl->m_PressureMapFrame;
}
TouchPointsFrame* TouchPad::CurrentTouchFrame()
{
	return &pImpl->m_TouchPointsFrame;
}


boost::signals2::signal<void(TouchPointsFrame* e)> &TouchPad::TouchFrameReady()
{
	return pImpl->TouchFrameReady;
}
boost::signals2::signal<void(PressureMapFrame* e)> &TouchPad::ForceFrameReady()
{
	return pImpl->ForceFrameReady;
}


void TouchPad::Impl::frameCallback(TactonicFrameEvent* e)
{
	if (DeviceTable.find(e->device.serialNumber) == DeviceTable.end())
		return;
	auto pad = DeviceTable[e->device.serialNumber].lock();
	if (pad && (pad->m_Usage & Usage_ForceMap))
	{
		Tactonic_CopyFrame(e->frame, pad->m_ForceFrame);
		std::memcpy((void*) &pad->m_PressureMapFrame, (void*) pad->m_ForceFrame, sizeof(PressureMapFrame));
		pad->ForceFrameReady(&pad->m_PressureMapFrame);
	}
}

void TouchPad::Impl::touchCallback(TactonicTouchEvent* e)
{
	unsigned long serial = TouchToDeviceSerialMap[e->device.serialNumber];
	if (DeviceTable.find(serial) == DeviceTable.end())
		return;
	auto pad = DeviceTable[serial].lock();
	if (pad && (pad->m_Usage & Usage_TouchPoints))
	{
		TactonicTouch_CopyFrame(e->frame, pad->m_TouchFrame);
		std::memcpy((void*) &pad->m_TouchPointsFrame, (void*) pad->m_TouchFrame, sizeof(PressureMapFrame));
		pad->TouchFrameReady(&pad->m_TouchPointsFrame);
	}
}

const TactonicDeviceList* TouchPad::Impl::GetDeviceList()
{
	if (!DeviceList)
		DeviceList = Tactonic_GetDeviceList();
	return DeviceList;//Tactonic_GetDeviceList();
}


std::map<unsigned long, unsigned long> TouchPad::Impl::TouchToDeviceSerialMap;
std::map<unsigned long, std::weak_ptr<TouchPad::Impl>> TouchPad::Impl::DeviceTable;
TouchPad::Impl::Initializer TouchPad::Impl::_initializer;
TactonicDeviceList* TouchPad::Impl::DeviceList;

TouchPad::Impl::Initializer::Initializer()
{
	TouchToDeviceSerialMap[3131961357U] = 17498432U;
	TouchToDeviceSerialMap[0U] = 17498432U;
}
