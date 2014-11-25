#include "LeapMotion.h"

Platform::Devices::LeapMotion::LeapMotion(bool useEvent)
{
	if (useEvent)
		LeapController.addListener(*this);
}

inline Platform::Devices::LeapMotion::~LeapMotion()
{
	LeapController.removeListener(*this);
}

inline Leap::Controller & Platform::Devices::LeapMotion::Controller() {
	return LeapController;
}

inline const Leap::Controller & Platform::Devices::LeapMotion::Controller() const {
	return LeapController;
}

inline void Platform::Devices::LeapMotion::onConnect(const Leap::Controller & controller)
{
	DeviceConnected(controller);
}

inline void Platform::Devices::LeapMotion::onDisconnect(const Leap::Controller & controller)
{
	DeviceConnected(controller);
}

inline void Platform::Devices::LeapMotion::onFrame(const Leap::Controller & controller)
{
	CurrentFrame = controller.frame();
	FrameArrived(controller);
}
