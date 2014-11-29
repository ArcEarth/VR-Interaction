#include "LeapMotion.h"
#include <iostream>

Platform::Devices::LeapMotion::LeapMotion(bool useEvent)
{
	if (useEvent)
		LeapController.addListener(*this);
}

Platform::Devices::LeapMotion::~LeapMotion()
{
	LeapController.removeListener(*this);
}

Leap::Controller & Platform::Devices::LeapMotion::Controller() {
	return LeapController;
}

const Leap::Controller & Platform::Devices::LeapMotion::Controller() const {
	return LeapController;
}

void Platform::Devices::LeapMotion::onConnect(const Leap::Controller & controller)
{
	DeviceConnected(controller);
	std::cout << "[Leap] Device Connected." << std::endl;
}

void Platform::Devices::LeapMotion::onDisconnect(const Leap::Controller & controller)
{
	DeviceConnected(controller);
	std::cout << "[Leap] Device Disconnected." << std::endl;
}

void Platform::Devices::LeapMotion::onFrame(const Leap::Controller & controller)
{
	CurrentFrame = controller.frame();
	FrameArrived(controller);

	UserHandsEventArgs e{ controller };

	if (CurrentFrame.hands().count() > PrevHandsCount)
	{
		PrevHandsCount = CurrentFrame.hands().count();
		HandsTracked(e);
	}
	else
	{
		PrevHandsCount = CurrentFrame.hands().count();
		HandsLost(e);
	}
	if (CurrentFrame.hands().count() > 0)
	{
		HandsMove(e);
	}

	//std::cout << "[Leap] Frame arrived." << std::endl;
}
