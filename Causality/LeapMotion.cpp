#include "LeapMotion.h"
#include <iostream>

using namespace Platform::Devices;

class LeapMotion::Listener : public Leap::Listener
{
public:
	Listener(LeapMotion* pLeap)
	{
		pOwner = pLeap;
	}
	virtual void onConnect(const Leap::Controller & controller) override;
	virtual void onDisconnect(const Leap::Controller &controller) override;
	virtual void onFrame(const Leap::Controller &controller) override;

	int							PrevHandsCount;
	Leap::Frame					CurrentFrame;
	LeapMotion*	pOwner;
};

Platform::Devices::LeapMotion::LeapMotion(bool useEvent, bool isFixed)
	:pListener(new Listener(this))
{
	using namespace DirectX;
	PrevConnectionStates = false;
	pHeadLocation = nullptr;
	pHeadOrientation = nullptr;
	Coordinate = XMMatrixScalingFromVector(XMVectorReplicate(0.001f)) * XMMatrixTranslation(0, 0, -0.50); // Default setting
	if (useEvent)
		LeapController.addListener(*pListener.get());
	if (!isFixed)
		LeapController.setPolicy(Leap::Controller::PolicyFlag::POLICY_OPTIMIZE_HMD);
}

Platform::Devices::LeapMotion::~LeapMotion()
{
	LeapController.removeListener(*pListener.get());
}

Leap::Controller & Platform::Devices::LeapMotion::Controller() {
	return LeapController;
}

const Leap::Controller & Platform::Devices::LeapMotion::Controller() const {
	return LeapController;
}

void Platform::Devices::LeapMotion::PullFrame()
{
	bool connected = LeapController.isConnected();
	if (connected && !PrevConnectionStates)
	{
		PrevConnectionStates = connected;
		pListener->onConnect(LeapController);
	}
	else if (!connected && PrevConnectionStates)
	{
		PrevConnectionStates = connected;
		pListener->onDisconnect(LeapController);
	}
	if (connected)
		pListener->onFrame(LeapController);
}

void Platform::Devices::LeapMotion::SetMotionProvider(DirectX::ILocatable * pHeadLoc, DirectX::IOriented * pHeadOrient)
{
	pHeadLocation = pHeadLoc;
	pHeadOrientation = pHeadOrient;
}

void Platform::Devices::LeapMotion::SetLeapCoord(const DirectX::Matrix4x4 & m)
{
	using namespace DirectX;
	Coordinate = XMMatrixScalingFromVector(XMVectorReplicate(0.001f)) * (XMMATRIX)m;
}

DirectX::XMMATRIX Platform::Devices::LeapMotion::ToWorldTransform() const
{
	using namespace DirectX;
	if (pHeadLocation && pHeadOrientation)
		return Coordinate * XMMatrixRotationQuaternion(pHeadOrientation->GetOrientation()) * XMMatrixTranslationFromVector(pHeadLocation->GetPosition());
	else
		return Coordinate;
}

void Platform::Devices::LeapMotion::Listener::onConnect(const Leap::Controller & controller)
{
	pOwner->DeviceConnected(controller);
	std::cout << "[Leap] Device Connected." << std::endl;
}

void Platform::Devices::LeapMotion::Listener::onDisconnect(const Leap::Controller & controller)
{
	pOwner->DeviceConnected(controller);
	std::cout << "[Leap] Device Disconnected." << std::endl;
}

void Platform::Devices::LeapMotion::Listener::onFrame(const Leap::Controller & controller)
{
	CurrentFrame = controller.frame();
	pOwner->FrameArrived(controller);

	UserHandsEventArgs e{ controller , pOwner->ToWorldTransform()};

	if (CurrentFrame.hands().count() > PrevHandsCount)
	{
		PrevHandsCount = CurrentFrame.hands().count();
		pOwner->HandsTracked(e);
	}
	else
	{
		PrevHandsCount = CurrentFrame.hands().count();
		pOwner->HandsLost(e);
	}
	if (CurrentFrame.hands().count() > 0)
	{
		pOwner->HandsMove(e);
	}

	//std::cout << "[Leap] Frame arrived." << std::endl;
}
