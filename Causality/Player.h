#pragma once
#include "OculusRift.h"

class Player : public DirectX::Scene::IStereoCamera
{
public:
	Player(const std::shared_ptr<Platform::Devices::OculusRift> &pRift)
		:m_pRift(pRift)
	{
		std::shared_ptr<int> pI;
		std::shared_ptr<const int> pCI;
		pCI = pI;
	}

	~Player()
	{
	}

	virtual DirectX::XMMATRIX GetViewMatrix(DirectX::Scene::EyesEnum eye) const
	{
		using namespace DirectX;
		auto pose = m_pRift->EyePoses((DirectX::Scene::EyesEnum) eye);

		XMVECTOR loc = (XMVECTOR)Position() + DirectX::XMVector3Rotate((XMVECTOR) pose.Position, pose.Orientation);
		XMVECTOR rot = Orientation() * pose.Orientation;
		XMVECTOR foward = XMVector3Rotate(Foward, rot);
		XMVECTOR up = XMVector3Rotate(Up, (XMVECTOR) pose.Orientation);
		return XMMatrixLookToRH(loc, foward, up);
	}

	virtual DirectX::XMMATRIX GetProjectionMatrix(DirectX::Scene::EyesEnum eye) const
	{
		return m_pRift->EyeProjection(eye);
	}

	// This ignore the effect of Oculus
	virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir)
	{
		using namespace DirectX;
		Foward = focusPoint - (XMVECTOR)Position();
		Up = upDir;
		SetOrientation(XMQuaternionIdentity());
	}

	virtual const Platform::Fundation::Vector3&    Position() const
	{
		return m_BodyPose.Position;
	}
	virtual void  SetPosition(Platform::Fundation::Vector3 p)
	{
		m_BodyPose.Position = p;
	}
	virtual const Platform::Fundation::Quaternion& Orientation() const
	{
		return m_BodyPose.Orientation;
	}
	virtual void  SetOrientation(Platform::Fundation::Quaternion q)
	{
		m_BodyPose.Orientation = q;
	}

private:
	Platform::Fundation::Vector3 Foward = { 0, 0, -1.0f }, Up = { 0, 1.0f, 0 };
	Platform::Fundation::StaticPose m_BodyPose;
	std::shared_ptr<Platform::Devices::OculusRift> m_pRift;
};

