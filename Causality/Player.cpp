#include "Player.h"

DirectX::XMMATRIX Player::GetViewMatrix(DirectX::Scene::EyesEnum eye) const
{
	using namespace DirectX;
	auto pose = m_pRift->EyePoses((DirectX::Scene::EyesEnum) eye);

	XMVECTOR loc = (XMVECTOR) Position() + DirectX::XMVector3Rotate((XMVECTOR) pose.Position, pose.Orientation);
	XMVECTOR rot = Orientation() * pose.Orientation;
	XMVECTOR foward = XMVector3Rotate(Foward, rot);
	XMVECTOR up = XMVector3Rotate(Up, rot);
	return XMMatrixLookToRH(loc, foward, up);
}

DirectX::XMMATRIX Player::GetProjectionMatrix(DirectX::Scene::EyesEnum eye) const
{
	return m_pRift->EyeProjection(eye);
}

// This ignore the effect of Oculus

void Player::FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir)
{
	using namespace DirectX;
	Foward = XMVector3Normalize(focusPoint - (XMVECTOR) Position());
	Up = upDir;
	SetOrientation(XMQuaternionIdentity());
}

const Platform::Fundation::Vector3 & Player::Position() const
{
	return m_BodyPose.Position;
}

void Player::SetPosition(Platform::Fundation::Vector3 p)
{
	m_BodyPose.Position = p;
}

const Platform::Fundation::Quaternion & Player::Orientation() const
{
	return m_BodyPose.Orientation;
}

void Player::SetOrientation(Platform::Fundation::Quaternion q)
{
	m_BodyPose.Orientation = q;
}

DirectX::XMMATRIX Player::GetViewMatrix() const
{
	using namespace DirectX;
	XMVECTOR loc = (XMVECTOR) Position();
	XMVECTOR rot = Orientation();
	XMVECTOR foward = XMVector3Rotate(Foward, rot);
	XMVECTOR up = XMVector3Rotate(Up, rot);
	return XMMatrixLookToRH(loc, foward, up);
}

DirectX::XMMATRIX Player::GetProjectionMatrix() const
{
	using namespace DirectX;
	return XMMatrixPerspectiveFovRH(Fov, AspectRatio, 0.01f,100.0f);
}

void Player::SetFov(float fovRadius, float aspectRatioHbyW)
{
	Fov = fovRadius; AspectRatio = aspectRatioHbyW;
}
