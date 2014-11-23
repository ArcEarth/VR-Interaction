#pragma once
#include "OculusRift.h"

class Player : public DirectX::Scene::IStereoCamera ,virtual public DirectX::Scene::ICamera
{
public:
	Player()
	{
	}

	~Player()
	{
	}

	void EnableStereo(const std::shared_ptr<Platform::Devices::OculusRift>& pRift)
	{
		m_pRift = pRift;
	}

	void DisableStoreo()
	{
		m_pRift = nullptr;
	}

	bool IsStereoEnabled()
	{
		return m_pRift != nullptr;
	}

	virtual DirectX::XMMATRIX GetViewMatrix(DirectX::Scene::EyesEnum eye) const;

	virtual DirectX::XMMATRIX GetProjectionMatrix(DirectX::Scene::EyesEnum eye) const;

	// This ignore the effect of Oculus
	virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir);

	virtual const Platform::Fundation::Vector3&    Position() const override;
	virtual void  SetPosition(const Platform::Fundation::Vector3& p) override;
	virtual const Platform::Fundation::Quaternion& Orientation() const override;
	virtual void  SetOrientation(const Platform::Fundation::Quaternion &q) override;
	// Inherited via ICamera
	virtual DirectX::XMMATRIX GetViewMatrix() const override;
	virtual DirectX::XMMATRIX GetProjectionMatrix() const override;
	virtual void SetFov(float fovRadius, float aspectRatioHbyW) override;
private:
	Platform::Fundation::Vector3 Foward = { 0, 0, -1.0f }, Up = { 0, 1.0f, 0 };
	Platform::Fundation::StaticPose m_BodyPose;
	float Fov, AspectRatio;
	std::shared_ptr<Platform::Devices::OculusRift> m_pRift;


};

