////////////////////////////////////////////////////////////////////////////////
// Filename: cameraclass.h
////////////////////////////////////////////////////////////////////////////////
#ifndef _CAMERACLASS_H_
#define _CAMERACLASS_H_
#pragma once

//////////////
// INCLUDES //
//////////////

#include <DirectXMath.h>
#include "Locatable.h"

////////////////////////////////////////////////////////////////////////////////
// Class name: Camera
////////////////////////////////////////////////////////////////////////////////

namespace DirectX
{
	namespace Scene
	{
		enum EyesEnum : int
		{
			Eye_Left = 0,
			Eye_Right = 1,
			Eye_Count = 2,
		};

		struct CameraBuffer : public ILocatable, public IOriented
		{
			DirectX::XMFLOAT4X4	ViewProjectionMatrix;
			DirectX::XMFLOAT4	CameraPosition;
			DirectX::XMFLOAT4	CameraFocus;
			DirectX::XMFLOAT4X4 ViewMatrix;
			DirectX::XMFLOAT4X4 ProjectionMatrix;
		};

		class ICamera abstract : public ILocatable, public IOriented
		{
		public:
			virtual DirectX::XMMATRIX GetViewMatrix() const = 0;
			virtual DirectX::XMMATRIX GetProjectionMatrix() const = 0;
			virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;
			virtual void SetFov(float fovRadius, float aspectRatioHbyW) = 0;
		};

		class IStereoCamera abstract : public ILocatable, public IOriented
		{
		public:
			virtual DirectX::XMMATRIX GetViewMatrix(EyesEnum eye) const = 0;
			virtual DirectX::XMMATRIX GetProjectionMatrix(EyesEnum eye) const = 0;
			virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;
		};
	}
}

//class PerspectiveCamera 
//	: public DirectX::RigidObject , public ICamera
//{
//private:
//	UINT m_Slot;
//	ID3D11Buffer *m_pGPUBuffer;
//	CameraBuffer m_Buffer;
//
//	enum DirtyFlags
//	{
//		ViewMatrix			= 0x1,
//		ProjectionMatrix	= 0x2,
//		FocusDepth			= 0x4,
//	};
//	unsigned int m_DirtyFlag;
//public:
//	float FOV , Aspect , Near , Far;
//
//public:
//	PerspectiveCamera(ID3D11Device *pDevice , UINT CameraBufferSlot , const float FovAngleY , const float AspectHbyW , const float NearZ = 0.1f , const float FarZ = 1000.0f);
//	PerspectiveCamera(const PerspectiveCamera&);
//	~PerspectiveCamera();
//	virtual DirectX::XMMATRIX GetViewMatrix() const;
//	virtual DirectX::XMMATRIX GetProjectionMatrix() const;
//	virtual DirectX::XMVECTOR GetPosition() const;
//
//	ID3D11Buffer*	GetBuffer() const;
//	ID3D11Buffer*	UpdateBuffer(ID3D11DeviceContext* pDeviceContext);
//
//	void Render(ID3D11DeviceContext *pContext);
//
//	void Move(DirectX::FXMVECTOR DisplacementVector);
//	void Turn(DirectX::FXMVECTOR TurningQuternion);
//};

#endif