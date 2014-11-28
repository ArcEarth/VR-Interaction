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

		// Manually adjust advance camera paprameters
		class ICameraParameters
		{
			virtual void SetFov(float fovRadius, float aspectRatioHbyW) = 0;
		};

		// The basic functions for camera, to provide View/Projection Matrix, Setup Position and focus
		class ICameraBase abstract : public ILocatable, public IOriented
		{
		public:
			virtual ~ICameraBase()
			{}
			virtual size_t ViewCount() const = 0;
			virtual DirectX::XMMATRIX GetViewMatrix(size_t view) const = 0;
			virtual DirectX::XMMATRIX GetProjectionMatrix(size_t view) const = 0;
			virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;

			void XM_CALLCONV Move(FXMVECTOR p) { SetPosition((XMVECTOR) GetPosition() + XMVector3Rotate(p, GetOrientation())); }
			void XM_CALLCONV Rotate(FXMVECTOR q) { SetOrientation(XMQuaternionMultiply(q, GetOrientation())); }
			virtual bool XM_CALLCONV IsInView(FXMVECTOR pos) const { return true; }
		};

		// Control the logic of render target setup and post-processing needed for current camera
		class ICameraRenderControl
		{
		public:
			// Called in the beginning of per-frame
			virtual void BeginFrame() = 0;
			// Called in in the end of per frame, should call Prenset inside
			virtual void EndFrame() = 0;
			// Called in advance of per-view rendering
			virtual void SetView(size_t view) = 0;
		};

		class IMonolithCamera abstract : public ICameraBase , public ICameraParameters
		{
		public:
			virtual size_t ViewCount() const { return 1U; }
		};

		// StereoCamera usually need to setup different view ports for differnt view, and it should implement the ICameraRenderControlInterface
		class IStereoCamera abstract : public ICameraBase , public ICameraRenderControl
		{
		public:
			virtual size_t ViewCount() const { return 2U; }
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