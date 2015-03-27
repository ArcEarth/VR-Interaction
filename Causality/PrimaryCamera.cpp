#include "pch_bcl.h"
#include "PrimaryCamera.h"
#include <DirectXColors.h>
using namespace DirectX;
using namespace Causality;

const DirectX::XMVECTORF32 Camera::Foward = DirectX::g_XMNegIdentityR2, Camera::Up = DirectX::g_XMIdentityR1;


size_t Camera::ViewCount() const
{
	return IsStereoEnabled() ? 2U : 1U;
}

DirectX::XMMATRIX Camera::GetViewMatrix(size_t view) const
{
	assert(view < ViewCount());
	if (IsStereoEnabled())
	{
		auto eye = (EyesEnum) view;
		auto pose = m_pRift->EyePoses(eye);
		XMVECTOR loc = (XMVECTOR) GetPosition() + DirectX::XMVector3Rotate((XMVECTOR) pose.Position, pose.Orientation);
		XMVECTOR rot = pose.Orientation * GetOrientation();
		XMVECTOR foward = XMVector3Rotate(Foward, rot);
		XMVECTOR up = XMVector3Rotate(Up, rot);
		return XMMatrixLookToRH(loc, foward, up);
	}
	else
	{
		XMVECTOR loc = (XMVECTOR) GetPosition();
		XMVECTOR rot = GetOrientation();
		XMVECTOR foward = XMVector3Rotate(Foward, rot);
		XMVECTOR up = XMVector3Rotate(Up, rot);
		return XMMatrixLookToRH(loc, foward, up);
	}
}

DirectX::XMMATRIX Camera::GetProjectionMatrix(size_t view) const
{
	assert(view < ViewCount());
	if (IsStereoEnabled())
	{
		auto eye = (EyesEnum) view;
		return m_pRift->EyeProjection(eye);
	}
	else
	{
		return UpdateProjectionCache();
	}
}

DirectX::XMMATRIX Causality::Camera::UpdateProjectionCache() const
{
	if (m_ProjectDirty)
	{
		using namespace DirectX;
		m_ProjectDirty = false;
		auto proj = XMMatrixPerspectiveFovRH(Fov, AspectRatio, 0.01f, 100.0f);
		XMStoreFloat4x4(&m_ProjectionCache, proj);
		DirectX::BoundingFrustumExtension::CreateFromMatrixRH(m_ViewFrutum, proj);
		return proj;
	}
	else
		return m_ProjectionCache;
}

const BoundingFrustum & Causality::Camera::GetViewFrustum(size_t view) const { 
	if (m_ProjectDirty)
	{
		UpdateProjectionCache();
	}

	m_ViewFrutum.Transform(m_ExtrinsicViewFrutum, TransformMatrix());
	return m_ExtrinsicViewFrutum;
}

bool XM_CALLCONV Causality::Camera::IsInView(DirectX::FXMVECTOR pos, size_t view) const
{
	auto& frustum = GetViewFrustum(view);
	return frustum.Contains(pos) != DirectX::ContainmentType::DISJOINT;
}

// This ignore the effect of Oculus

void Camera::FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir)
{
	using namespace DirectX;
	// Right-Hand Coordinate
	XMVECTOR foward = -XMVector3Normalize(focusPoint - (XMVECTOR) GetPosition()); // Foward is (0,0,-1), Right is (1,0,0) Up is (0,1,0)
	XMVECTOR right = XMVector3Normalize(XMVector3Cross(upDir, foward)); // Y x Z
	XMVECTOR up = XMVector3Normalize(XMVector3Cross(foward, right)); // Z x X
	XMMATRIX m;
	m.r[0] = right;
	m.r[1] = up;
	m.r[2] = foward;
	m.r[3] = g_XMIdentityR3;
	XMVECTOR scale, quat, trans;
	XMMatrixDecompose(&scale, &quat, &trans, m);
	SetOrientation(quat);
}

void XM_CALLCONV Camera::Move(FXMVECTOR p)
{
	SetPosition((XMVECTOR) GetPosition() + XMVector3Rotate(p, GetOrientation()));
}

void XM_CALLCONV Camera::Rotate(FXMVECTOR q)
{
	SetOrientation(XMQuaternionMultiply(q, GetOrientation()));
}

//Vector3 Camera::GetPosition() const
//{
//	return SceneObject::GetPosition();
//}
//
//void Camera::SetPosition(const Vector3 &p)
//{
//	SceneObject::SetPosition(p);
//}
//
//Quaternion Camera::GetOrientation() const
//{
//	return SceneObject::GetOrientation();
//}
//
//void Camera::SetOrientation(const Quaternion &q)
//{
//	SceneObject::SetOrientation(q);
//}
//
//void XM_CALLCONV PlayerCamera::Move(FXMVECTOR p) 
//{
//
//}
//
//void XM_CALLCONV PlayerCamera::Rotate(FXMVECTOR q)
//{
//
//}


Causality::Camera::Camera(const RenderContext & context)
	: m_pRenderContext(context)
{
	m_ProjectDirty = true;
	m_ViewDirty = true;
}

Causality::Camera::~Camera()
{
}

DirectX::RenderTarget & Causality::Camera::GetRenderTarget(int view)
{
	return m_RenderTarget;
}

void Causality::Camera::SetRenderTarget(DirectX::RenderTarget & renderTarget, int view, bool autoAspect )
{
	m_RenderTarget = renderTarget;
	if (autoAspect)
		AspectRatio = renderTarget.ViewPort().Width / renderTarget.ViewPort().Height;
}

void Causality::Camera::SetRenderTarget(DirectX::RenderTarget && renderTarget, int view, bool autoAspect )
{
	m_RenderTarget = std::move(renderTarget);
	if (autoAspect)
		AspectRatio = renderTarget.ViewPort().Width / renderTarget.ViewPort().Height;
}

void Causality::Camera::SetRenderContext(const RenderContext & context)
{
	m_pRenderContext = context;
}

// Stereo Settings

void Causality::Camera::EnableStereo(const std::shared_ptr<Devices::OculusRift>& pRift)
{
	m_pRift = pRift;
}

void Causality::Camera::DisableStoreo()
{
	m_pRift = nullptr;
}

bool Causality::Camera::IsStereoEnabled() const
{
	return m_pRift != nullptr;
}

void Camera::BeginFrame()
{
	if (IsStereoEnabled())
	{
		m_pRift->BeginFrame();
	}
	else
	{
		auto context = m_pRenderContext.Get();
		// Reset the viewport to target the whole screen.
		//auto viewport = m_pDeviceResources->GetScreenViewport();
		//context->RSSetViewports(1, &viewport);
		m_RenderTarget.Clear(context, Background);
		m_RenderTarget.SetAsRenderTarget(context);
	}
}

void Camera::EndFrame()
{
	if (IsStereoEnabled())
	{
		m_pRift->EndFrame();
	}
	else
	{
	}
}

void Camera::SetView(size_t view)
{
	if (IsStereoEnabled())
	{
		m_pRift->ViewTarget((EyesEnum)view).SetAsRenderTarget(m_pRenderContext);
		//m_pRift->EyeTexture((DirectX::Scene::EyesEnum) view).SetAsRenderTarget(m_pDeviceResources->GetD3DDeviceContext(), m_pRift->DepthStencilBuffer());
	}
	else
	{
		//auto viewport = m_pDeviceResources->GetScreenViewport();
		//viewport.Width /= 2;
		//if (view == 1)
		//{
		//	viewport.TopLeftX = viewport.Width;
		//}
		m_pRenderContext->RSSetViewports(1, &m_RenderTarget.ViewPort());
	}
}

//DirectX::XMMATRIX Player::GetViewMatrix() const
//{
//	using namespace DirectX;
//	XMVECTOR loc = (XMVECTOR) Position();
//	XMVECTOR rot = Orientation();
//	XMVECTOR foward = XMVector3Rotate(Foward, rot);
//	XMVECTOR up = XMVector3Rotate(Up, rot);
//	return XMMatrixLookToRH(loc, foward, up);
//}
//
//DirectX::XMMATRIX Player::GetProjectionMatrix() const
//{
//	using namespace DirectX;
//	return XMMatrixPerspectiveFovRH(Fov, AspectRatio, 0.01f,100.0f);
//}

void Camera::SetPerspective(float fovRadius, float aspectRatioHbyW)
{
	Fov = fovRadius; AspectRatio = aspectRatioHbyW;
	m_ProjectDirty = true;
}

void Causality::Camera::SetOrthographic(float viewWidth, float viewHeight)
{
	m_ProjectDirty = true;
}

float Camera::GetFov() const
{
	return Fov;
}
