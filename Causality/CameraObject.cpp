#include "pch_bcl.h"
#include "CameraObject.h"
#include <DirectXColors.h>
using namespace DirectX;
using namespace Causality;

const XMVECTORF32 Camera::Foward = g_XMNegIdentityR2, Camera::Up = g_XMIdentityR1;

IEffect * IRenderControl::GetRenderEffect() { return nullptr; }

size_t Camera::ViewCount() const
{
	return IsStereoEnabled() ? 2U : 1U;
}

XMMATRIX Camera::GetViewMatrix(size_t view) const
{
	assert(view < ViewCount());
	if (IsStereoEnabled())
	{
		auto eye = (EyesEnum)view;
		auto pose = m_pRift->EyePoses(eye);
		XMVECTOR loc = (XMVECTOR)GetPosition() + XMVector3Rotate((XMVECTOR)pose.Position, pose.Orientation);
		XMVECTOR rot = pose.Orientation * GetOrientation();
		XMVECTOR foward = XMVector3Rotate(Foward, rot);
		XMVECTOR up = XMVector3Rotate(Up, rot);
		return XMMatrixLookToRH(loc, foward, up);
	}
	else
	{
		XMVECTOR loc = (XMVECTOR)GetPosition();
		XMVECTOR rot = GetOrientation();
		XMVECTOR foward = XMVector3Rotate(Foward, rot);
		XMVECTOR up = XMVector3Rotate(Up, rot);
		return XMMatrixLookToRH(loc, foward, up);
	}
}

XMMATRIX Camera::GetProjectionMatrix(size_t view) const
{
	assert(view < ViewCount());
	if (IsStereoEnabled())
	{
		auto eye = (EyesEnum)view;
		return m_pRift->EyeProjection(eye);
	}
	else
	{
		return UpdateProjectionCache();
	}
}

XMMATRIX Camera::UpdateProjectionCache() const
{
	if (m_ProjectDirty)
	{
		using namespace DirectX;
		m_ProjectDirty = false;
		XMMATRIX proj;
		if (m_IsPerspective)
		{
			proj = XMMatrixPerspectiveFovRH(m_Fov, m_AspectRatio, m_Near, m_Far);
			m_ViewFrutum.Type = BoundingGeometryType::Geometry_Frustum;
			BoundingFrustumExtension::CreateFromMatrixRH(m_ViewFrutum.Frustum, proj);
		}
		else
		{
			proj = XMMatrixOrthographicRH(m_Fov * m_AspectRatio, m_Fov, m_Near, m_Far);
			m_ViewFrutum.Type = BoundingGeometryType::Geometry_OrientedBox;
			m_ViewFrutum.OrientedBox.Center = { 0, 0, 0.5f * (m_Near + m_Far) };
			m_ViewFrutum.OrientedBox.Extents = { 0.5f * m_Fov * m_AspectRatio, 0.5f* m_Fov, 0.5f * abs(m_Near - m_Far) };
			m_ViewFrutum.OrientedBox.Orientation = Quaternion::Identity;
		}

		XMStoreFloat4x4(&m_ProjectionCache, proj);
		return proj;
	}
	else
		return m_ProjectionCache;
}

bool Camera::AcceptRenderFlags(RenderFlags flags)
{
	return flags.Contains(Visible);
}

const BoundingGeometry & Camera::GetViewFrustum(size_t view) const {
	if (m_ProjectDirty)
	{
		UpdateProjectionCache();
	}
	XMMATRIX transform ;

	if (m_ViewFrutum.Type == Geometry_OrientedBox || m_ViewFrutum.Type == Geometry_AxisAlignedBox)
	{
		// make sure the transform rotation origin is 0,0,0 instead of bounding geometry's center
		transform = XMMatrixTranslation(0, 0, -0.5f * (m_Near + m_Far)) * GlobalTransformMatrix();
	}
	else
	{
		transform = GlobalTransformMatrix();
	}

	m_ViewFrutum.Transform(m_ExtrinsicViewFrutum, transform);
	return m_ExtrinsicViewFrutum;
}

bool XM_CALLCONV Camera::IsInView(FXMVECTOR pos, size_t view) const
{
	auto& frustum = GetViewFrustum(view);
	return frustum.Contains(pos) != ContainmentType::DISJOINT;
}

// This ignore the effect of Oculus

void Camera::FocusAt(FXMVECTOR focusPoint, FXMVECTOR upDir)
{
	using namespace DirectX;
	// Right-Hand Coordinate
	XMVECTOR foward = -XMVector3Normalize(focusPoint - (XMVECTOR)GetPosition()); // Foward is (0,0,-1), Right is (1,0,0) Up is (0,1,0)
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
	SetPosition((XMVECTOR)GetPosition() + XMVector3Rotate(p, GetOrientation()));
}

void XM_CALLCONV Camera::Rotate(FXMVECTOR q)
{
	SetOrientation(XMQuaternionMultiply(q, GetOrientation()));
}

Camera::Camera()
{
	m_ProjectDirty = true;
	m_ViewDirty = true;
}

Camera::~Camera()
{
}

RenderTarget & Camera::GetRenderTarget(int view)
{
	return m_RenderTarget;
}

void Camera::SetRenderTarget(RenderTarget & renderTarget, int view, bool autoAspect)
{
	m_RenderTarget = renderTarget;
	if (autoAspect)
	{
		m_ProjectDirty = true;
		m_AspectRatio = renderTarget.ViewPort().Width / renderTarget.ViewPort().Height;
	}
}

void Camera::SetRenderTarget(RenderTarget && renderTarget, int view, bool autoAspect)
{
	m_RenderTarget = std::move(renderTarget);
	if (autoAspect)
	{
		m_ProjectDirty = true;
		m_AspectRatio = renderTarget.ViewPort().Width / renderTarget.ViewPort().Height;
	}
}

void Camera::SetRenderContext(const RenderContext & context)
{
	m_pRenderContext = context;
}

// Stereo Settings

void Camera::EnableStereo(const std::shared_ptr<Devices::OculusRift>& pRift)
{
	m_pRift = pRift;
}

void Camera::DisableStoreo()
{
	m_pRift = nullptr;
}

bool Camera::IsStereoEnabled() const
{
	return m_pRift != nullptr;
}

void Camera::BeginFrame(RenderContext& context)
{
	m_pRenderContext = context;
	if (IsStereoEnabled())
	{
		m_pRift->BeginFrame();
	}
	else
	{
		auto context = m_pRenderContext.Get();
		m_RenderTarget.Clear(context, m_Background);
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
	m_pRenderContext = nullptr;
}

void Camera::SetView(size_t view)
{
	if (IsStereoEnabled())
	{
		m_pRift->ViewTarget((EyesEnum)view).SetAsRenderTarget(m_pRenderContext);
	}
	else
	{
		m_pRenderContext->RSSetViewports(1, &m_RenderTarget.ViewPort());
	}
}

//XMMATRIX Player::GetViewMatrix() const
//{
//	using namespace DirectX;
//	XMVECTOR loc = (XMVECTOR) Position();
//	XMVECTOR rot = Orientation();
//	XMVECTOR foward = XMVector3Rotate(Foward, rot);
//	XMVECTOR up = XMVector3Rotate(Up, rot);
//	return XMMatrixLookToRH(loc, foward, up);
//}
//
//XMMATRIX Player::GetProjectionMatrix() const
//{
//	using namespace DirectX;
//	return XMMatrixPerspectiveFovRH(Fov, AspectRatio, 0.01f,100.0f);
//}

void Camera::SetPerspective(float fovRadius, float aspectRatioHbyW, float _near, float _far)
{
	m_IsPerspective = true;
	m_ProjectDirty = true;
	m_Fov = fovRadius; m_AspectRatio = aspectRatioHbyW; m_Near = _near; m_Far = _far;
}

void Camera::SetOrthographic(float viewWidth, float viewHeight, float _near, float _far)
{
	m_IsPerspective = false;
	m_ProjectDirty = true;
	m_Fov = viewHeight; m_AspectRatio = viewWidth / viewHeight;
	m_Near = _near; m_Far = _far;
}

bool Camera::IsPerspective() const { return m_IsPerspective; }

float Camera::GetFov() const
{
	return m_Fov;
}

float Camera::GetAspectRatio() const { return m_AspectRatio; }

float Camera::GetNear() const { return m_Near; }

float Camera::GetFar() const { return m_Far; }
