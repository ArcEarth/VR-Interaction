#include "pch_bcl.h"
#include "SceneObject.h"
#include "CausalityApplication.h"
#include <PrimitiveVisualizer.h>

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;

bool Causality::g_DebugView = false;
bool Causality::g_ShowCharacterMesh = true;

Causality::SceneObject::~SceneObject()
{
	int *p = nullptr;
}

Causality::SceneObject::SceneObject() {
	m_IsEnabled = true;
	m_IsStatic = false;
}

void Causality::SceneObject::Update(time_seconds const & time_delta) {}

DirectX::XMMATRIX SceneObject::GlobalTransformMatrix() const
{
	if (parent() != nullptr)
	{
		DirectX::XMMATRIX mat = parent()->GlobalTransformMatrix();
		mat *= TransformMatrix();
		return mat;
	}
	else
	{
		return TransformMatrix();
	}
}

void SceneObject::SetPosition(const Vector3 & p)
{
	m_GlobalTransformDirty = true;
	AffineTransform::Translation = p;
}

void SceneObject::SetOrientation(const Quaternion & q)
{
	m_GlobalTransformDirty = true;
	AffineTransform::Rotation = q;
}

void SceneObject::SetScale(const Vector3 & s)
{
	m_GlobalTransformDirty = true;
	AffineTransform::Scale = s;
}

DirectX::Scene::IModelNode * VisualObject::RenderModel(int LoD)
{
	return m_pRenderModel;
}

const DirectX::Scene::IModelNode * VisualObject::RenderModel(int LoD) const
{
	return m_pRenderModel;
}

void VisualObject::SetRenderModel(DirectX::Scene::IModelNode * pMesh, int LoD)
{
	m_pRenderModel = pMesh;
}

RenderFlags Causality::VisualObject::GetRenderFlags() const
{
	return RenderFlags::OpaqueObjects;
}

void VisualObject::Render(RenderContext & pContext, DirectX::IEffect* pEffect)
{
	if (m_pRenderModel)
		m_pRenderModel->Render(pContext, GlobalTransformMatrix(), pEffect);
}

void XM_CALLCONV VisualObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
}

Causality::VisualObject::VisualObject()
{
	m_isVisable = true;
	m_opticity = 1.0f;
	m_isFocuesd = false;
}

bool VisualObject::IsVisible(const BoundingFrustum & viewFrustum) const
{
	if (!m_isVisable) return false;
	auto box = m_pRenderModel->GetOrientedBoundingBox();
	box.Transform(box, this->GlobalTransformMatrix());
	return viewFrustum.Contains(box) != DirectX::ContainmentType::DISJOINT;
}

KeyboardMouseFirstPersonControl::KeyboardMouseFirstPersonControl(IRigid* pTarget)
{
	Speed = 2.0f;
	SetTarget(pTarget);
}

void KeyboardMouseFirstPersonControl::SetTarget(IRigid * pTarget)
{
	if (!pTarget && m_pTarget)
	{
		Unregister();
	}
	else if (pTarget)
	{
		bool needToReg = !m_pTarget;
		m_pTarget = pTarget;
		InitialOrientation = pTarget->GetOrientation();
		AddationalYaw = 0;
		AddationalPitch = 0;
		AddationalRoll = 0;
		if (needToReg)
			Register();
	}

}

void KeyboardMouseFirstPersonControl::Update(time_seconds const& time_delta)
{
	using namespace DirectX;
	if (m_pTarget)
	{
		XMVECTOR disp = XMVector3Normalize(CameraVeclocity);
		disp = XMVector3Rotate(disp, m_pTarget->GetOrientation());
		disp *= Speed * (float)time_delta.count();
		m_pTarget->Move(disp);
	}
}

void KeyboardMouseFirstPersonControl::OnKeyDown(const KeyboardEventArgs & e)
{
	if (!m_pTarget) return;

	if (e.Key == 'W')
		CameraVeclocity += Vector3{ 0,0,-1 };
	if (e.Key == 'S')
		CameraVeclocity += Vector3{ 0,0,1 };
	if (e.Key == 'A')
		CameraVeclocity += Vector3{ -1,0,0 };
	if (e.Key == 'D')
		CameraVeclocity += Vector3{ 1,0,0 };
}

void KeyboardMouseFirstPersonControl::OnKeyUp(const KeyboardEventArgs & e)
{
	if (!m_pTarget) return;
	if (e.Key == 'W')
		CameraVeclocity -= Vector3{ 0,0,-1 };
	if (e.Key == 'S')
		CameraVeclocity -= Vector3{ 0,0,1 };
	if (e.Key == 'A')
		CameraVeclocity -= Vector3{ -1,0,0 };
	if (e.Key == 'D')
		CameraVeclocity -= Vector3{ 1,0,0 };
	if (e.Key == 'K')
		g_DebugView = !g_DebugView;
	if (e.Key == 'T')
		g_ShowCharacterMesh = !g_ShowCharacterMesh;

	if (e.Key == VK_SPACE)
	{
		if (!this->Scene->IsPaused())
			this->Scene->Pause();
		else
			this->Scene->Resume();
	}

	if (e.Key == VK_ESCAPE)
		App::Current()->Exit();
}

void KeyboardMouseFirstPersonControl::OnMouseButtonDown(const CursorButtonEvent & e)
{
	if (e.Button == CursorButtonEnum::RButton)
	{
		IsTrackingCursor = true;
		//CursorMoveEventConnection = pWindow->CursorMove += MakeEventHandler(&App::OnCursorMove_RotateCamera, this);
	}
}

void KeyboardMouseFirstPersonControl::OnMouseButtonUp(const CursorButtonEvent & e)
{
	if (e.Button == CursorButtonEnum::RButton)
	{
		IsTrackingCursor = false;
		//CursorMoveEventConnection.disconnect();
		//pWindow->CursorMove -= CursorMoveEventConnection;
	}
}

void KeyboardMouseFirstPersonControl::OnMouseMove(const CursorMoveEventArgs & e)
{
	using namespace DirectX;
	if (!IsTrackingCursor) return;
	auto yaw = -e.PositionDelta.x / 1000.0f * XM_PI;
	auto pitch = -e.PositionDelta.y / 1000.0f * XM_PI;
	AddationalYaw += yaw;
	AddationalPitch += pitch;
	if (m_pTarget)
	{
		XMVECTOR extrinsic = XMQuaternionRotationRollPitchYaw(AddationalPitch, AddationalYaw, 0);
		XMVECTOR intial = InitialOrientation;
		intial = XMQuaternionMultiply(extrinsic, intial);
		m_pTarget->SetOrientation(intial);
	}
}

bool Causality::CoordinateAxis::IsVisible(const BoundingFrustum & viewFrustum) const
{
	return true;
}

void Causality::CoordinateAxis::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	float ub = 5, lb = -5, majorIdent = 1, minorIdent = 0.25f;
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;
	g_PrimitiveDrawer.DrawSphere({ .0f,.0f,.0f,0.02f }, Colors::Cyan);

	float Ar = 0.03f, Al = ub, Almr = Al - Ar, Alpr = Al + Ar;
	g_PrimitiveDrawer.Begin();

	for (float x = lb; x <= ub; x += minorIdent)
	{
		g_PrimitiveDrawer.DrawLine({ -Al,.0f,x }, { Al,.0f,x }, Colors::LightGray);
		g_PrimitiveDrawer.DrawLine({ x,.0f,-Al }, { x,.0f,Al }, Colors::LightGray);
	}

	for (float x = lb; x <= ub; x += majorIdent)
	{
		g_PrimitiveDrawer.DrawLine({ -Al,.0f,x }, { Al,.0f,x }, Colors::Black);
		g_PrimitiveDrawer.DrawLine({ x,.0f,-Al }, { x,.0f,Al }, Colors::Black);
	}

	g_PrimitiveDrawer.DrawLine({ -Al,.0f,.0f }, { Al,.0f,.0f }, Colors::Red);
	g_PrimitiveDrawer.DrawLine({ .0f,-Al,.0f }, { .0f,Al,.0f }, Colors::Green);
	g_PrimitiveDrawer.DrawLine({ .0f,.0f,-Al }, { .0f,.0f,Al }, Colors::Blue);


	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,Ar,.0f }, { Almr,-Ar,.0f }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,-Ar,.0f }, { Almr,Ar,.0f }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,.0f,Ar }, { Almr,.0f,-Ar }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,.0f,-Ar }, { Almr,.0f,Ar }, Colors::Red);

	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { -Ar,Almr,.0f }, { Ar,Almr,.0f }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { Ar,Almr,.0f }, { -Ar,Almr,.0f }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { .0f,Almr,-Ar }, { .0f,Almr,Ar }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { .0f,Almr,Ar }, { .0f,Almr,-Ar }, Colors::Green);

	g_PrimitiveDrawer.DrawTriangle({ .0f,.0f,Alpr }, { Ar,.0f,Almr }, { -Ar,.0f,Almr }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ .0f,.0f,Alpr }, { -Ar,.0f,Almr }, { Ar,.0f,Almr }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ .0f,.0f,Alpr }, { .0f,Ar,Almr }, { .0f,-Ar,Almr }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ .0f,.0f,Alpr }, { .0f,-Ar,Almr }, { .0f,Ar,Almr }, Colors::Blue);
	g_PrimitiveDrawer.End();
}

void XM_CALLCONV Causality::CoordinateAxis::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	g_PrimitiveDrawer.SetView(view);
	g_PrimitiveDrawer.SetProjection(projection);
	g_PrimitiveDrawer.SetWorld(GlobalTransformMatrix());
}

RenderFlags Causality::CoordinateAxis::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}
