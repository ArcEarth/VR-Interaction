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
	m_TransformDirty = false;
}

void Causality::SceneObject::Update(time_seconds const & time_delta) {
	UpdateTransformsChildWard();
}

DirectX::XMMATRIX SceneObject::GlobalTransformMatrix() const
{
	return this->m_Transform.GlobalTransform().TransformMatrix();
	//if (parent() != nullptr)
	//{
	//	DirectX::XMMATRIX mat = parent()->GetPosition();
	//	mat *= TransformMatrix();
	//	return mat;
	//}
	//else
	//{
	//	return TransformMatrix();
	//}
}

void XM_CALLCONV Causality::SceneObject::Move(FXMVECTOR p)
{
	SetPosition((XMVECTOR)GetPosition() + XMVector3Rotate(p, GetOrientation()));
}

void XM_CALLCONV Causality::SceneObject::Rotate(FXMVECTOR q)
{
	SetOrientation(XMQuaternionMultiply(q, GetOrientation()));
}

void Causality::SceneObject::SetTransformDirty()
{
	if (!m_TransformDirty)
	{
		m_TransformDirty = true;
		for (auto& child : children())
		{
			child.SetTransformDirty();
		}
	}
}

Vector3 Causality::SceneObject::GetPosition() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblTranslation;
}

Quaternion Causality::SceneObject::GetOrientation() const {
	return m_Transform.GblRotation;
}

Vector3 Causality::SceneObject::GetScale() const {
	return m_Transform.GblScaling;
}

void SceneObject::SetPosition(const Vector3 & p)
{
	if (!parent())
	{
		m_Transform.LclTranslation = m_Transform.GblTranslation = p;
		m_TransformDirty = false;
	}
	else
	{
		XMVECTOR refQ = parent()->GetOrientation(); // parent global orientation
		XMVECTOR V = parent()->GetPosition();
		V = p.Load() - V;
		refQ = XMQuaternionConjugate(refQ);
		V = XMVector3Rotate(V, refQ); // V *= Inverse(ref.Orientation)
		m_Transform.LclTranslation = V;
	}
	SetTransformDirty();
}

void SceneObject::SetOrientation(const Quaternion & q)
{
	if (!parent())
	{
		m_Transform.LclRotation = m_Transform.GblRotation = q;
		m_TransformDirty = false;
	}
	else
	{
		XMVECTOR refQ = parent()->GetOrientation(); // parent global orientation
		refQ = XMQuaternionConjugate(refQ);
		m_Transform.LclRotation = XMQuaternionMultiply(q, refQ);
	}
	SetTransformDirty();
}

void SceneObject::SetScale(const Vector3 & s)
{
	if (!parent())
	{
		m_Transform.LclScaling = m_Transform.GblScaling = s;
	}
	else
	{
		XMVECTOR refS = parent()->GetScale();
		m_Transform.LclScaling = s.Load() / refS;
	}
	SetTransformDirty();
}

void Causality::SceneObject::SetLocalTransform(const DirectX::ScaledRigidTransform & lcl)
{
	m_Transform.LocalTransform() = lcl;
	SetTransformDirty();
}

const DirectX::ScaledRigidTransform & Causality::SceneObject::GetGlobalTransform() const
{
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GlobalTransform();
}

void Causality::SceneObject::UpdateTransformsParentWard() const
{
	if (!m_TransformDirty) return;
	const SceneObject *pObj = parent();

	if (pObj)
	{
		if (pObj->m_TransformDirty)
			pObj->UpdateTransformsParentWard();
		m_Transform.UpdateGlobalTransform(pObj->m_Transform);
	}
	else
		m_Transform.GlobalTransform() = m_Transform.LocalTransform();
	m_TransformDirty = false;
}

void Causality::SceneObject::UpdateTransformsChildWard()
{
	if (m_TransformDirty)
	{
		UpdateTransformsParentWard();
		for (auto& child : children())
		{
			child.UpdateTransformsChildWard();
		}
	}
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

bool VisualObject::IsVisible(const BoundingGeometry & viewFrustum) const
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
	if (m_pTarget && CameraVeclocity.LengthSquared() > 0.01f)
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

	if (e.Key == '-' || e.Key == '_' || e.Key == VK_SUBTRACT || e.Key == VK_OEM_MINUS)
		this->Scene->SetTimeScale(this->Scene->GetTimeScale() - 0.1);

	if (e.Key == '=' || e.Key == '+' || e.Key == VK_ADD || e.Key == VK_OEM_PLUS)
		this->Scene->SetTimeScale(this->Scene->GetTimeScale() + 0.1);

	if (e.Key == '0' || e.Key == ')')
		this->Scene->SetTimeScale(1.0);

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
		intial = XMQuaternionMultiply(intial, extrinsic);
		m_pTarget->SetOrientation(intial);
	}
}

bool Causality::CoordinateAxis::IsVisible(const BoundingGeometry & viewFrustum) const
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
	g_PrimitiveDrawer.DrawLine({ .0f,-Al,.0f }, { .0f,Al,.0f }, Colors::Lime);
	g_PrimitiveDrawer.DrawLine({ .0f,.0f,-Al }, { .0f,.0f,Al }, Colors::Blue);


	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,Ar,.0f }, { Almr,-Ar,.0f }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,-Ar,.0f }, { Almr,Ar,.0f }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,.0f,Ar }, { Almr,.0f,-Ar }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ Alpr,.0f,.0f }, { Almr,.0f,-Ar }, { Almr,.0f,Ar }, Colors::Red);

	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { -Ar,Almr,.0f }, { Ar,Almr,.0f }, Colors::Lime);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { Ar,Almr,.0f }, { -Ar,Almr,.0f }, Colors::Lime);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { .0f,Almr,-Ar }, { .0f,Almr,Ar }, Colors::Lime);
	g_PrimitiveDrawer.DrawTriangle({ .0f,Alpr,.0f }, { .0f,Almr,Ar }, { .0f,Almr,-Ar }, Colors::Lime);

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

Causality::GlowingBorder::GlowingBorder()
{
	m_Color = Colors::Red.v;
}

Causality::GlowingBorder::GlowingBorder(const DirectX::Color & color)
	: m_Color(color)
{

}

bool Causality::GlowingBorder::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	if (!IsEnabled()) return false;
	auto pVisual = dynamic_cast<const VisualObject*>(parent());
	if (pVisual)
	{
		return pVisual->IsVisible(viewFrustum);
	}
}

RenderFlags Causality::GlowingBorder::GetRenderFlags() const
{
	return RenderFlags::BloomEffectSource;
}

void Causality::GlowingBorder::Render(RenderContext & pContext, DirectX::IEffect * pEffect)
{
	auto pVisual = dynamic_cast<VisualObject*>(parent());
	if (pVisual)
	{
		auto pModel = pVisual->RenderModel();
		pModel->Render(pContext,pVisual->GlobalTransformMatrix(), pEffect); // Render parent model with customized effect
	}
}

void XM_CALLCONV Causality::GlowingBorder::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
}
