#include "pch_bcl.h"
#include "SceneObject.h"
#include "CausalityApplication.h"
#include <PrimitiveVisualizer.h>
#include <ShadowMapGenerationEffect.h>
//#include "CharacterObject.h"

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;

bool g_DebugView = false;
bool g_ShowCharacterMesh = true;
float g_DebugArmatureThinkness = 0.005f;
bool g_MirrowInputX = false;


void XM_CALLCONV DrawBox(_In_reads_(8) Vector3 *conners, FXMVECTOR color)
{
	auto& drawer = Visualizers::g_PrimitiveDrawer;
	drawer.DrawLine(conners[0], conners[1], color);
	drawer.DrawLine(conners[1], conners[2], color);
	drawer.DrawLine(conners[2], conners[3], color);
	drawer.DrawLine(conners[3], conners[0], color);

	drawer.DrawLine(conners[0], conners[4], color);
	drawer.DrawLine(conners[1], conners[5], color);
	drawer.DrawLine(conners[2], conners[6], color);
	drawer.DrawLine(conners[3], conners[7], color);

	drawer.DrawLine(conners[4], conners[5], color);
	drawer.DrawLine(conners[5], conners[6], color);
	drawer.DrawLine(conners[6], conners[7], color);
	drawer.DrawLine(conners[7], conners[4], color);

}

void XM_CALLCONV DrawGeometryOutline(const BoundingGeometry& geometry, FXMVECTOR color)
{
	Vector3 conners[8];
	if (geometry.Type == BoundingGeometryType::Geometry_Frustum)
	{
		geometry.Frustum.GetCorners(conners);
		DrawBox(conners, color);
	}
	else if (geometry.Type == BoundingGeometryType::Geometry_OrientedBox)
	{
		geometry.OrientedBox.GetCorners(conners);
		DrawBox(conners, color);
	}
	else if (geometry.Type == BoundingGeometryType::Geometry_AxisAlignedBox)
	{
		geometry.AxisAlignedBox.GetCorners(conners);
		DrawBox(conners, color);
	}
	else if (geometry.Type == BoundingGeometryType::Geometry_Sphere)
	{
	}
}

SceneObject::~SceneObject()
{
	int *p = nullptr;
}

SceneObject::SceneObject() {
	m_IsEnabled = true;
	m_IsStatic = false;
	m_TransformDirty = false;
}

void SceneObject::AddChild(SceneObject * child)
{
	if (child != nullptr)
	{
		std::lock_guard<std::mutex> guard(Scene->ContentMutex());
		auto oldParent = child->parent();
		append_children_back(child);
		child->OnParentChanged(oldParent);
	}
}

void SceneObject::OnParentChanged(SceneObject* oldParent)
{
}

void SceneObject::Update(time_seconds const & time_delta) {
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

void XM_CALLCONV SceneObject::Move(FXMVECTOR p)
{
	SetPosition((XMVECTOR)GetPosition() + XMVector3Rotate(p, GetOrientation()));
}

void XM_CALLCONV SceneObject::Rotate(FXMVECTOR q)
{
	SetOrientation(XMQuaternionMultiply(q, GetOrientation()));
}

void SceneObject::SetTransformDirty()
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

Vector3 SceneObject::GetPosition() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblTranslation;
}

Quaternion SceneObject::GetOrientation() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblRotation;
}

Vector3 SceneObject::GetScale() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
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

void SceneObject::SetLocalTransform(const DirectX::IsometricTransform & lcl)
{
	m_Transform.LocalTransform() = lcl;
	SetTransformDirty();
}

const DirectX::IsometricTransform & SceneObject::GetGlobalTransform() const
{
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GlobalTransform();
}

void SceneObject::UpdateTransformsParentWard() const
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

void SceneObject::UpdateTransformsChildWard()
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

RenderFlags VisualObject::GetRenderFlags() const
{
	return RenderFlags::OpaqueObjects;
}

void VisualObject::Render(RenderContext & pContext, DirectX::IEffect* pEffect)
{
	if (g_ShowCharacterMesh && m_pRenderModel)
		m_pRenderModel->Render(pContext, GlobalTransformMatrix(), pEffect);
	if (g_ShowCharacterMesh && g_DebugView && m_pRenderModel)
	{
		BoundingGeometry geo(m_pRenderModel->GetBoundingBox());
		geo.Transform(geo, GlobalTransformMatrix());
		auto& drawer = Visualizers::g_PrimitiveDrawer;
		drawer.Begin();
		DrawGeometryOutline(geo, DirectX::Colors::Orange);
		drawer.End();
	}
}

void XM_CALLCONV VisualObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
}

VisualObject::VisualObject()
{
	m_isVisable = true;
	m_opticity = 1.0f;
	m_isFocuesd = false;
}

bool VisualObject::IsVisible(const BoundingGeometry & viewFrustum) const
{
	if (!m_isVisable || m_pRenderModel == nullptr) return false;
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

	if (e.Key == VK_OEM_4) // [{
	{
		g_DebugArmatureThinkness = std::max(0.001f,g_DebugArmatureThinkness - 0.002f);
	}
	if (e.Key == VK_OEM_6) // ]}
	{
		g_DebugArmatureThinkness = std::min(0.015f, g_DebugArmatureThinkness + 0.002f);
	}

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

bool CoordinateAxis::IsVisible(const BoundingGeometry & viewFrustum) const
{
	return g_DebugView;
}

void CoordinateAxis::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	float ub = 10, lb = -10, majorIdent = 1, minorIdent = 0.25f;
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;
	//g_PrimitiveDrawer.DrawSphere({ .0f,.0f,.0f,0.02f }, Colors::Cyan);

	float Ar = 0.03f, Al = ub, Almr = Al - Ar, Alpr = Al + Ar;
	g_PrimitiveDrawer.Begin();

	for (float x = lb; x <= ub; x += minorIdent)
	{
		g_PrimitiveDrawer.DrawLine({ -Al,.0f,x }, { Al,.0f,x }, Colors::DarkGray);
		g_PrimitiveDrawer.DrawLine({ x,.0f,-Al }, { x,.0f,Al }, Colors::DarkGray);
	}

	for (float x = lb; x <= ub; x += majorIdent)
	{
		g_PrimitiveDrawer.DrawLine({ -Al,.0f,x }, { Al,.0f,x }, Colors::DimGray);
		g_PrimitiveDrawer.DrawLine({ x,.0f,-Al }, { x,.0f,Al }, Colors::DimGray);
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

void XM_CALLCONV CoordinateAxis::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	g_PrimitiveDrawer.SetView(view);
	g_PrimitiveDrawer.SetProjection(projection);
	g_PrimitiveDrawer.SetWorld(GlobalTransformMatrix());
}

RenderFlags CoordinateAxis::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

GlowingBorder::GlowingBorder()
{
	m_Color = Colors::Red.v;
}

GlowingBorder::GlowingBorder(const DirectX::Color & color)
	: m_Color(color)
{

}

bool GlowingBorder::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	if (!IsEnabled()) return false;
	auto pVisual = this->FirstAncesterOfType<VisualObject>();
	if (pVisual)
	{
		return pVisual->IsVisible(viewFrustum);
	}
}

RenderFlags GlowingBorder::GetRenderFlags() const
{
	return RenderFlags::BloomEffectSource;
}

void GlowingBorder::Render(RenderContext & pContext, DirectX::IEffect * pEffect)
{
	auto pVisual = this->FirstAncesterOfType<VisualObject>();
	auto pModel = pVisual ? pVisual->RenderModel() : nullptr;
	auto pSGEffect = dynamic_cast<ShadowMapGenerationEffect*> (pEffect);
	if (pSGEffect)
	{
		if (pSGEffect->GetShadowFillMode() == ShadowMapGenerationEffect::SolidColorFill)
		{
			pSGEffect->SetShadowColor(m_Color);
		}
	}

	if (pModel)
	{
		pModel->Render(pContext,pVisual->GlobalTransformMatrix(), pEffect); // Render parent model with customized effect
	}
}

void XM_CALLCONV GlowingBorder::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
}

SkyDome::SkyDome()
{
}

SkyDome::~SkyDome()
{

}

void SkyDome::CreateDeviceResource(ID3D11Device * device, DirectX::EnvironmentMapEffect * pEffect)
{
	m_pSphere = DirectX::Scene::GeometricPrimtives::CreateSphere(device, 1.0f,16,true,true);
	m_pSphere->CreateInputLayout(device, pEffect);
	m_pEffect = pEffect;
}


void SkyDome::SetTexture(DirectX::Texture & texture)
{
	m_Texture = texture;
}

// Inherited via IRenderable

bool SkyDome::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	return g_ShowCharacterMesh;
}

void SkyDome::Render(RenderContext & context, DirectX::IEffect * pEffect)
{
	auto pStates = DirectX::Visualizers::g_PrimitiveDrawer.GetStates();
	m_pEffect->SetTexture(NULL);
	m_pEffect->SetWorld(XMMatrixIdentity());
	m_pEffect->SetEnvironmentMap(m_Texture.ShaderResourceView());
	m_pEffect->SetEnvironmentMapAmount(0.5f);
	m_pEffect->SetDiffuseColor(DirectX::Colors::White.v);
	m_pEffect->SetAmbientLightColor(DirectX::Colors::Azure.v);
	m_pEffect->SetFresnelFactor(0.0f);
	m_pEffect->Apply(context.Get());

	ComPtr<ID3D11DepthStencilState> pFomerState;
	UINT sRef;
	context->RSSetState(pStates->CullClockwise());
	context->OMGetDepthStencilState(&pFomerState, &sRef);
	context->OMSetDepthStencilState(pStates->DepthNone(), sRef);

	m_pSphere->Draw(context.Get(), m_pEffect);

	context->OMSetDepthStencilState(pFomerState.Get(), sRef);
	context->RSSetState(pStates->CullCounterClockwise());
}

void XM_CALLCONV SkyDome::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	XMMATRIX View = view;
	// Last column of View Inverse is camera's position
	View.r[3] = g_XMIdentityR3;
	m_pEffect->SetView(View);
	m_pEffect->SetProjection(projection);
}

// Inherited via IRenderable

RenderFlags SkyDome::GetRenderFlags() const
{
	return RenderFlags::SkyView;
}
