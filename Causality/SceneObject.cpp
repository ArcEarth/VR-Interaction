#include "pch_bcl.h"
#include "SceneObject.h"
#include "CausalityApplication.h"
#include "Common\PrimitiveVisualizer.h"

using namespace Causality;

bool Causality::g_DebugView = true;

KinematicSceneObject::frame_type & KinematicSceneObject::MapCurrentFrameForUpdate()
{
	return m_CurrentFrame;
	m_DirtyFlag = true;
}

void KinematicSceneObject::ReleaseCurrentFrameFrorUpdate()
{
	m_DirtyFlag = true;
}

IArmature & KinematicSceneObject::Armature() { return m_pAnimationSpace->Armature(); }

const IArmature & KinematicSceneObject::Armature() const { return m_pAnimationSpace->Armature(); }

AnimationSpace & KinematicSceneObject::Behavier() { return *m_pAnimationSpace; }

const AnimationSpace & KinematicSceneObject::Behavier() const { return *m_pAnimationSpace; }

void KinematicSceneObject::SetBehavier(AnimationSpace & behaver) { m_pAnimationSpace = &behaver; }

void Causality::KinematicSceneObject::Render(RenderContext & pContext)
{
	if (g_DebugView)
	{
		using namespace DirectX;
		using Visualizers::g_PrimitiveDrawer;
		const auto& frame = m_CurrentFrame;
		//const auto& dframe = Armature().default_frame();
		for (auto& bone : frame)
		{
			g_PrimitiveDrawer.DrawCylinder(bone.OriginPosition, bone.EndPostion, 0.01f, DirectX::Colors::LimeGreen);
		}

	}
	RenderableSceneObject::Render(pContext);
}

void XM_CALLCONV Causality::KinematicSceneObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	if (g_DebugView)
	{
		using namespace DirectX;
		using Visualizers::g_PrimitiveDrawer;
		g_PrimitiveDrawer.SetView(view);
		g_PrimitiveDrawer.SetProjection(projection);
	}
	RenderableSceneObject::UpdateViewMatrix(view, projection);
}

Causality::SceneObject::~SceneObject()
{
	int *p = nullptr;
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

DirectX::Scene::IModelNode * RenderableSceneObject::RenderModel(int LoD)
{
	return m_pRenderModel;
}

const DirectX::Scene::IModelNode * RenderableSceneObject::RenderModel(int LoD) const
{
	return m_pRenderModel;
}

void RenderableSceneObject::SetRenderModel(DirectX::Scene::IModelNode * pMesh, int LoD)
{
	m_pRenderModel = pMesh;
}

void RenderableSceneObject::Render(RenderContext & pContext)
{
	//m_pRenderModel->SetModelMatrix(this->GlobalTransformMatrix());
	m_pRenderModel->Render(pContext,GlobalTransformMatrix());
}

void XM_CALLCONV RenderableSceneObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	return;
}

bool RenderableSceneObject::IsVisible(const BoundingFrustum & viewFrustum) const
{
	if (!m_isVisable) return false;
	auto box = m_pRenderModel->GetOrientedBoundingBox();
	box.Transform(box,this->GlobalTransformMatrix());
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
	} else if (pTarget)
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
		disp *= Speed * (float) time_delta.count();
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
	auto yaw = e.PositionDelta.x / 1000.0f * XM_PI;
	auto pitch = e.PositionDelta.y / 1000.0f * XM_PI;
	AddationalYaw += yaw;
	AddationalPitch += pitch;
	if (m_pTarget)
	{
		XMVECTOR extrinsic = XMQuaternionRotationRollPitchYawRH(AddationalPitch, AddationalYaw, 0);
		XMVECTOR intial = InitialOrientation;
		intial = XMQuaternionMultiply(extrinsic,intial);
		m_pTarget->SetOrientation(intial);
	}
}

bool Causality::CoordinateAxis::IsVisible(const BoundingFrustum & viewFrustum) const
{
	return true;
}

void Causality::CoordinateAxis::Render(RenderContext & context)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;
	g_PrimitiveDrawer.Begin();
	g_PrimitiveDrawer.DrawSphere({ 0,0,0,0.02 }, Colors::Red);
	g_PrimitiveDrawer.DrawLine({ -5,0,0 }, { 5,0,0 }, Colors::Red);
	g_PrimitiveDrawer.DrawLine({ 0,-5,0 }, { 0,5,0 }, Colors::Green);
	g_PrimitiveDrawer.DrawLine({ 0,0,-5 }, { 0,0,5 }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ 5.05f,0,0 }, { 4.95,0.05,0 }, { 4.95,-0.05,0 }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ 5.05f,0,0 }, { 4.95,-0.05,0 }, { 4.95,0.05,0 }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ 5.05f,0,0 }, { 4.95,0,0.05 }, { 4.95,0,-0.05 }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ 5.05f,0,0 }, { 4.95,0,-0.05 }, { 4.95,0,0.05 }, Colors::Red);
	g_PrimitiveDrawer.DrawTriangle({ 0,5.05f,0 }, { -0.05,4.95,0 }, { 0.05,4.95,0 }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ 0,5.05f,0 }, { 0.05,4.95,0 }, { -0.05,4.95,0 }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ 0,5.05f,0 }, { 0.0,4.95,-0.05 }, { 0,4.95,0.05 }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ 0,5.05f,0 }, { 0.0,4.95,0.05 }, { 0,4.95,-0.05 }, Colors::Green);
	g_PrimitiveDrawer.DrawTriangle({ 0,0,5.05f }, { 0.05,0,4.95 }, { -0.05,0,4.95 }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ 0,0,5.05f }, { -0.05,0,4.95 }, { 0.05,0,4.95 }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ 0,0,5.05f }, { 0,0.05,4.95 }, { 0,-0.05,4.95 }, Colors::Blue);
	g_PrimitiveDrawer.DrawTriangle({ 0,0,5.05f }, { 0,-0.05,4.95 }, { 0,0.05,4.95 }, Colors::Blue);
	g_PrimitiveDrawer.End();
}

void XM_CALLCONV Causality::CoordinateAxis::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	g_PrimitiveDrawer.SetView(view);
	g_PrimitiveDrawer.SetProjection(projection);
	g_PrimitiveDrawer.SetWorld(GlobalTransformMatrix());
}
