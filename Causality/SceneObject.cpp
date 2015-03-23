#include "pch_bcl.h"
#include "SceneObject.h"
#include "CausalityApplication.h"

using namespace Causality;

bool PlayerController::UpdatePlayerFrame(const BoneDisplacementFrame & frame)
{
	BoneDisplacementFrame tframe;
	// Caculate likilihood
	for (int i = 0, end = States.size(); i < end; ++i)
	{
		auto& obj = States[i];
		auto& binding = obj.Binding();
		auto& kobj = obj.Object();
		auto& armature = kobj.Armature();
		auto& space = kobj.Behavier();

		binding.Transform(obj.PotientialFrame,frame);
		Likilihood(i) = space.FrameLikilihood(frame);
	}

	// State evolution
	StateProbality.noalias() = TransferMatrix * StateProbality;

	// 
	StateProbality = StateProbality.cwiseProduct(Likilihood);
	float c = StateProbality.sum();
	StateProbality = StateProbality / c;

	int maxIdx = -1;
	auto currentP = StateProbality.maxCoeff(&maxIdx);
	if (maxIdx != CurrentIdx)
	{
		int oldIdx = CurrentIdx;
		CurrentIdx = maxIdx;
		StateChangedEventArgs arg = {oldIdx,maxIdx,1.0f,States[oldIdx],States[maxIdx]};
		if (!StateChanged.empty())
			StateChanged(arg);
	}

	if (!IsIdel())
	{
		auto& state = CurrentState();
		auto& cframe = state.Object().MapCurrentFrameForUpdate();
		state.Binding().Transform(cframe, frame);
		state.Object().ReleaseCurrentFrameFrorUpdate();
	}
	return true;
}

KinematicSceneObject::frame_type & KinematicSceneObject::MapCurrentFrameForUpdate()
{
	return m_CurrentFrame;
}

void KinematicSceneObject::ReleaseCurrentFrameFrorUpdate()
{
	m_DirtyFlag = true;
}

IArmature & KinematicSceneObject::Armature() { return m_pAnimationSpace->Armature(); }

const IArmature & KinematicSceneObject::Armature() const { return m_pAnimationSpace->Armature(); }

inline AnimationSpace & KinematicSceneObject::Behavier() { return *m_pAnimationSpace; }

const AnimationSpace & KinematicSceneObject::Behavier() const { return *m_pAnimationSpace; }

void KinematicSceneObject::SetBehavier(AnimationSpace & behaver) { m_pAnimationSpace = &behaver; }

inline DirectX::XMMATRIX SceneObject::GlobalTransformMatrix() const
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
	return m_pRenderModel.get();
}

const DirectX::Scene::IModelNode * RenderableSceneObject::RenderModel(int LoD) const
{
	return m_pRenderModel.get();
}

void RenderableSceneObject::SetRenderModel(DirectX::Scene::IModelNode * pMesh, int LoD)
{
	return m_pRenderModel.reset(pMesh);
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
	auto box = m_pRenderModel->GetOrientedBoundingBox();
	box.Transform(box,this->GlobalTransformMatrix());
	return viewFrustum.Contains(box) != DirectX::ContainmentType::DISJOINT;
}

KeyboardMouseFirstPersonControl::KeyboardMouseFirstPersonControl(IRigid* pTarget = nullptr)
{
	Speed = 2.0f;
	SetTarget(pTarget);
}

void KeyboardMouseFirstPersonControl::SetTarget(IRigid * pTarget)
{
	if (pTarget && !m_pTarget)
		Register();
	else if (!pTarget && m_pTarget)
		Unregister();
	m_pTarget = pTarget;
	InitialOrientation = pTarget->GetOrientation();
	AddationalYaw = 0;
	AddationalPitch = 0;
	AddationalRoll = 0;
}

void KeyboardMouseFirstPersonControl::UpdateAnimation(time_seconds const& time_delta)
{
	if (m_pTarget)
	m_pTarget->Move(XMVector3Normalize(CameraVeclocity) * (Speed * (float) time_delta.count()));
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
	auto yaw = -e.PositionDelta.x / 1000.0f * XM_PI;
	auto pitch = -e.PositionDelta.y / 1000.0f * XM_PI;
	AddationalYaw += yaw;
	AddationalPitch += pitch;
	if (m_pTarget)
		m_pTarget->SetOrientation((Quaternion) XMQuaternionRotationRollPitchYaw(AddationalPitch, AddationalYaw, 0)*InitialOrientation);
}
