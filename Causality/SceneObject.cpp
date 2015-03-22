#include "pch_bcl.h"
#include "SceneObject.h"

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

void Causality::SceneObject::SetPosition(const Vector3 & p)
{
	m_GlobalTransformDirty = true;
	AffineTransform::Translation = p;
}

void Causality::SceneObject::SetOrientation(const Quaternion & q)
{
	m_GlobalTransformDirty = true;
	AffineTransform::Rotation = q;
}

void Causality::SceneObject::SetScale(const Vector3 & s)
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

void Causality::RenderableSceneObject::Render(RenderContext & pContext)
{
	//m_pRenderModel->SetModelMatrix(this->GlobalTransformMatrix());
	m_pRenderModel->Render(pContext,GlobalTransformMatrix());
}

void XM_CALLCONV Causality::RenderableSceneObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{

	return;
}

bool Causality::RenderableSceneObject::IsVisible(const BoundingFrustum & viewFrustum) const
{
	auto box = m_pRenderModel->GetOrientedBoundingBox();
	box.Transform(box,this->GlobalTransformMatrix());
	return viewFrustum.Contains(box) != DirectX::ContainmentType::DISJOINT;
}
