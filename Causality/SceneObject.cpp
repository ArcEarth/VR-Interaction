#include "SceneObject.h"

bool Causality::PlayerController::UpdatePlayerFrame(const BoneDisplacementFrame & frame)
{
	BoneDisplacementFrame tframe;
	// Caculate likilihood
	for (int i = 0; i < States.size(); ++i)
	{
		auto& obj = States[i];
		auto& binding = obj.Binding();
		auto& kobj = obj.Object();
		auto& armature = kobj.Armature();
		auto& space = kobj.Space();

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

	if (!IsIdel)
	{
		auto& state = CurrentState();
		auto& cframe = state.Object().MapCurrentFrameForUpdate();
		state.Binding().Transform(cframe, frame);
		state.Object().ReleaseCurrentFrameFrorUpdate();
	}
}

void Causality::KinematicSceneObject::ReleaseCurrentFrameFrorUpdate()
{
	DirectX::SkinnedEffect *pEffect;
}

inline DirectX::XMMATRIX Causality::SceneObject::GlobalTransformMatrix() const
{
	if (Parent != nullptr)
	{
		DirectX::XMMATRIX mat = Parent->GlobalTransformMatrix();
		mat *= TransformMatrix();
		return mat;
	}
	else
	{
		return TransformMatrix();
	}
}
