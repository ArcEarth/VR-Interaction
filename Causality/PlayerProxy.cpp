#include "PlayerProxy.h"
#include "Common\PrimitiveVisualizer.h"
#include <fstream>
#include <Eigen\fft>
using namespace Causality;

Causality::PlayerProxy::~PlayerProxy()
{
	//std::ofstream fout("handpos.txt", std::ofstream::out);

	//fout.close();
}

void Causality::PlayerProxy::Initialize()
{
	pKinect = Devices::Kinect::GetForCurrentView();
	pPlayerArmature = &pKinect->Armature();
	for (auto& child : children())
	{
		auto pObj = dynamic_cast<KinematicSceneObject*>(&child);
		if (pObj)
		{
			States.emplace_back();
			States.back().SetSourceArmature(*pPlayerArmature);
			States.back().SetTargetObject(*pObj);
		}
	}
}

void Causality::PlayerProxy::Update(time_seconds const & time_delta)
{
	const auto &players = pKinect->GetLatestPlayerFrame();
	if (players.size() != 0)
	{
		auto player = players.begin()->second;
		const auto& frame = player->PoseFrame;
		if (FrameBuffer.full())
			FrameBuffer.pop_back();
		FrameBuffer.push_front(frame);
		Eigen::FFT<float> fft;

		UpdatePlayerFrame(frame);
	}
}

bool PlayerProxy::UpdatePlayerFrame(const BoneDisplacementFrame & frame)
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

		binding.Transform(obj.PotientialFrame, frame);
		Likilihood(i) = space.FrameLikilihood(frame);
	}

	// State evolution
	StateProbality.noalias() = TransferMatrix * StateProbality;

	// 
	StateProbality = StateProbality.cwiseProduct(Likilihood);
	float c = StateProbality.sum();
	StateProbality = StateProbality / c;

	for (size_t i = 0; i < States.size(); i++)
	{
		States[i].Object().SetOpticity(StateProbality(i));
	}

	int maxIdx = -1;
	auto currentP = StateProbality.maxCoeff(&maxIdx);
	if (maxIdx != CurrentIdx)
	{
		int oldIdx = CurrentIdx;
		CurrentIdx = maxIdx;
		StateChangedEventArgs arg = { oldIdx,maxIdx,1.0f,States[oldIdx],States[maxIdx] };
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

bool Causality::PlayerProxy::IsVisible(const BoundingFrustum & viewFrustum) const
{
	return g_DebugView;
}

void Causality::PlayerProxy::Render(RenderContext & context)
{
}

void XM_CALLCONV Causality::PlayerProxy::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
}

Causality::KinectVisualizer::KinectVisualizer()
{
	pKinect = Devices::Kinect::GetForCurrentView();
}

bool Causality::KinectVisualizer::IsVisible(const BoundingFrustum & viewFrustum) const
{
	return true;
}

void Causality::KinectVisualizer::Render(RenderContext & context)
{
	const auto &players = pKinect->GetLatestPlayerFrame();
	using DirectX::Visualizers::g_PrimitiveDrawer;

	for (auto& item : players)
	{
		const auto& player = *item.second;
		const auto& frame = player.PoseFrame;

		for (auto& bone : frame)
		{
			g_PrimitiveDrawer.DrawCylinder(bone.OriginPosition, bone.EndPostion, 0.01f, DirectX::Colors::LimeGreen);
		}
	}
}

void XM_CALLCONV Causality::KinectVisualizer::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
}

inline const ArmatureTransform & Causality::PlayerProxy::ControlState::Binding() const { return m_Binding; }

inline ArmatureTransform & Causality::PlayerProxy::ControlState::Binding() { return m_Binding; }

inline const KinematicSceneObject & Causality::PlayerProxy::ControlState::Object() const { return *m_pSceneObject; }

inline KinematicSceneObject & Causality::PlayerProxy::ControlState::Object() { return *m_pSceneObject; }

inline void Causality::PlayerProxy::ControlState::SetSourceArmature(const IArmature & armature) { m_Binding.SetSourceArmature(armature); }

inline void Causality::PlayerProxy::ControlState::SetTargetObject(KinematicSceneObject & object) {
	m_pSceneObject = &object;
	m_Binding.SetTargetArmature(object.Armature());
	PotientialFrame = object.Armature().default_frame();
}
