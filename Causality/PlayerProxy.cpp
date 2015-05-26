#include "pch_bcl.h"
#include "PlayerProxy.h"
#include "Common\PrimitiveVisualizer.h"
#include <fstream>
#include <Eigen\fft>
using namespace Causality;
using namespace Eigen;
using namespace std;

std::pair<JointType, JointType> XFeaturePairs [] = { 
	{JointType_SpineBase, JointType_SpineShoulder},
	{ JointType_SpineShoulder, JointType_Head },
	{JointType_ShoulderLeft, JointType_ElbowLeft},
	{JointType_ShoulderRight, JointType_ElbowRight },
	{ JointType_ElbowLeft, JointType_HandLeft },
	{ JointType_ElbowRight, JointType_HandRight },
	{ JointType_HipLeft, JointType_KneeLeft},
	{ JointType_HipRight, JointType_KneeRight},
	{ JointType_KneeLeft, JointType_AnkleLeft},
	{ JointType_KneeRight, JointType_AnkleRight},
	//{ JointType_HandLeft, JointType_HandTipLeft },
	//{ JointType_HandRight, JointType_HandTipRight },
	//{ JointType_HandLeft, JointType_ThumbLeft },
	//{ JointType_HandRight, JointType_ThumbRight },
};

JointType KeyJoints [] = {
	JointType_SpineBase,		//1
	JointType_SpineShoulder,	//2
	JointType_Head,				//3
	JointType_ShoulderLeft,		//4
	JointType_ElbowLeft,		//5
	JointType_WristLeft,		//6
	JointType_ShoulderRight,	//7
	JointType_ElbowRight,		//8
	JointType_WristRight,		//9
	JointType_HipLeft,			//10
	JointType_KneeLeft,			//11
	JointType_AnkleLeft,		//12
	JointType_HipRight,			//13
	JointType_KneeRight,		//14
	JointType_AnkleRight,		//15
};

static const size_t KeyJointCount = ARRAYSIZE(KeyJoints);

static const size_t FeatureCount = (KeyJointCount * (KeyJointCount - 1))/2;
static const size_t FeatureDim = FeatureCount * 3;

VectorXf HumanFeatureFromFrame(const BoneDisplacementFrame& frame)
{
	VectorXf X(ARRAYSIZE(XFeaturePairs)*3);
	for (size_t i = 0; i < ARRAYSIZE(XFeaturePairs); i++)
	{
		auto& p = XFeaturePairs[i];
		Vector3 v = frame[p.first].EndPostion - frame[p.second].EndPostion;
		X.block<3, 1>(i * 3,0) = Map<Vector3f>(&v.x);
	}

	//VectorXf X(FeatureDim);
	//int k = 0;
	//Vector3 v;
	//for (size_t i = 0; i < KeyJointCount; i++)
	//{
	//	for (size_t j = i+1; j < KeyJointCount; j++)
	//		if (i != j)
	//		{
	//			v = frame[KeyJoints[i]].EndPostion - frame[KeyJoints[j]].EndPostion;
	//			X.block<3, 1>(k++ * 3,0) = Map<Vector3f>(&v.x);
	//		}
	//}
	return X;
}

Causality::PlayerProxy::PlayerProxy()
	: FrameBuffer(BufferFramesCount)
{
	IsInitialized = false;
}

Causality::PlayerProxy::~PlayerProxy()
{
	//std::ofstream fout("handpos.txt", std::ofstream::out);

	//fout.close();
}

void Causality::PlayerProxy::Initialize()
{
	pKinect = Devices::Kinect::GetForCurrentView();
	pPlayerArmature = &pKinect->Armature();
	//FrameBuffer.clear();
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

	pKinect->OnPlayerTracked += [this](TrackedBody& body)
	{
		if (!this->ConnectedBody()->IsCurrentTracked)
		{
			this->Connect(&body);
		}
	};

	pKinect->OnPlayerTracked += [this](TrackedBody& body)
	{
		if (body.Id == this->ConnectedBody()->Id)
		{
			this->ResetConnection();
		}
	};

	IsInitialized = true;
}

std::pair<float,float> PlayerProxy::ExtractUserMotionPeriod()
{
	MatrixXf
		MotionBuffer(BufferFramesCount, JointCount * JointDemension);
	MatrixXcf
		MotionSpecturm(BufferFramesCount, JointCount * JointDemension);
	MatrixXf
		JointReductedSpecturm(BufferFramesCount, JointCount);
	Array<float, JointCount,1>		JointPeriod; // Period time in seconds, from fft Peek frequency

	FFT<float> fft;
	for (size_t i = 0; i < MotionBuffer.rows(); i++)
	{
		fft.fwd(MotionSpecturm.row(i), MotionBuffer.row(i));
	}
	MotionSpecturm = MotionSpecturm.array() * MotionSpecturm.array().conjugate();
	MotionBuffer = MotionSpecturm.real().transpose();

	for (size_t i = 0; i < JointCount; i++)
	{
		Map<Matrix<float, JointCount, JointDemension>> Jsp(&MotionBuffer(0,i * JointDemension));
		// sum up spectrum energy for X-Y-Z components
		JointReductedSpecturm.col(i) = Jsp.rowwise().sum(); 
	}

	for (size_t j = 0; j < JointReductedSpecturm.rows(); j++)
	{
		int idx;
		// filter frequency with 3-15 reoccurence times
		JointReductedSpecturm.block<1, BandWidth>(j, MinimumFrequency).maxCoeff(&idx);
		JointPeriod[j] = 15.0f / (float) (idx + 3); // 450 frame/(30 frame/s)
	}

	auto overallSpectrum = JointReductedSpecturm.colwise().sum();

	int idx;
	overallSpectrum.block<1, BandWidth>(1, MinimumFrequency).maxCoeff(&idx);
	return make_pair(.0f, idx);
}

void Causality::PlayerProxy::PrintFrameBuffer(int No)
{
	int flag = std::ofstream::out | (No == 1 ? 0 : std::ofstream::app);
	std::ofstream fout("handpos.csv", flag);

	//Matrix<float, Dynamic, FeatureDim> X(BufferFramesCount);
	for (auto& frame : FrameBuffer)
	{
		auto Xj = HumanFeatureFromFrame(frame);

		for (size_t i = 0; i < Xj.size() - 1; i++)
		{
			fout << Xj(i) << ',';
		}
		fout << Xj(Xj.size() - 1) << endl;

		//for (auto& bone : frame)
		//{
		//	const auto& pos = bone.EndPostion;
		//	//DirectX::XMVECTOR quat = bone.LclRotation;
		//	//Vector3 pos = DirectX::XMQuaternionLn(quat);
		//	fout << pos.x << ',' << pos.y << ',' << pos.z << ',';
		//}
		//fout << endl;
	}
	fout.close();
}

void Causality::PlayerProxy::Update(time_seconds const & time_delta)
{
	if (!IsInitialized || !IsConnected())
		return;
	static long long frame_count = 0;

	auto& player = *pBody;
	if (!player.IsCurrentTracked) return;

	const auto& frame = player.GetPoseFrame();
	if (&frame == nullptr) 
		return;

	if (FrameBuffer.full())
		FrameBuffer.pop_back();
	FrameBuffer.push_front(frame);

	++frame_count;
	if (frame_count % BufferFramesCount == 0 && frame_count != 0)
	{
		PrintFrameBuffer(frame_count / BufferFramesCount);
	}

	UpdatePlayerFrame(frame);
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
	auto &players = pKinect->GetTrackedBodies();
	using DirectX::Visualizers::g_PrimitiveDrawer;

	for (auto& player : players)
	{
		if (player.IsCurrentTracked)
		{
			const auto& frame = player.GetPoseFrame();
			if (&frame == nullptr) return;

			for (auto& bone : frame)
			{
				g_PrimitiveDrawer.DrawCylinder(bone.OriginPosition, bone.EndPostion, 0.015f, DirectX::Colors::LimeGreen);
				g_PrimitiveDrawer.DrawSphere(bone.EndPostion, 0.03f, DirectX::Colors::LimeGreen);
			}
		}
	}
}

void XM_CALLCONV Causality::PlayerProxy::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
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
	auto &players = pKinect->GetTrackedBodies();
	using DirectX::Visualizers::g_PrimitiveDrawer;

	for (auto& player : players)
	{
		if (player.IsCurrentTracked)
		{
			const auto& frame = player.GetPoseFrame();
			if (&frame == nullptr) return;

			for (auto& bone : frame)
			{
				g_PrimitiveDrawer.DrawCylinder(bone.OriginPosition, bone.EndPostion, 0.015f, DirectX::Colors::LimeGreen);
				g_PrimitiveDrawer.DrawSphere(bone.EndPostion, 0.03f, DirectX::Colors::LimeGreen);
			}
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
