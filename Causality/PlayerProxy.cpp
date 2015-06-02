#include "pch_bcl.h"
#include "PlayerProxy.h"
#include "Common\PrimitiveVisualizer.h"
#include <fstream>
#include <Eigen\fft>
#include "CCA.h"
#include "EigenExtension.h"
#include <boost\multi_array.hpp>

using namespace Causality;
using namespace Eigen;
using namespace std;

class CcaArmatureTransform : public ArmatureTransform
{
public:
	struct CcaMap
	{
		DenseIndex Jx, Jy;
		MatrixXf A, B;
		RowVectorXf uX, uY;
		JacobiSVD<MatrixXf> svdBt;
		MatrixXf invB;
		bool useInvB;
	};

	std::vector<CcaMap> Maps;

public:
	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		const auto dF = frame_type::StdFeatureDimension;
		const auto dY = 3;
		RowVectorXf X(source_frame.size() * dF);
		RowVectorXf Y(target_frame.size() * dF);
		source_frame.PopulateStdFeatureVector(X);
		target_frame.PopulateStdFeatureVector(Y); //! populate non-mapped with default values

		for (const auto& map : Maps)
		{
			auto& Xp = X.middleCols<dF>(map.Jx * dF);
			auto& Yp = Y.middleCols<dY>(map.Jy * dF);
			auto U = ((Xp.rowwise() - map.uX) * map.A).eval();
			if (map.useInvB)
				Yp = U * map.invB;
			else
				Yp = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
			Yp.rowwise() += map.uY;
		}

		target_frame.RebuildFromStdFeatureVector(Y, *pTarget);
	}

};

std::pair<JointType, JointType> XFeaturePairs[] = {
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

JointType KeyJoints[] = {
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

float BoneRadius[JointType_Count] = {

};

static const size_t KeyJointCount = ARRAYSIZE(KeyJoints);

static const size_t FeatureCount = (KeyJointCount * (KeyJointCount - 1)) / 2;
static const size_t FeatureDim = FeatureCount * 3;

VectorXf HumanFeatureFromFrame(const AffineFrame& frame)
{
	VectorXf X(ARRAYSIZE(XFeaturePairs) * 3);
	for (size_t i = 0; i < ARRAYSIZE(XFeaturePairs); i++)
	{
		auto& p = XFeaturePairs[i];
		Vector3 v = frame[p.first].EndPostion - frame[p.second].EndPostion;
		X.block<3, 1>(i * 3, 0) = Map<Vector3f>(&v.x);
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
	: FrameBuffer(BufferFramesCount), pBody(nullptr)
{
	IsInitialized = false;
	CurrentIdx = -1;
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
			pObj->SetOpticity(0.5f);
		}
	}

	pKinect->OnPlayerTracked += [this](TrackedBody& body)
	{
		if (!this->IsConnected() || !this->ConnectedBody()->IsCurrentTracked)
		{
			this->Connect(&body);
		}
	};

	pKinect->OnPlayerLost += [this](TrackedBody& body)
	{
		if (body.Id == this->ConnectedBody()->Id)
		{
			this->ResetConnection();
		}
	};

	IsInitialized = true;
}

int Causality::PlayerProxy::SelectControlStateByLatestFrames()
{
	auto& player = *pBody;
	static std::ofstream flog("Xlog.txt");

	MatrixXf X = player.GetFeatureMatrix();
	//? WARNNING! player.GetFeatureMatrix() is ROW Major!!!
	//? But we have convert it into Column Major! Yes and No!!!
	auto N = X.rows();
	auto K = X.cols();

	if (N*K == 0) return -1; // Feature Matrix is not ready yet

	cout << "X : min = " << X.minCoeff() << " , max = " << X.maxCoeff() << endl;
	//cout << "X = " << X << endl;
	flog << X << endl;
	flog.flush();

	MatrixXcf Xf(N, K);
	MatrixXf Xr(N, K);

	FFT<float> fft;
	for (size_t i = 0; i < K; i++)
		fft.fwd(Xf.col(i).data(), X.col(i).data(), N);

	MatrixXf Xs = Xf.cwiseAbs2();

	int idx;
	VectorXf Ea = Xs.middleRows(3, 20).rowwise().sum().transpose();
	Ea.middleRows(1, Ea.rows() - 2).maxCoeff(&idx); // Frequency 3 - 30
	cout << Ea.transpose() << endl;

	auto Ex = Ea.middleRows<3>(idx);
	idx += 3;
	cout << Ex.transpose() << endl;

	Vector3f Ix = { idx - 1.0f, (float)idx, idx + 1.0f };
	float peekFreq = Ex.dot(Ix) / Ex.sum();

	size_t T = ceil(N / peekFreq);
	// Peek Frequency

	float sigma = 6;
	auto G = ArrayXf::LinSpaced(N, 0, sigma);
	auto Eg = (-G.square()).exp().eval();

	cout << "Eg : min = " << Eg.minCoeff() << " , max = " << Eg.maxCoeff() << endl;
	cout << "Xf : min = " << Xs.minCoeff() << " , max = " << Xs.maxCoeff() << endl;
	Xf.array() *= Eg.replicate(1, K).array(); // low-pass filter

	for (size_t i = 0; i < K; i++)
		fft.inv(Xr.col(i), Xf.col(i));

	cout << "Xr : min = " << Xr.minCoeff() << " , max = " << Xr.maxCoeff() << endl;
	Xs = resample(Xr.bottomRows(2 * T), T, ScaledFramesCount);
	cout << "Xs : min = " << Xs.minCoeff() << " , max = " << Xs.maxCoeff() << endl;

	T = ScaledFramesCount; // Since we have resampled , Time period "T" is now equals to ScaledFramesCount
	int Ti = 5; // 2
	int Ts = T / Ti;

	size_t Ju = JointType_Count;
	vector<vector<MeanThinQr<Matrix<float, -1, JointDemension>>>> QrXs(Ts);
	for (auto& v : QrXs)
		v.resize(Ju);

	//? average two period = =||| this is a HACK, not right
	Xs.topRows(T) += Xs.bottomRows(T);
	Xs.bottomRows(T) += Xs.topRows(T);
	Xs.array() *= 0.5f;
	//Xs.rowwise() -= Xs.colwise().mean();
	cout << "Xs : min = " << Xs.minCoeff() << " , max = " << Xs.maxCoeff() << endl;

	// Fuck it... 11,250,000 times of CCA(SVD) computation every guess
	// 1,406,250 SVD for 5x5 setup, much more reasonable!
	for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
	{
		auto Xp = Xs.middleRows(T - phi, T);
		for (size_t i = 0; i < Ju; i++)			//? <= 25 joint per USER?
		{
			auto Xk = Xp.middleCols<JointDemension>(i*JointDemension);
			QrXs[phi/Ti][i].compute(Xk, false);
		}
	}

	//? HACK Juk Jck! Pre-reduce DOF
	size_t Juk[] = { JointType_Head,JointType_ElbowLeft,JointType_ElbowRight,JointType_KneeLeft,JointType_KneeRight };
	size_t Jck[] = { 8,12,17,22,35,40,45,50,54};
	for (auto& jc : Jck)
		jc -= 1;

	////? HACK Ju
	//Ju = std::size(Juk);


	for (auto& state : States)			//? <= 5 character
	{
		auto& object = state.Object();
		size_t Jc = object.Armature().size();

		////? HACK Jc
		//Jc = std::size(Jck);

		auto& anim = object.Behavier()["walk"];
		//x for (auto& action : object.Behavier())	//? <= 5 animation per character
		{
			//auto& anim = action.second;
			boost::multi_array<float, 3> Rm(boost::extents[Ts][Ju][Jc]);
			std::fill_n(Rm.data(), Ts*Ju*Jc, .0f);

			float maxScore = std::numeric_limits<float>::min();
			DenseIndex maxPhi = -1;
			std::vector<DenseIndex> maxMatching;

			//DenseIndex phi = 0;
			for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
			{
				auto Xp = Xs.middleRows(T - phi, T);

				//int i = JointType_ElbowRight;
				for (auto& i : Juk)
				//for (size_t i = 0; i < Ju; i++)			//? <= 25 joint per USER?
				{
					auto& qrX = QrXs[phi / Ti][i];

					auto Xk = Xp.middleCols<JointDemension>(i*JointDemension);
					//cout << "X : min = " << Xk.minCoeff() << " , max = " << Xk.maxCoeff() << endl;

					//int j = 17;
					for (auto& j : Jck)
					//for (size_t j = 0; j < Jc; j++)		//? <= 50, we should applies joint reduce in characters
					{
						auto& qrY = anim.QrYs[j];
						Cca cca;
						cca.computeFromQr(qrX, qrY, true);	//? it's <= 6x6 SVD inside
						auto r = cca.correlaltions().minCoeff();
						Rm[phi / Ti][i][j] = r;
					}
				}

				auto A = MatrixXf::Map(&Rm[phi / Ti][0][0], Jc, Ju).transpose();
				auto matching = max_weight_bipartite_matching(A);
				auto score = matching_cost(A,matching);
				if (score > maxScore)
				{
					maxPhi = phi;
					maxMatching = matching;
					maxScore = score;
				}
			}

			state.SpatialMotionScore = maxScore;
			auto pBinding = new CcaArmatureTransform();
			state.SetBinding(pBinding);
			cout << "Best assignment for " << state.Object().Name << " : " << anim.Name << endl;

			int sdxass[] = {3,14,19,24,29};
			for (size_t i = 0; i < std::size(sdxass); i++)
				maxMatching[Juk[i]] = sdxass[i];

			for (auto i : Juk)
			{
				DenseIndex Jx = i, Jy = maxMatching[i];
				if (Jy == -1) continue;

				cout << pPlayerArmature->at(Jx)->Name() << " ==> " << state.Object().Armature()[Jy]->Name() << endl;

				pBinding->Maps.emplace_back();
				pBinding->SetSourceArmature(*pPlayerArmature);
				pBinding->SetTargetArmature(state.Object().Armature());

				auto& map = pBinding->Maps.back();
				auto& qrX = QrXs[maxPhi / Ti][Jx];
				auto& qrY = anim.QrYs[Jy];

				Cca cca;
				cca.computeFromQr(QrXs[maxPhi / Ti][Jx], anim.QrYs[Jy], true);

				// populate map
				map.Jx = Jx; map.Jy = Jy;
				map.A = cca.matrixA();
				map.B = cca.matrixB();
				map.uX = qrX.mean();
				map.uY = qrY.mean();
				if (cca.rank() == qrY.cols()) // d == dY
				{
					map.useInvB = true;
					map.invB = map.B.inverse();
				}
				else
				{
					map.useInvB = false;
					map.svdBt = JacobiSVD<MatrixXf>(map.B.transpose(),ComputeThinU | ComputeThinV);;
				}
			}
		}
		CurrentIdx = state.ID;
	}

	return 1;
}

//x DON"T USE THIS
std::pair<float, float> PlayerProxy::ExtractUserMotionPeriod()
{
	MatrixXf
		X(BufferFramesCount, JointCount * JointDemension);
	MatrixXcf
		Xf(BufferFramesCount, JointCount * JointDemension);
	MatrixXf
		JointReductedSpecturm(BufferFramesCount, JointCount);
	Array<float, JointCount, 1>		JointPeriod; // Period time in seconds, from fft Peek frequency

	FFT<float> fft;
	for (size_t i = 0; i < X.rows(); i++)
	{
		fft.fwd(Xf.row(i), X.row(i));
	}
	Xf = Xf.array() * Xf.array().conjugate();
	X = Xf.real().transpose();

	for (size_t i = 0; i < JointCount; i++)
	{
		Map<Matrix<float, JointCount, JointDemension>> Jsp(&X(0, i * JointDemension));
		// sum up spectrum energy for X-Y-Z components
		JointReductedSpecturm.col(i) = Jsp.rowwise().sum();
	}

	for (size_t j = 0; j < JointReductedSpecturm.rows(); j++)
	{
		int idx;
		// filter frequency with 3-15 reoccurence times
		JointReductedSpecturm.block<1, BandWidth>(j, MinimumFrequency).maxCoeff(&idx);
		JointPeriod[j] = 15.0f / (float)(idx + 3); // 450 frame/(30 frame/s)
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
	using namespace std;
	using namespace Eigen;

	if (!IsInitialized || !IsConnected())
		return;

	static long long frame_count = 0;

	auto& player = *pBody;
	if (!player.IsCurrentTracked) return;

	if (IsMapped())
	{
		auto& state = CurrentState();
		//state.Object().StopAction();
		auto& cframe = state.Object().MapCurrentFrameForUpdate();
		state.Binding().Transform(cframe, player.GetPoseFrame());
		state.Object().ReleaseCurrentFrameFrorUpdate();
		return;
	}

	if (frame_count++ % 100 != 0)
	{
		return;
	}

	if (player.FeatureBuffer.size() >= 0)
	{
		SelectControlStateByLatestFrames();
		cout << "Mapped!!!!!!!!!" << endl;
		if (IsMapped())
			CurrentState().Object().StopAction();
	}
}

bool PlayerProxy::UpdatePlayerFrame(const AffineFrame & frame)
{
	AffineFrame tframe;
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

	if (IsMapped())
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
	using DirectX::Visualizers::g_PrimitiveDrawer;

	if (!IsConnected()) return;
	auto& player = *pBody;

	DirectX::Color color = DirectX::Colors::LimeGreen.v;

	if (IsMapped())
		color.A(0.05f);

	if (player.IsCurrentTracked)
	{
		const auto& frame = player.GetPoseFrame();
		if (&frame == nullptr) return;

		for (auto& bone : frame)
		{
			g_PrimitiveDrawer.DrawCylinder(bone.OriginPosition, bone.EndPostion, 0.015f, color);
			g_PrimitiveDrawer.DrawSphere(bone.EndPostion, 0.03f, color);
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

inline const ArmatureTransform & Causality::PlayerProxy::ControlState::Binding() const { return *m_pBinding; }

inline ArmatureTransform & Causality::PlayerProxy::ControlState::Binding() { return *m_pBinding; }

void Causality::PlayerProxy::ControlState::SetBinding(ArmatureTransform * pBinding)
{
	m_pBinding = pBinding;
}

inline const KinematicSceneObject & Causality::PlayerProxy::ControlState::Object() const { return *m_pSceneObject; }

inline KinematicSceneObject & Causality::PlayerProxy::ControlState::Object() { return *m_pSceneObject; }

inline void Causality::PlayerProxy::ControlState::SetSourceArmature(const IArmature & armature) { 
	if (m_pBinding)
		m_pBinding->SetSourceArmature(armature);
}

inline void Causality::PlayerProxy::ControlState::SetTargetObject(KinematicSceneObject & object) {
	m_pSceneObject = &object;
	if (m_pBinding)
		m_pBinding->SetTargetArmature(object.Armature());
	PotientialFrame = object.Armature().default_frame();
}
