#include "pch_bcl.h"
#include <PrimitiveVisualizer.h>
#include <fstream>
#include <algorithm>
#include <ppl.h>
#include <boost\filesystem.hpp>
#include <random>
#include <unsupported\Eigen\fft>
#include <unsupported\Eigen\CXX11\Tensor>
#include "CCA.h"
#include "EigenExtension.h"
#include "GaussianProcess.h"

#include "ArmatureBlock.h"
#include "AnimationAnalyzer.h"

#include "PlayerProxy.h"
#include "Scene.h"
#include <tinyxml2.h>
#include <boost\format.hpp>
#include "ArmatureTransforms.h"
#include "Settings.h"


//					When this flag set to true, a CCA will be use to find the general linear transform between Player 'Limb' and Character 'Limb'

//float				g_NoiseInterpolation = 1.0f;


using namespace Causality;
using namespace Eigen;
using namespace std;
using namespace ArmaturePartFeatures;
using boost::filesystem::path;

path g_LogRootDir = "Log";
static const char*  DefaultAnimationSet = "walk";
Eigen::RowVector3d	g_NoiseInterpolation = { 1.0,1.0,1.0 };
const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

ShrinkedArmature		 g_PlayeerBlocks;
std::map<string, string> g_DebugLocalMotionAction;
bool					 g_DebugLocalMotion = false;

pair<JointType, JointType> XFeaturePairs[] = {
	{ JointType_SpineBase, JointType_SpineShoulder },
	{ JointType_SpineShoulder, JointType_Head },
	{ JointType_ShoulderLeft, JointType_ElbowLeft },
	{ JointType_ShoulderRight, JointType_ElbowRight },
	{ JointType_ElbowLeft, JointType_HandLeft },
	{ JointType_ElbowRight, JointType_HandRight },
	{ JointType_HipLeft, JointType_KneeLeft },
	{ JointType_HipRight, JointType_KneeRight },
	{ JointType_KneeLeft, JointType_AnkleLeft },
	{ JointType_KneeRight, JointType_AnkleRight },
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

#define BEGIN_TO_END(range) range.begin(), range.end()


// Player Proxy methods
void PlayerProxy::StreamPlayerFrame(const TrackedBody& body, const TrackedBody::FrameType& frame)
{
	using namespace Eigen;
	using namespace DirectX;
	std::lock_guard<std::mutex> guard(m_BufferMutex);

	FeatureBuffer.push_back();

	if (FeatureBuffer.size() > RecordFeatures)
		FeatureBuffer.pop_front(); // everything works fine beside it invaliad the memery...

	if (!FeatureBuffer.is_linearized())
		FeatureBuffer.linearize();

	auto& fb = FeatureBuffer.back();
	float* vs = reinterpret_cast<float*>(fb.data());

	BoneHiracheryFrame rotatedFrame = frame;
	if (g_IngnoreInputRootRotation)
	{
		auto& rotBone = rotatedFrame[m_pPlayerArmature->root()->ID];
		rotBone.GblRotation = rotBone.LclRotation = Quaternion::Identity;
		rotatedFrame.RebuildGlobal(*m_pPlayerArmature);
	}

	for (const auto& block : g_PlayeerBlocks)
	{
		auto vsi = RowVectorXf::Map(
			vs + block->AccumulatedJointCount * InputFeature::Dimension,
			block->Joints.size() * InputFeature::Dimension);

		vsi = m_pPlayerFeatureExtrator->Get(*block, frame);
	}
}

void PlayerProxy::ResetPlayer(TrackedBody * pOld, TrackedBody * pNew)
{
	ClearPlayerFeatureBuffer();
	SetActiveController(-1);
}

Eigen::Map<PlayerProxy::FeatureMatrixType> PlayerProxy::GetPlayerFeatureMatrix(time_seconds duration)
{
	using namespace std;
	int si = duration.count() * 30;
	if (FeatureBuffer.size() >= si)
	{
		std::lock_guard<mutex> guard(m_BufferMutex);
		si = min(max(si, 0), (int)FeatureBuffer.size());
		auto sidx = FeatureBuffer.size() - si;
		auto head = &FeatureBuffer[sidx][0];
		return FeatureMatrixType::Map(head, si, FeatureMatrixType::ColsAtCompileTime);
	}
	else
	{
		return FeatureMatrixType::Map((float*)nullptr, 0, FeatureMatrixType::ColsAtCompileTime);
	}
}

VectorXf HumanFeatureFromFrame(const BoneHiracheryFrame& frame)
{
	VectorXf X(ARRAYSIZE(XFeaturePairs) * 3);
	for (size_t i = 0; i < ARRAYSIZE(XFeaturePairs); i++)
	{
		auto& p = XFeaturePairs[i];
		Vector3 v = frame[p.first].GblTranslation - frame[p.second].GblTranslation;
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
	//			v = frame[KeyJoints[i]].GblTranslation - frame[KeyJoints[j]].GblTranslation;
	//			X.block<3, 1>(k++ * 3,0) = Map<Vector3f>(&v.x);
	//		}
	//}
	return X;
}

PlayerProxy::PlayerProxy()
	: m_IsInitialized(false),
	m_playerSelector(nullptr),
	m_CurrentIdx(-1),
	FeatureBuffer(FeatureBufferCaptcity),
	current_time(0),
	m_mapTaskOnGoing(false),
	m_EnableOverShoulderCam(false)
{
	m_pPlayerFeatureExtrator.reset(new AllJoints<InputFeature>(g_EnableInputFeatureLocalization));
	m_pKinect = Devices::KinectSensor::GetForCurrentView();
	m_pPlayerArmature = &m_pKinect->Armature();
	if (g_PlayeerBlocks.empty())
	{
		g_PlayeerBlocks.SetArmature(*m_pPlayerArmature);
	}

	auto fReset = std::bind(&PlayerProxy::ResetPlayer, this, placeholders::_1, placeholders::_2);
	m_playerSelector.SetPlayerChangeCallback(fReset);

	auto fFrame = std::bind(&PlayerProxy::StreamPlayerFrame, this, placeholders::_1, placeholders::_2);
	m_playerSelector.SetFrameCallback(fFrame);

	m_playerSelector.Initialize(m_pKinect.get(), TrackedBodySelector::SelectionMode::Closest);

	Register();
	m_IsInitialized = true;
}

PlayerProxy::~PlayerProxy()
{
	Unregister();
	//std::ofstream fout("handpos.txt", std::ofstream::out);

	//fout.close();
}

void PlayerProxy::AddChild(SceneObject* pChild)
{
	SceneObject::AddChild(pChild);
	auto pChara = dynamic_cast<CharacterObject*>(pChild);
	if (pChara)
	{
		m_Controllers.emplace_back();
		auto& controller = m_Controllers.back();
		controller.ID = m_Controllers.size() - 1;
		controller.Initialize(*m_pPlayerArmature, *pChara);
		pChara->SetOpticity(1.0f);

		if (g_DebugLocalMotion)
		{
			g_DebugLocalMotionAction[pChara->Name] = pChara->CurrentActionName();
			pChara->StopAction();
		}

		auto glow = pChara->FirstChildOfType<CharacterGlowParts>();
		if (glow == nullptr)
		{
			glow = new CharacterGlowParts();
			glow->SetEnabled(false);
			pChara->AddChild(glow);
		}
	}
}

void PlayerProxy::SetActiveController(int idx)
{
	std::lock_guard<std::mutex> guard(m_controlMutex);

	if (idx >= 0)
		idx = idx % m_Controllers.size();

	for (auto& c : m_Controllers)
	{
		if (c.ID != idx)
		{
			auto& chara = c.Character();

			if (!g_DebugLocalMotion && !g_DebugLocalMotionAction[chara.Name].empty())
			{
				auto& action = g_DebugLocalMotionAction[chara.Name];
				chara.StartAction(action);
				g_DebugLocalMotionAction[chara.Name] = "";
			}

			chara.SetOpticity(0.5f);
			auto glow = chara.FirstChildOfType<CharacterGlowParts>();
			glow->SetEnabled(false);

			if (c.ID == m_CurrentIdx && m_CurrentIdx != idx)
			{
				chara.SetPosition(c.CMapRefPos);
				chara.SetOrientation(c.CMapRefRot);
				chara.EnabeAutoDisplacement(false);
			}
		}
	}

	if (m_CurrentIdx != idx)
	{
		m_CurrentIdx = idx;
		if (m_CurrentIdx != -1)
		{
			auto& controller = GetController(m_CurrentIdx);
			auto& chara = controller.Character();

			if (!g_DebugLocalMotion && !chara.CurrentActionName().empty())
			{
				g_DebugLocalMotionAction[chara.Name] = chara.CurrentActionName();
				chara.StopAction();
			}

			chara.SetOpticity(1.0f);
			auto glow = chara.FirstChildOfType<CharacterGlowParts>();
			glow->SetEnabled(!g_DebugView);

			controller.MapRefPos = m_playerSelector->PeekFrame()[0].GblTranslation;
			controller.LastPos = controller.MapRefPos;
			controller.CMapRefPos = chara.GetPosition();

			controller.MapRefRot = m_playerSelector->PeekFrame()[0].GblRotation;
			controller.CMapRefRot = chara.GetOrientation();

			chara.EnabeAutoDisplacement(g_UsePersudoPhysicsWalk);
		}
	}
}

float GetConstraitedRotationFromSinVector(Eigen::Matrix3f &Rot, const Eigen::MatrixXf &covXY, int pivot)
{
	//RowVector3f angles;
	//for (int i = 0; i < 3; i++)
	//{
	//	float sin = sinRot[i];
	//	if (sin < -1.0f || sin > 1.0f)
	//		angles[i] = 0;
	//	else
	//		angles[i] = asinf(sin);
	//}

	//DirectX::Matrix4x4 rot = DirectX::XMMatrixRotationRollPitchYaw(angles[0], angles[1], angles[2]);
	//for (int i = 0; i < 3; i++)
	//	for (int j = 0; j < 3; j++)
	//		Rot(i, j) = rot(i, j);

	//return;

	// Assumption on one axis rotation

	//DenseIndex pivot = -1;
	//sinRot.cwiseAbs().minCoeff(&pivot);
	float tanX = (covXY(1, 2) - covXY(2, 1)) / (covXY(1, 1) + covXY(2, 2));
	float tanY = (covXY(2, 0) - covXY(0, 2)) / (covXY(0, 0) + covXY(2, 2));
	float tanZ = (covXY(0, 1) - covXY(1, 0)) / (covXY(0, 0) + covXY(1, 1));
	//assert(sin <= 1.0f && sin >= -1.0f && "sin value must within range [0,1]");

	// there is nothing bad about using positive value of cosine, it ensure the angle set in [-pi/2,pi/2]
	float cosX = 1.0f / sqrt(1 + tanX*tanX);
	float sinX = cosX * tanX;
	float cosY = 1.0f / sqrt(1 + tanY*tanY);
	float sinY = cosY * tanY;
	float cosZ = 1.0f / sqrt(1 + tanZ*tanZ);
	float sinZ = cosZ * tanZ;

	sinX = -sinX;
	sinY = -sinY;
	sinZ = -sinZ;
	Rot.setIdentity();

	//! IMPORTANT, Right-Hand 
	switch (pivot)
	{
	case 0:
		Rot(1, 1) = cosX;
		Rot(1, 2) = -sinX;
		Rot(2, 2) = cosX;
		Rot(2, 1) = sinX;
		break;
	case 1:
		Rot(0, 0) = cosY;
		Rot(0, 2) = sinY;
		Rot(2, 2) = cosY;
		Rot(2, 0) = -sinY;
		break;
	case 2:
		Rot(0, 0) = cosZ;
		Rot(0, 1) = -sinZ;
		Rot(1, 1) = cosZ;
		Rot(1, 0) = sinZ;
		break;
	}
	return atanf(tanX);
}

Matrix3f FindIsometricTransformXY(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y)
{
	assert(X.cols() == Y.cols() && X.rows() == Y.rows() && X.cols() == 3 && "input X,Y dimension disagree");

	auto uX = X.colwise().mean().eval();
	auto uY = Y.colwise().mean().eval();

	//sum(Xi1*Yi1,Xi2*Yi2,Xi3*Yi3)
	MatrixXf covXY = X.transpose() * Y;

	// The one axis rotation matrix
	Matrix3f BestRot;
	float BestScale, bestAng;
	int bestPiv = -1;
	float bestErr = numeric_limits<float>::max();

	for (int pivot = 0; pivot < 3; pivot++)
	{
		Matrix3f Rot;
		float scale = 1.0f;
		float ang = GetConstraitedRotationFromSinVector(Rot, covXY, pivot);
		// the isometric scale factor
		scale = ((X * Rot).array()*Y.array()).sum() / X.cwiseAbs2().sum();
		float err = (X * scale * Rot - Y).cwiseAbs2().sum();
		if (err < bestErr)
		{
			bestPiv = pivot;
			BestRot = Rot;
			BestScale = scale;
			bestErr = err;
			bestAng = ang;
		}
	}

	if (bestPiv == -1)
	{
		cout << "[!] Error , Failed to find isometric transform about control handle" << endl;
	}
	else
	{
		static char xyz[] = "XYZ";
		cout << "Isometric transform found : Scale [" << BestScale << "] , Rotation along axis [" << xyz[bestPiv] << "] for " << bestAng / DirectX::XM_PI << "pi , Error = " << bestErr << endl;
	}

	return BestScale * BestRot;
}

template <typename Iterator>
inline bool next_combination(Iterator first,
	Iterator k,
	Iterator last);

template <typename Iterator>
inline bool next_combination(const Iterator first, Iterator k, const Iterator last)
{
	/* Credits: Thomas Draper */
	// http://stackoverflow.com/a/5097100/8747
	if ((first == last) || (first == k) || (last == k))
		return false;
	Iterator itr1 = first;
	Iterator itr2 = last;
	++itr1;
	if (last == itr1)
		return false;
	itr1 = last;
	--itr1;
	itr1 = k;
	--itr2;
	while (first != itr1)
	{
		if (*--itr1 < *itr2)
		{
			Iterator j = k;
			while (!(*itr1 < *j)) ++j;
			std::iter_swap(itr1, j);
			++itr1;
			++j;
			itr2 = k;
			std::rotate(itr1, j, last);
			while (last != j)
			{
				++j;
				++itr2;
			}
			std::rotate(k, itr2, last);
			return true;
		}
	}
	std::rotate(first, k, last);
	return false;
}

float quadratic_assignment_cost(const MatrixXf& A, const Tensor<float, 4> &C, _In_reads_(A.rows()) DenseIndex* ass)
{
	int n = A.rows();
	float score = 0;
	for (int i = 0; i < n; i++)
	{
		score += A(i, ass[i]);
		for (int j = i + 1; j < n; j++)
		{
			score += C(i, j, ass[i], ass[j]);
		}
	}
	return score;
}


float max_quadratic_assignment(const MatrixXf& A, const Tensor<float, 4> &C, _Out_ std::vector<DenseIndex>& assignment)
{
	auto nx = A.rows(), ny = A.cols();
	assert(ny >= nx);
	vector<DenseIndex> s(std::max(nx, ny));
	for (int i = 0; i < s.size(); i++)
	{
		s[i] = i;
	}

	vector<DenseIndex>  optAss(nx);
	float optScore = std::numeric_limits<float>::min();

	do {
		do {
			float score = quadratic_assignment_cost(A, C, s.data());
			if (score > optScore)
			{
				optAss.assign(s.begin(), s.begin() + nx);
				optScore = score;
			}
			for (auto& i : s)
			{
				cout << i << ' ';
			}
			//cout << ':' << score << endl;
		} while (std::next_permutation(s.begin(), s.begin() + nx));
	} while (next_combination(s.begin(), s.begin() + nx, s.end()));

	assignment = optAss;
	return optScore;
}

// helper functions
void CaculateQuadraticDistanceMatrix(Eigen::Tensor<float, 4> &C, const ClipInfo& iclip, const ClipInfo& cclip);

void CreateControlBinding(CharacterController & controller, const InputClipInfo& iclip);

void SetIdentity(Causality::PcaCcaMap & map, const Eigen::Index &rank);

float max_cols_assignment(Eigen::MatrixXf & A, Eigen::MatrixXf & Scor, std::vector<ptrdiff_t> &matching);

int PlayerProxy::MapCharacterByLatestMotion()
{
	static const size_t FilterStregth = 4U; //Applies 4 iterates of Laplicaian filter
	static const size_t CropMargin = 1 * SAMPLE_RATE; // Ignore most recent 1 sec
	static const float Cutoff_Correlation = 0.5f;
	time_seconds InputDuration(8.5333333);

	auto& player = *m_playerSelector;
	static ofstream flog;
	if (g_EnableDebugLogging)
		flog.open("Xlog.txt");

	// so that it's 256 rows, good for fft
	MatrixXf X = GetPlayerFeatureMatrix(InputDuration);
	//? WARNNING! GetPlayerFeatureMatrix() is ROW Major!!!
	//? But we have convert it into Column Major! Yes and No!!!
	auto N = X.rows();
	auto K = X.cols();

	if (N*K == 0) return -1; // Feature Matrix is not ready yet



	string timeStr;
	path LoggingDir;
	{
		stringstream ss;
		std::time_t timeStemp = std::time(nullptr);
		ss << put_time(std::localtime(&timeStemp), "%Y_%m_%d_%H_%I_%S");
		timeStr = ss.str();
		LoggingDir = g_LogRootDir / timeStr;
		if (g_EnableRecordLogging)
		{
			if (!boost::filesystem::exists(LoggingDir))
			{
				bool result = boost::filesystem::create_directory(LoggingDir);

			}
			assert(boost::filesystem::is_directory(LoggingDir));
		}
	}

	cout << "X : min = " << X.minCoeff() << " , max = " << X.maxCoeff() << endl;



	MatrixXcf Xf(N, K);

	concurrency::parallel_for(0, (int)K, [&Xf, &X](auto i)
		//for (size_t i = 0; i < K; i++)
	{
		FFT<float> fft;
		fft.fwd(Xf.col(i).data(), X.col(i).data(), Xf.rows());
	}
	);

	cout << "X : min = " << X.minCoeff() << " , max = " << X.maxCoeff() << endl;
	//flog << X << endl;
	//flog.close();

	MatrixXf Xs = Xf.cwiseAbs2();

	//? How to normalize enery term between position and angular? express arg as arc length???
	RowVectorXf Eu3 = Xs.middleRows(MinHz, MaxHz).colwise().sum();
	RowVectorXf Euj = MatrixXf::Map(Eu3.data(), JointDemension, K / JointDemension).colwise().sum();

	int idx;
	VectorXf Ea = Xs.middleRows(MinHz - 1, MaxHz + 2).rowwise().sum();
	cout << Ea.transpose() << endl;

	Ea.segment(1, Ea.size() - 2).maxCoeff(&idx); // Frequency 3 - 30
	++idx;

	auto Ex = Ea.middleRows<3>(idx - 1);
	idx += MinHz;
	cout << Ex.transpose() << endl;

	Vector3f Ix = { idx - 1.0f, (float)idx, idx + 1.0f };
	float peekFreq = Ex.dot(Ix) / Ex.sum();

	int T = (int)ceil(N / peekFreq);
	//! Retive Primary Time Period and Peek Frequency

	//! Crop and Resample the sequence
	int nT = max(5, (int)(N / T));

	//! Smooth the input 
	laplacian_smooth(X, 0.8f, FilterStregth);

	 cublic_bezier_resample(Xs,
		X.middleRows(N - (2 * T + CropMargin + 1), T * 2),
		CLIP_FRAME_COUNT * 2,
		Eigen::CloseLoop);

	if (flog.is_open())
	{
		flog << Xs << endl;
		flog.close();
	}

	T = CLIP_FRAME_COUNT; // Since we have resampled , Time period "T" is now equals to StretchedSampleCount
	int Ti = 1; // 2
	int Ts = T / Ti;

	int Ju = g_PlayeerBlocks.size();


	InputClipInfo iclip(g_PlayeerBlocks);
	iclip.Period = T;
	iclip.TemproalSampleInterval = Ti;
	iclip.Xbs.resize(Ju);

	// Spatial Traits
	Eigen::MatrixXf Xsp(3, Ju);
	//vector<vector<MatrixXf>> RawXs(Ts);
	vector<vector<MeanThinQr<MatrixXf>>> QrXs(Ts);
	vector<Pca<MatrixXf>> PcaXs(Ju);
	for (size_t i = 0; i < Ts; i++)
	{
		//RawXs[i].resize(Ju);
		QrXs[i].resize(Ju);
	}

	VectorXf Eub(Ju);

	// Fuck it... 11,250,000 times of CCA(SVD) computation every guess
	// 1,406,250 SVD for 5x5 setup, much more reasonable!
	//for (auto i : Juk) //? <= 25 joint per USER?
	concurrency::parallel_for(0, (int)Ju, [&](auto i)
	{
		const auto& block = *g_PlayeerBlocks[i];
		auto stCol = block.AccumulatedJointCount * JointDemension;
		auto cols = block.Joints.size() * JointDemension;

		//? How to normalize muilti-joint block's energy???? 
		//? Use max or sum ??? - max seems like a better idea!
		Eub(i) = Euj.middleCols(stCol / JointDemension, cols / JointDemension).maxCoeff();

		auto Xk = Xs.middleCols(stCol, cols);

		//? Spatial Traits Calculation!!!
		Xsp.col(i) = Xk.rightCols<3>().colwise().mean().transpose();

		auto& pca = PcaXs[i];

		pca.compute(Xk.rightCols<3>(), true);
		auto d = pca.reducedRank(g_PlayerPcaCutoff);//pca.reducedRank(PcaCutoff);
		auto coords = pca.coordinates(d);
		iclip.Xbs[i] = Xk.rightCols<3>();

		for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
		{
			auto Xp = Xk.middleRows(T - phi, T);
			auto Xcords = coords.middleRows(T - phi, T);

			//! Important!!! Using Xp.rightCols<3>() instead of Xp for QrXs!!!
			//pca.compute(Xp, true);
			QrXs[phi / Ti][i].compute(Xcords, true);
		}
	}
	);

	//for (auto& block : boost::make_iterator_range(g_PlayeerBlocks.rbegin(),g_PlayeerBlocks.rend()))
	//{
	//	if (!g_EnableInputFeatureLocalization && block->parent() != nullptr)
	//	{
	//		Xsp.col(block->Index) -= Xsp.col(block->parent()->Index);
	//	}
	//}

	Xsp.colwise().normalize();
	//// Spatial Traits
	//Eigen::Matrix<float, -1, -1> Xsp(3, Ju);
	//{
	//	AnimationAnalyzer anlyzer(g_PlayeerBlocks);
	//	anlyzer.ComputeSpatialTraits()
	//}

	Eub = Eub.cwiseSqrt();
	Eub /= Eub.maxCoeff();

	vector<int> Juk;

	for (size_t i = 0; i < Ju; i++)
	{
		if (Eub(i) > g_PlayerEnergyCutoff)
		{
			Xsp.col(Juk.size()) = Xsp.col(i);
			Eub(Juk.size()) = Eub(i);
			Juk.push_back(i);
		}
	}
	Xsp.conservativeResize(Xsp.rows(), Juk.size());
	Eub.conservativeResize(Juk.size());
	MatrixXf XDir(2 * T * 3, Juk.size());
	for (size_t i = 0; i < Juk.size(); i++)
	{
		static_assert(JointDemension == 3, "This code segments assums Joint Feature is X-Y-Z relative position");
		auto& block = *g_PlayeerBlocks[Juk[i]];
		auto stCol = block.AccumulatedJointCount * JointDemension;
		auto cols = block.Joints.size() * JointDemension;
		stCol = stCol + cols - JointDemension;

		auto vs = Matrix<float, 3, -1>::Map(XDir.col(i).data(), 3, 2 * T);
		vs = Xs.middleCols<JointDemension>(stCol).transpose();// 2*t x 3
		vs.colwise().normalize();
	}

	Array<RowVector3f, Dynamic, Dynamic> XpMean(Juk.size(), Juk.size());
	Array<Matrix3f, Dynamic, Dynamic> XpCov(Juk.size(), Juk.size());

	MatrixXf Xij(Xs.rows(), 3);
	for (int i = 0; i < Juk.size(); i++)
	{
		auto& block = *g_PlayeerBlocks[Juk[i]];
		auto stCol = block.AccumulatedJointCount * JointDemension;
		auto cols = block.Joints.size() * JointDemension;
		stCol = stCol + cols - JointDemension;
		auto Xi = Xs.middleCols<JointDemension>(stCol);

		for (int j = i + 1; j < Juk.size(); j++)
		{
			auto& block = *g_PlayeerBlocks[Juk[j]];
			auto stCol = block.AccumulatedJointCount * JointDemension;
			auto cols = block.Joints.size() * JointDemension;
			stCol = stCol + cols - JointDemension;
			auto Xj = Xs.middleCols<JointDemension>(stCol);

			Xij = Xi - Xj;
			XpMean(i, j) = Xij.colwise().mean();
			Xij.rowwise() -= XpMean(i, j);
			XpCov(i, j).noalias() = Xij.transpose() * Xij;
		}
	}



	//? HACK Juk Jck! Pre-reduce DOF
	//size_t Juk[] = { JointType_Head,JointType_ElbowLeft,JointType_ElbowRight,JointType_KneeLeft,JointType_KneeRight };

	//size_t Jck[] = { 8,12,17,22,35,40,45,50,54};
	//for (auto& jc : Jck)
	//	jc -= 1;

	////? HACK Ju
	//Ju = std::size(Juk);

	if (g_EnableRecordLogging)
	{
		ofstream fout((LoggingDir / "X.csv").wstring());
		fout << X.format(CSVFormat);
		fout.close();
	}

	iclip.ActiveParts = std::move(Juk);
	iclip.Pcas = std::move(PcaXs);
	iclip.qrXs = std::move(QrXs);
	iclip.PvDifMean = std::move(XpMean);
	iclip.PvDifCov = std::move(XpCov);
	iclip.Pvs = std::move(XDir);
	iclip.Eb = std::move(Eub);

	for (auto& controller : m_Controllers)			//? <= 5 character
	{
		if (!controller.IsReady)
			continue;
		CreateControlBinding(controller, iclip);
	}

	CharacterController* pControl = nullptr;
	for (auto& con : m_Controllers)
	{
		if (!pControl || con.CharacterScore > pControl->CharacterScore)
			pControl = &con;
	}
	if (!pControl) return -1;
	SetActiveController(pControl->ID);

	return pControl->ID;
}

void CreateControlBinding(CharacterController & controller, const InputClipInfo& iclip)
{
	if (!controller.IsReady)
		return;

	int T = iclip.Period;
	const std::vector<int> &Juk = iclip.ActiveParts;
	int Ti = iclip.TemproalSampleInterval;
	int Ts = T / Ti;
	//const Eigen::MatrixXf &Xsp = iclip.Xsp;
	//const Eigen::MatrixXf &Xs = iclip.Xs;
	const auto & XDir = iclip.Pvs;
	const auto & PcaXs = iclip.Pcas;
	const auto & Eub = iclip.Eb;

	auto& character = controller.Character();
	auto& chraraBlocks = character.Behavier().Blocks();
	size_t Jc = chraraBlocks.size();
	auto& clips = character.Behavier().Clips();

	controller.CharacterScore = numeric_limits<float>::min();
	//auto& anim = character.Behavier()[DefaultAnimationSet];
	auto& anim = *character.CurrentAction();
	//for (auto& anim : clips)	//? <= 5 animation per character
	{
		auto& cclip = controller.GetAnimationInfo(anim.Name);
		if (&cclip == nullptr)
			return;

		// Caculate Ecb, Energy of Character Blocks
		auto Ecb = cclip.Eb;
		MatrixXf Ecb3 = cclip.Eb3;
		// Independent Active blocks only
		vector<int> Jck;
		for (size_t i = 0; i < Jc; i++)
		{
			if (Ecb(i) > cclip.ActiveEnergyThreshold)
			{
				Ecb(Jck.size()) = Ecb(i);
				Ecb3.col(Jck.size()) = Ecb3.col(i);
				Jck.push_back(i);
			}
		}
		Ecb.conservativeResize(Jck.size());
		Ecb3.conservativeResize(3, Jck.size());

		// Alternate Spatial traits
		Matrix<float, -1, -1> Csp(3, Jck.size());
		MatrixXf Csp2(3 * T, Jck.size());

		for (size_t i = 0; i < Jck.size(); i++)
		{
			Csp.col(i) = cclip.Sp.block<3, 1>(0, Jck[i]);
			Csp2.col(i) = cclip.Pvs.col(Jck[i]);

			// Normalize Pvs to caculate directions
			auto cpvs = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(Csp2.col(i).data(), CLIP_FRAME_COUNT, 3);
			cpvs.rowwise().normalize();

			RowVector3f E3 = cclip.Eb3.col(Jck[i]).transpose();

			// d(X)*d(P) >= h/2, thus, more energy in movement means less important to measure it's position
			E3 /= E3.maxCoeff();
			E3 = (E3.array().cwiseSqrt() + 0.1f).cwiseInverse();
			E3 /= E3.maxCoeff();
			Ecb3.col(i) = E3.transpose();
			cpvs.array() *= E3.replicate(T, 1).array();
		}

		// Score for spatial traits !!!
		//MatrixXf Asp = Xsp.transpose() * Csp;

		// Memery allocation
		auto CoRSize = Juk.size() + Jck.size();

		std::vector<MatrixXf> Ra(Ts), Rm(Ts), Rs(Ts);

		for (int i = 0; i < Ts; i++)
		{
			Ra[i].setZero(Juk.size(), Jck.size());
			Rs[i].setZero(Juk.size(), Jck.size());
			Rm[i].setZero(Juk.size(), Jck.size());
		}

		Tensor<float, 4> C((int)Juk.size(), (int)Juk.size(), (int)Jck.size(), (int)Jck.size());

		CaculateQuadraticDistanceMatrix(C, iclip, cclip);
		//MatrixXf CoR;//(Ju + Jc, Ju + Jc);
		//CoR.setZero(CoRSize, CoRSize);


		VectorXf mScores(Ts);
		vector<vector<DenseIndex>> mMatchings(Ts);
		//DenseIndex phi = 0;

		//concurrency::parallel_for(0, (int)T, Ti, [&](auto phi)
		for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
		{
			auto phidx = phi / Ti;

			//auto Xp = Xs.middleRows(T - phi, T);


			auto& Mcor = Rm[phidx];//.topLeftCorner(Juk.size(), Jck.size());
			auto& Scor = Rs[phidx];

			auto Xsp2 = XDir.middleRows((T - phi) * 3, T * 3);


			for (int i = 0; i < Juk.size(); ++i)
			{
				int uid = Juk[i];
				auto& qrX = iclip.QrXs(phi / Ti,uid);
				auto& rawX = iclip.RawXs(phi / Ti,uid);

				for (int j = 0; j < Jck.size(); j++)
				{
					int cid = Jck[j];
					//? Here we uses Qrs or PvQrs is important!!!
					auto& qrY = cclip.PvQrs[cid];
					auto rawY = MatrixXf::Map(cclip.Pvs.col(cid).data(), 3, CLIP_FRAME_COUNT).transpose();

					float r = 0;

					if (qrX.rank() * qrY.rank() != 0)
					{
						Cca<float> cca;
						cca.computeFromQr(qrX, qrY, false);	//? it's <= 3x3 SVD inside
						r = cca.correlaltions().minCoeff();

						//float alpha = rawY.cwiseAbs2().sum() / rawX.cwiseAbs2().sum();
						//float err = (rawY - rawY * alpha).cwiseAbs2().sum();
					}

					Mcor(i, j) = r;
					Scor(i, j) = ((Xsp2.col(i) - Csp2.col(j)).array() * Ecb3.col(j).replicate(T, 1).array()).cwiseAbs2().sum();
				}
			}
			// a = (a > cutoff) ? a : -1.0f
			//Mcor = (Mcor.array() > Cutoff_Correlation).select(Mcor, -1.0f);

			//Scor = Xsp2.transpose() * Csp2;

			Scor /= (float)T;
			Scor = (-(Scor.array() / (DirectX::XM_PI / 6)).cwiseAbs2()).exp();
			MatrixXf& A = Ra[phidx];
			// Cutoff the none-correlated match
			//A *= alpha;

			// Here Eub and Ecb is Kinetic Energy, it should only affect the motion correlation term
			Mcor.array() *= Eub.transpose().array().replicate(1, A.cols());
			//Mcor.array() *= Ecb.array().replicate(A.rows(), 1);

			A = Mcor * g_BlendWeight + Scor;

			//CoR.topLeftCorner(Ju, Jc) = A;

			//auto matching = max_weight_bipartite_matching(A.transpose());
			////auto score = matching_cost(A, matching);
			//auto score = matching_cost(A.transpose(), matching);// / A.cols();

			//auto score = quadratic_assignment_cost(A,C, matching);// / A.cols();

			vector<DenseIndex> matching(A.cols());

			float score = max_quadratic_assignment(A, C, matching);

			//float score = max_cols_assignment(A, Scor, matching);

			mScores[phidx] = score;
			mMatchings[phidx] = matching;
		}
		//);

		float maxScore = numeric_limits<float>::min();
		DenseIndex maxPhi = -1;
		vector<DenseIndex> maxMatching;
		for (DenseIndex phi = 0; phi < T; phi += Ti)
		{
			auto phidx = phi / Ti;
			auto score = mScores[phidx];
			if (score > maxScore)
			{
				maxPhi = phi;
				maxMatching = mMatchings[phidx];
				maxScore = score;
			}
		}

		//cout << "Scores over Phi : \n" << mScores << endl;

		cclip.Score = maxScore;
		cclip.Matching = maxMatching;
		cclip.Phi = maxPhi;
		if (maxPhi > -1)
		{
			cclip.Ra = Ra[maxPhi];
			cclip.Rk = Rm[maxPhi];
			cclip.Rs = Rs[maxPhi];
		}

		cout << "=============================================" << endl;
		cout << "Best assignment for " << controller.Character().Name << " : " << anim.Name << endl;
		cout << "Scores : " << maxScore << endl;

		cout << "*********************************************" << endl;
		cout << "Human Skeleton Blocks : " << endl;
		for (auto i : Juk)
		{
			const auto& blX = *g_PlayeerBlocks[i];
			cout << "Block " << i << " = {";
			for (auto pJoint : blX.Joints)
			{
				cout << pJoint->Name << ", ";
			}
			cout << "\b\b}" << endl;
		}

		cout << "*********************************************" << endl;
		cout << "Character " << controller.Character().Name << "'s Skeleton Blocks : " << endl;

		for (auto& i : Jck)
		{
			const auto& blY = *controller.Character().Behavier().Blocks()[i];
			cout << "Block " << i << " = {";
			for (auto pJoint : blY.Joints)
			{
				cout << pJoint->Name << ", ";
			}
			cout << "\b\b}" << endl;
		}
		cout << "*********************************************" << endl;

		// Setup Binding from matching
		if (maxPhi > -1)//controller.CharacterScore < maxScore)
		{
			//cout << "New best : " << anim.Name << endl;
			controller.CharacterScore = std::max(maxScore, controller.CharacterScore);

			auto pBinding = new RBFInterpolationTransform(&controller.GetClipInfos(),
				&g_PlayeerBlocks,
				&controller.Character().Behavier().Blocks());

			//auto pBinding = new BlockizedCcaArmatureTransform(&g_PlayeerBlocks, &controller.Character().Behavier().Blocks());
			//pBinding->pInputExtractor.reset(new AllJoints<InputFeature>(g_EnableInputFeatureLocalization));
			//pBinding->pOutputExtractor.reset(new AllJoints<CharacterFeature>(false));

			cclip.pLocalBinding.reset(pBinding);

			for (int i = 0; i < Juk.size(); ++i)
				//for (int j = 0; j < Jck.size(); ++j)
			{
				//DenseIndex Jx = maxMatching[j], Jy = j;
				DenseIndex Jx = i, Jy = maxMatching[i];
				if (Jy == -1 || Jy >= Jc || Jx == -1 || Jx >= Jc) continue;
				Jx = Juk[Jx]; Jy = Jck[Jy];

				const auto& blX = *g_PlayeerBlocks[Jx];
				const auto& blY = *controller.Character().Behavier().Blocks()[Jy];

				//cout << pPlayerArmature->at(Jx)->Name() << " ==> " << controller.Character().Armature()[Jy]->Name() << endl;

				auto& rawX = iclip.RawXs(maxPhi / Ti,Jx);
				auto rawY = MatrixXf::Map(cclip.Pvs.col(Jy).data(), 3, CLIP_FRAME_COUNT).transpose();

				cout << '{';
				for (auto pJoint : blX.Joints)
				{
					cout << pJoint->Name << ", ";
				}
				cout << "\b\b} ==> {";
				for (auto pJoint : blY.Joints)
				{
					cout << pJoint->Name << ", ";
				}
				cout << "\b\b}" << endl;

				// populate map
				pBinding->Maps.emplace_back();
				auto& map = pBinding->Maps.back();
				map.Jx = Jx; map.Jy = Jy;

				auto rank = rawX.cols();
				SetIdentity(map, rank);

				if (g_PartAssignmentTransform == PAT_CCA)
				{
					//? Again!!! PvQrs vs Qrs
					auto& qrX = iclip.QrXs(maxPhi / Ti,Jx);
					auto& pcaX = PcaXs[Jx];

					auto& qrY = cclip.PvQrs[Jy];
					auto& pcaY = cclip.PvPcas[Jy];
					//auto& qrY = cclip.Qrs[Jy];
					//auto& pcaY = cclip.Pcas[Jy];

					Cca<float> cca;
					cca.computeFromQr(qrX, qrY, true);

					if (cca.rank() <= 0)
						continue;

					map.A = cca.matrixA();
					map.B = cca.matrixB();
					map.uX = qrX.mean();
					map.uY = qrY.mean();
					map.uXpca = pcaX.mean();
					map.uYpca = pcaY.mean();
					map.pcX = pcaX.components(qrX.cols());
					map.pcY = pcaY.components(qrY.cols());
					if (cca.rank() == qrY.cols()) // d == dY
					{
						map.useInvB = true;
						map.invB = map.B.inverse();
					}
					else
					{
						map.useInvB = false;
						map.svdBt = JacobiSVD<MatrixXf>(map.B.transpose(), ComputeThinU | ComputeThinV);;
					}
					//map.TransformMatrix();
				}
				else if (g_PartAssignmentTransform == PAT_OneAxisRotation)
				{
					//RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
					//float err = (rawY - rawX * alpha.asDiagonal()).cwiseAbs2().sum();

					auto Transf = FindIsometricTransformXY(rawX, rawY);
					auto rank = rawX.cols();

					map.A = Transf;
				}
				else if (g_PartAssignmentTransform == PAT_AnisometricScale)
				{
					RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
					float err = (rawY - rawY * alpha.asDiagonal()).cwiseAbs2().sum();
					map.A = alpha.asDiagonal();
				}
				else if (g_PartAssignmentTransform == PAT_RST)
				{
					RowVectorXf uX = rawX.colwise().mean();
					RowVectorXf uY = rawY.colwise().mean();
					MatrixXf _X = rawX - uX.replicate(rawX.rows(), 1);
					MatrixXf _Y = rawY - uY.replicate(rawY.rows(), 1);
					float unis = sqrtf(_Y.cwiseAbs2().sum() / _X.cwiseAbs2().sum());
					RowVectorXf alpha = (_Y.cwiseAbs2().colwise().sum().array() / _X.cwiseAbs2().colwise().sum().array()).cwiseSqrt() / unis;

					alpha = alpha.cwiseMax(0.5f).cwiseMin(1.5f) * unis;

					float err = (_Y - _X * alpha.asDiagonal()).cwiseAbs2().sum();

					alpha[2] = -alpha[2];
					float flipErr = (_Y - _X * alpha.asDiagonal()).cwiseAbs2().sum();
					if (err < flipErr)
					{
						alpha[2] = -alpha[2];
					}



					auto rank = rawX.cols();

					map.A = alpha.asDiagonal();
					map.uX = uX;
					map.uY = uY;
				}
			}
		}
	} // Animation clip scope

	  //auto pBinding = new BlockizedCcaArmatureTransform(&g_PlayeerBlocks, &controller.Character().Behavier().Blocks());
	  //pBinding->pInputExtractor.reset(new AllJoints<InputFeature>(g_EnableInputFeatureLocalization));
	  //pBinding->pOutputExtractor.reset(new AllJoints<CharacterFeature>(false));

	auto pBinding = new RBFInterpolationTransform(&controller.GetClipInfos(),
		&g_PlayeerBlocks,
		&controller.Character().Behavier().Blocks());

	controller.SetBinding(pBinding);

	auto clipinfos = controller.GetClipInfos() | adaptors::map_values;
	auto maxClip = std::max_element(BEGIN_TO_END(clipinfos), [](const ClipInfo* lhs, const ClipInfo *rhs) {
		return (rhs != nullptr) && (lhs == nullptr || lhs->Score < rhs->Score);
	});

	auto pCcaBinding = dynamic_cast<CcaArmatureTransform*>((*maxClip)->pLocalBinding.get());
	if (pCcaBinding)
		pBinding->Maps = pCcaBinding->Maps;

	if (g_EnableDependentControl)
	{
		for (auto& pBlock : chraraBlocks)
		{
			auto& block = *pBlock;
			//if (block.Index == 0)
			//	continue;
			if (block.ActiveActionCount == 0 && block.SubActiveActionCount > 0)
			{
				pBinding->Maps.emplace_back(block.PdCca);
			}
		}
	}

	//VectorXf JuScore(Juk.size());
	//JuScore.setZero();

	//cout << endl;
	//cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

	//for (auto& pBlock : controller.Character().Behavier().Blocks())
	//{
	//	auto bid = pBlock->Index;
	//	if (pBlock->ActiveActionCount > 0)
	//	{
	//		float bestScore = g_MatchAccepetanceThreshold;
	//		PcaCcaMap* pMap = nullptr;
	//		string bestAction = "<empty>";

	//		cout << "{";
	//		for (auto pJoint : pBlock->Joints)
	//		{
	//			cout << pJoint->Name() << ", ";
	//		}
	//		cout << "\b\b} <<== {";
	//		for (const auto& action : pBlock->ActiveActions)
	//		{
	//			auto& clipinfo = controller.GetAnimationInfo(action);
	//			if (clipinfo.Phi == -1) continue;
	//			auto jyid = std::find(BEGIN_TO_END(clipinfo.ActiveParts), bid) - clipinfo.ActiveParts.begin();
	//			auto jxid = clipinfo.Matching[jyid];

	//			if (jxid == -1) continue;
	//			auto score = clipinfo.Rs(jxid, jyid);

	//			cout << "(" << action << " Rs=" << setprecision(3) << score << ", Rk=" << clipinfo.Rk(jxid, jyid) << "),";


	//			if (score > bestScore)
	//			{
	//				auto itr = std::find_if(BEGIN_TO_END(clipinfo.pLocalBinding->Maps), [bid](const auto& map)
	//				{ return map.Jy == bid;});

	//				// This bind is discard due to threshold cutoff
	//				if (itr == clipinfo.pLocalBinding->Maps.end())
	//					continue;

	//				bestScore = score;
	//				pMap = &(*itr);
	//				bestAction = action;

	//				JuScore[jxid] = std::max(JuScore[jxid], score);
	//			}
	//		}

	//		if (pMap != nullptr)
	//		{
	//			pBinding->Maps.push_back(*pMap);
	//		}
	//		cout << "\b <<== [" << bestAction << "]" << endl;
	//	}
	//}

	//cout << endl;

	//cout << "=========================================================" << endl;
	//JuScore.array() *= Eub.array();
	//controller.CharacterScore = JuScore.sum();
	//cout << "Total score = " << controller.CharacterScore << endl;
	//cout << "=========================================================" << endl;
	//cout << endl;
}

void CaculateQuadraticDistanceMatrix(Eigen::Tensor<float, 4> &C,const ClipInfo& iclip, const ClipInfo& cclip)
{
	C.setZero();

	auto& Juk = iclip.ActiveParts;
	auto& Jck = cclip.ActiveParts;
	//const std::vector<int> &Juk, const std::vector<int> &Jck, const Eigen::Array<Eigen::RowVector3f, -1, -1> &XpMean, const Eigen::Array<Eigen::Matrix3f, -1, -1> &XpCov, const Causality::CharacterController & controller);

	for (int i = 0; i < Juk.size(); i++)
	{
		for (int j = i + 1; j < Juk.size(); j++)
		{
			for (int si = 0; si < Jck.size(); si++)
			{
				for (int sj = si + 1; sj < Jck.size(); sj++)
				{
					auto xu = iclip.XpvMean(i, j);
					auto xc = cclip.XpvMean(si, sj);
					auto &cu = iclip.XpvCov(i, j);
					auto &cc = cclip.XpvCov(si, sj);

					float val = 0;
					if (xu.norm() > 0.1f && xc.norm() > 0.1f)
					{

						//auto edim = (-cu.diagonal() - cc.diagonal()).array().exp().eval();
						RowVector3f _x = xu.array() * xc.array();
						_x /= xu.norm() * xc.norm();
						val = (_x.array() /** edim.transpose()*/).sum();
						C(i, j, si, sj) = val;
						C(j, i, sj, si) = val;
						C(i, j, sj, si) = -val;
						C(j, i, si, sj) = -val;
					}
					else if (xu.norm() + xc.norm() > 0.1f)
					{
						val = -1.0f;
						C(i, j, si, sj) = val;
						C(j, i, sj, si) = val;
						C(i, j, sj, si) = val;
						C(j, i, si, sj) = val;
					}
					else
					{
						C(i, j, si, sj) = 0;
						C(j, i, sj, si) = 0;
						C(i, j, sj, si) = 0;
						C(j, i, si, sj) = 0;

					}

				}
			}
		}
	}

	cout << C << endl;

}

float max_cols_assignment(Eigen::MatrixXf & A, Eigen::MatrixXf & Scor, std::vector<ptrdiff_t> &matching)
{
	VectorXf XScore(A.rows());
	VectorXi XCount(A.rows());
	XCount.setZero();
	XScore.setZero();
	for (int k = 0; k < A.cols(); k++)
	{
		DenseIndex jx;
		Scor.col(k).maxCoeff(&jx);
		auto score = A(jx, k);
		if (score < g_MatchAccepetanceThreshold) // Reject the match if it's less than a threshold
			matching[k] = -1;
		else
		{
			matching[k] = jx;
			XScore(jx) += score;
			++XCount(jx);
		}
	}
	//XScore.array() /= XCount.array().cast<float>();
	return XScore.sum();
}

void SetIdentity(Causality::PcaCcaMap & map, const Eigen::Index &rank)
{
	map.A.setIdentity(rank, rank);
	map.B.setIdentity(rank, rank);
	map.uX.setZero(rank);
	map.uY.setZero(rank);
	map.uXpca.setZero(rank);
	map.uYpca.setZero(rank);
	map.pcX.setIdentity(rank, rank);
	map.pcY.setIdentity(rank, rank);
	map.useInvB = true;
	map.invB.setIdentity(rank, rank);
}

//void PlayerProxy::LocalizePlayerFeature(Eigen::MatrixXf &X)
//{
//	for (const auto &pBlock : g_PlayeerBlocks) //? <= 25 joint per USER?
//	{
//		auto& block = *pBlock;
//		if (block.Joints[0]->is_root()) continue;
//		auto refId = block.Joints[0]->parent()->ID;
//		auto refCols = X.middleCols<JointDemension>(refId*JointDemension);
//		for (auto& pJ : block.Joints)
//		{
//			auto jid = pJ->ID;
//			X.middleCols<JointDemension>(jid*JointDemension) -= refCols;
//		}
//	}
//}

void PlayerProxy::ClearPlayerFeatureBuffer()
{
	std::lock_guard<std::mutex> guard(m_BufferMutex);
	FeatureBuffer.clear();
}

// Character Map State

bool PlayerProxy::IsMapped() const { return m_CurrentIdx >= 0; }

const CharacterController & PlayerProxy::CurrentController() const {
	for (auto& c : m_Controllers)
	{
		if (c.ID == m_CurrentIdx)
			return c;
	}
}

CharacterController & PlayerProxy::CurrentController() {
	for (auto& c : m_Controllers)
	{
		if (c.ID == m_CurrentIdx)
			return c;
	}
}

const CharacterController & PlayerProxy::GetController(int state) const {
	for (auto& c : m_Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

CharacterController & PlayerProxy::GetController(int state)
{
	for (auto& c : m_Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

void PlayerProxy::OnKeyUp(const KeyboardEventArgs & e)
{
	if (e.Key == VK_OEM_PERIOD || e.Key == '.' || e.Key == '>')
	{
		SetActiveController(m_CurrentIdx + 1);
	}
	else if (e.Key == VK_OEM_COMMA || e.Key == ',' || e.Key == '<')
	{
		SetActiveController(m_CurrentIdx - 1);
	}
	else if (e.Key == 'L')
	{
		// this behavier should not change in mapped mode
		if (IsMapped()) return;

		g_DebugLocalMotion = !g_DebugLocalMotion;
		if (g_DebugLocalMotion)
		{
			for (auto& controller : m_Controllers)
			{
				auto& chara = controller.Character();
				g_DebugLocalMotionAction[chara.Name] = chara.CurrentActionName();
				chara.StopAction();
			}
		}
		else
		{
			for (auto& controller : m_Controllers)
			{
				auto& chara = controller.Character();
				auto& action = g_DebugLocalMotionAction[chara.Name];
				if (!action.empty())
					chara.StartAction(action);
				g_DebugLocalMotionAction[chara.Name] = "";
			}
		}
	}
	else if (e.Key == VK_UP || e.Key == VK_DOWN)
	{
		for (auto& controller : m_Controllers)
		{
			auto& chara = controller.Character();
			auto& clips = chara.Behavier().Clips();
			auto& idx = controller.CurrentActionIndex;
			if (e.Key == VK_UP)
				idx = (idx + 1) % clips.size();
			else
				idx = idx == 0 ? clips.size() - 1 : idx - 1;

			if (g_DebugLocalMotion)
				g_DebugLocalMotionAction[chara.Name] = clips[idx].Name;
			else
				chara.StartAction(clips[idx].Name);
		}
	}
	else if (e.Key == 'P')
	{
		g_EnableDependentControl = !g_EnableDependentControl;
		cout << "Enable Dependency Control = " << g_EnableDependentControl << endl;
	}
	else if (e.Key == 'C')
	{
		m_EnableOverShoulderCam = !m_EnableOverShoulderCam;
		//g_UsePersudoPhysicsWalk = m_EnableOverShoulderCam;
		cout << "Over Shoulder Camera Mode = " << m_EnableOverShoulderCam << endl;
		cout << "Persudo-Physics Walk = " << g_UsePersudoPhysicsWalk << endl;
	}
	else if (e.Key == 'M')
	{
		g_MirrowInputX = !g_MirrowInputX;
		cout << "Kinect Input Mirrowing = " << g_MirrowInputX << endl;
		m_pKinect->EnableMirrowing(g_MirrowInputX);
	}
	else if (e.Key == VK_NUMPAD1)
	{
		g_NoiseInterpolation[0] -= 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD3)
	{
		g_NoiseInterpolation[0] += 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD2)
	{
		g_NoiseInterpolation[0] = 1.0f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD4)
	{
		g_NoiseInterpolation[1] -= 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD6)
	{
		g_NoiseInterpolation[1] += 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD5)
	{
		g_NoiseInterpolation[1] = 1.0f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD7)
	{
		g_NoiseInterpolation[2] -= 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD9)
	{
		g_NoiseInterpolation[2] += 0.1f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
	else if (e.Key == VK_NUMPAD8)
	{
		g_NoiseInterpolation[2] = 1.0f;
		cout << "Local Motion Sythesis Jaming = " << g_NoiseInterpolation << endl;
	}
}

void PlayerProxy::OnKeyDown(const KeyboardEventArgs & e)
{
}

//x DON"T USE THIS
pair<float, float> PlayerProxy::ExtractUserMotionPeriod()
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
		JointReductedSpecturm.block<1, HzWidth>(j, MinHz).maxCoeff(&idx);
		JointPeriod[j] = 15.0f / (float)(idx + 3); // 450 frame/(30 frame/s)
	}

	auto overallSpectrum = JointReductedSpecturm.colwise().sum();

	int idx;
	overallSpectrum.block<1, HzWidth>(1, MinHz).maxCoeff(&idx);
	return make_pair(.0f, idx);
}

RenderFlags Causality::PlayerProxy::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

//void PlayerProxy::PrintFrameBuffer(int No)
//{
//	int flag = ofstream::out | (No == 1 ? 0 : ofstream::app);
//	ofstream fout("handpos.csv", flag);
//
//	//Matrix<float, Dynamic, FeatureDim> X(BufferFramesCount);
//	for (auto& frame : FrameBuffer)
//	{
//		auto Xj = HumanFeatureFromFrame(frame);
//
//		for (size_t i = 0; i < Xj.size() - 1; i++)
//		{
//			fout << Xj(i) << ',';
//		}
//		fout << Xj(Xj.size() - 1) << endl;
//
//		//for (auto& bone : frame)
//		//{
//		//	const auto& pos = bone.GblTranslation;
//		//	//DirectX::XMVECTOR quat = bone.LclRotation;
//		//	//Vector3 pos = DirectX::XMQuaternionLn(quat);
//		//	fout << pos.x << ',' << pos.y << ',' << pos.z << ',';
//		//}
//		//fout << endl;
//	}
//	fout.close();
//}

void AddNoise(BoneHiracheryFrame& frame, float sigma)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());

	//std::normal_distribution<float> nd(1.0f, sigma);

	//for (auto& bone : frame)
	//{
	//	bone.GblTranslation *= 0.95;//nd(gen);
	//}
}

void PlayerProxy::Update(time_seconds const & time_delta)
{
	SceneObject::Update(time_delta);
	using namespace std;
	using namespace Eigen;

	if (!m_IsInitialized)
		return;

	if (g_DebugLocalMotion && !IsMapped())
	{
		current_time += time_delta;
		BoneHiracheryFrame last_frame;
		BoneHiracheryFrame anotherFrame, anotherLastFrame;
		for (auto& controller : m_Controllers)
		{
			if (!controller.IsReady)
				continue;
			auto& chara = controller.Character();
			auto& actionName = g_DebugLocalMotionAction[chara.Name];
			if (actionName.empty())
				continue;
			auto& action = controller.Character().Behavier()[actionName];
			auto& target_frame = controller.Character().MapCurrentFrameForUpdate();
			auto frame = controller.Character().Armature().default_frame();

			target_frame = frame;
			action.GetFrameAt(frame, current_time);
			action.GetFrameAt(last_frame, current_time - time_delta);

			//auto& anotheraction = controller.Character().Behavier()["run"];
			//anotheraction.GetFrameAt(anotherFrame, current_time);
			//anotheraction.GetFrameAt(anotherLastFrame, current_time - time_delta);

			//for (size_t i = 0; i < frame.size(); i++)
			//{
			//	frame[i].GblTranslation = DirectX::XMVectorLerp(frame[i].GblTranslation, anotherFrame[i].GblTranslation, g_NoiseInterpolation);
			//	last_frame[i].GblTranslation = DirectX::XMVectorLerp(last_frame[i].GblTranslation, anotherLastFrame[i].GblTranslation, g_NoiseInterpolation);
			//}

			// Add motion to non-active joints that visualize more about errors for active joints
			//target_frame = frame;
			//AddNoise(frame, .1f);
			controller.m_pSelfBinding->Transform(target_frame, frame, last_frame, time_delta.count());
		}
		return;
	}


	if (!m_IsInitialized || !m_playerSelector)
		return;

	static long long frame_count = 0;

	auto& player = *m_playerSelector;
	if (!player.IsTracked()) return;

	// no new frame is coming
	if (!player.ReadLatestFrame()) 
		return;
	const auto& frame = player.PeekFrame();
	m_LastPlayerFrame = m_CurrentPlayerFrame;
	m_CurrentPlayerFrame = frame;

	if (IsMapped())
	{
		auto& controller = CurrentController();
		controller.UpdateTargetCharacter(frame, m_LastPlayerFrame, time_delta.count());
		if (m_EnableOverShoulderCam)
			UpdatePrimaryCameraForTrack();
		return;
	}

	if (frame_count++ % SampleRate != 0)
	{
		return;
	}

	if (FeatureBuffer.size() >= 0 && !m_mapTaskOnGoing)
	{
		m_mapTaskOnGoing = true;
		m_mapTask = concurrency::create_task([this]() {
			auto idx = MapCharacterByLatestMotion();
			if (IsMapped())
			{
				CurrentController().Character().StopAction();
				cout << "Mapped!!!!!!!!!" << endl;
			}
			m_mapTaskOnGoing = false;
		});
	}
}

void PlayerProxy::UpdatePrimaryCameraForTrack()
{
	auto& camera = *this->Scene->PrimaryCamera();
	auto& cameraPos = dynamic_cast<SceneObject&>(camera);
	auto& contrl = this->CurrentController();
	auto& chara = contrl.Character();
	using namespace DirectX;
	XMVECTOR ext = XMLoad(chara.RenderModel()->GetBoundingBox().Extents);
	ext = XMVector3LengthEst(ext);
	ext *= chara.GetGlobalTransform().Scale;
	cameraPos.SetPosition((XMVECTOR)chara.GetPosition() + XMVector3Rotate(XMVectorMultiplyAdd(ext, XMVectorSet(-2.0f, 2.0f, -2.0f, 0.0f), XMVectorSet(-0.5f, 0.5, -0.5, 0)), chara.GetOrientation()));
	camera.GetView()->FocusAt((XMVECTOR)chara.GetPosition() + XMVector3Rotate(XMVectorMultiplyAdd(ext, XMVectorSet(-2.0f, 0.0f, 0.0f, 0.0f), XMVectorSet(-0.5f, 0.5, -0.5, 0)), chara.GetOrientation()), g_XMIdentityR1.v);
}

bool PlayerProxy::UpdateByFrame(const BoneHiracheryFrame & frame)
{
	BoneHiracheryFrame tframe;
	// Caculate likilihood
	for (int i = 0, end = m_Controllers.size(); i < end; ++i)
	{
		auto& obj = GetController(i);
		auto& binding = obj.Binding();
		auto& kobj = obj.Character();
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

	for (size_t i = 0; i < m_Controllers.size(); i++)
	{
		GetController(i).Character().SetOpticity(StateProbality(i));
	}

	int maxIdx = -1;
	auto currentP = StateProbality.maxCoeff(&maxIdx);
	if (maxIdx != m_CurrentIdx)
	{
		int oldIdx = m_CurrentIdx;
		m_CurrentIdx = maxIdx;
		//StateChangedEventArgs arg = { oldIdx,maxIdx,1.0f,GetController(oldIdx),GetController(maxIdx) };
		//if (!StateChanged.empty())
		//	StateChanged(arg);
	}

	if (IsMapped())
	{
		auto& state = CurrentController();
		auto& cframe = state.Character().MapCurrentFrameForUpdate();
		state.Binding().Transform(cframe, frame);
		state.Character().ReleaseCurrentFrameFrorUpdate();
	}
	return true;
}

bool PlayerProxy::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	return true;
}

void DrawJammedGuidingVectors(const ShrinkedArmature & barmature, const BoneHiracheryFrame & frame, const Color & color, const Matrix4x4 & world, float thinkness = 0.015f)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;
	if (frame.empty())
		return;
	//g_PrimitiveDrawer.SetWorld(world);
	g_PrimitiveDrawer.SetWorld(XMMatrixIdentity());
	//g_PrimitiveDrawer.Begin();
	for (auto& block : barmature)
	{
		if (block->parent() != nullptr)
		{
			auto& bone = frame[block->Joints.back()->ID];
			XMVECTOR ep = bone.GblTranslation;

			auto& pbone = frame[block->parent()->Joints.back()->ID];
			XMVECTOR sp = pbone.GblTranslation;

			sp = XMVector3Transform(sp, world);
			ep = XMVector3Transform(ep, world);
			//g_PrimitiveDrawer.DrawLine(sp, ep, color);

			//XMVECTOR v = ep - sp;
			//RowVectorXf ux = block->PdGpr.uX.cast<float>();


			g_PrimitiveDrawer.DrawCylinder(sp, ep, g_DebugArmatureThinkness, color);
			g_PrimitiveDrawer.DrawSphere(ep, g_DebugArmatureThinkness * 1.5f, color);
		}
	}
	//g_PrimitiveDrawer.End();


}

void DrawGuidingVectors(const ShrinkedArmature & barmature, const BoneHiracheryFrame & frame, const Color & color, const Matrix4x4 & world, float thinkness = 0.015f)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;
	if (frame.empty())
		return;
	//g_PrimitiveDrawer.SetWorld(world);
	g_PrimitiveDrawer.SetWorld(XMMatrixIdentity());
	//g_PrimitiveDrawer.Begin();
	for (auto& block : barmature)
	{
		if (block->parent() != nullptr)
		{
			auto& bone = frame[block->Joints.back()->ID];
			XMVECTOR ep = bone.GblTranslation;

			auto& pbone = frame[block->parent()->Joints.back()->ID];
			XMVECTOR sp = pbone.GblTranslation;

			sp = XMVector3Transform(sp, world);
			ep = XMVector3Transform(ep, world);
			//g_PrimitiveDrawer.DrawLine(sp, ep, color);

			g_PrimitiveDrawer.DrawCylinder(sp, ep, g_DebugArmatureThinkness, color);
			g_PrimitiveDrawer.DrawSphere(ep, g_DebugArmatureThinkness * 1.5f, color);
		}
	}
	//g_PrimitiveDrawer.End();


}

void DrawControllerHandle(const CharacterController& controller)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;
	using namespace DirectX;

	g_PrimitiveDrawer.SetWorld(XMMatrixIdentity());
	XMMATRIX world = controller.Character().GlobalTransformMatrix();

	auto& barmature = controller.Character().Behavier().Blocks();
	auto& frame = controller.Character().GetCurrentFrame();
	XMVECTOR color = Colors::Pink;
	XMVECTOR vel_color = Colors::Navy;

	for (auto& block : barmature)
	{
		if (block->ActiveActionCount > 0)
		{
			auto& handle = controller.PvHandles()[block->Index];
			XMVECTOR ep = handle.first;

			auto& pbone = frame[block->parent()->Joints.back()->ID];
			XMVECTOR sp = pbone.GblTranslation;
			ep = sp + ep;

			sp = XMVector3Transform(sp, world);
			ep = XMVector3Transform(ep, world);

			g_PrimitiveDrawer.DrawCylinder(sp, ep, g_DebugArmatureThinkness, color);
			g_PrimitiveDrawer.DrawSphere(ep, g_DebugArmatureThinkness * 1.5f, color);
			sp = ep;
			ep = handle.second;
			ep = XMVector3TransformNormal(ep, world);
			ep = sp + ep;
			g_PrimitiveDrawer.DrawCylinder(sp, ep, g_DebugArmatureThinkness, vel_color);
			g_PrimitiveDrawer.DrawCone(ep, ep - sp, g_DebugArmatureThinkness * 5, g_DebugArmatureThinkness * 3, vel_color);
		}
	}
}

void PlayerProxy::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	BoneHiracheryFrame charaFrame;

	if (g_DebugLocalMotion && g_DebugView)
	{
		for (auto& controller : m_Controllers)
		{
			if (!controller.IsReady)
				continue;
			auto& chara = controller.Character();
			auto& action = controller.Character().Behavier()[g_DebugLocalMotionAction[chara.Name]];
			action.GetFrameAt(charaFrame, current_time);
			auto world = chara.GlobalTransformMatrix();
			DrawArmature(chara.Armature(), charaFrame, DirectX::Colors::LimeGreen.v, world, g_DebugArmatureThinkness / chara.GetGlobalTransform().Scale.x);
			DrawControllerHandle(controller);
		}
	}

	if (!m_playerSelector) return;
	auto& player = *m_playerSelector;

	Color color = DirectX::Colors::Yellow.v;

	if (player.IsTracked())
	{
		const auto& frame = player.PeekFrame();

		if (IsMapped())
			color.A(0.3f);

		DrawArmature(*player.BodyArmature, frame, color);
	}

	// IsMapped() && 
	if (IsMapped() && g_DebugView)
	{
		//auto& controller = this->CurrentController().Character();
		for (auto& controller : m_Controllers)
		{
			if (!controller.IsReady)
				continue;

			auto& chara = controller.Character();

			//DirectX::Visualizers::g_PrimitiveDrawer.SetView(chara.GlobalTransformMatrix());
			auto pBinding = dynamic_cast<RBFInterpolationTransform*>(&controller.Binding());
			//if (pBinding)
			{
				auto world = chara.GlobalTransformMatrix();
				charaFrame = chara.GetCurrentFrame();

				//DrawGuidingVectors(chara.Behavier().Blocks(), charaFrame, DirectX::Colors::Crimson.v, world);

				pBinding->Render(DirectX::Colors::Crimson.v, world);
			}
		}

	}

}

void XM_CALLCONV PlayerProxy::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
}

KinectVisualizer::KinectVisualizer()
{
	pKinect = Devices::KinectSensor::GetForCurrentView();
}

bool KinectVisualizer::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	return true;
}

void KinectVisualizer::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	auto &players = pKinect->GetTrackedBodies();
	using DirectX::Visualizers::g_PrimitiveDrawer;

	for (auto& player : players)
	{
		if (player.IsTracked())
		{
			const auto& frame = player.PeekFrame();

			DrawArmature(*player.BodyArmature, frame, DirectX::Colors::LimeGreen.v);
		}
	}
}

void XM_CALLCONV KinectVisualizer::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
}

RenderFlags Causality::KinectVisualizer::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

