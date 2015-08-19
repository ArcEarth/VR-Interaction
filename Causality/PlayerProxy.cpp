#include "pch_bcl.h"
#include "PlayerProxy.h"
#include <PrimitiveVisualizer.h>
#include <fstream>
#include <Eigen\fft>
#include "CCA.h"
#include "EigenExtension.h"
#include "ArmatureBlock.h"
#include <algorithm>
#include "AnimationAnalyzer.h"
#include <ppl.h>
#include <boost\filesystem.hpp>
#include <random>

bool				g_UseGeneralTransform = false;
bool				g_EnableDebugLogging = true;
bool				g_EnableRecordLogging = false;
bool				g_EnableDependentControl = true;

using namespace Causality;
using namespace Eigen;
using namespace std;

using boost::filesystem::path;

path g_LogRootDir = "Log";

BlockArmature		g_PlayeerBlocks;

bool				g_DebugLocalMotion = false;
static const float	PcaCutoff = 0.04f; // 0.2^2
//static const float	EnergyCutoff = 0.3f;
static const float	EnergyCutoff = 0.3f;

static const float	CharacterPcaCutoff = 0.004f; // 0.2^2
static const float	CharacterEnergyCutoff = 0.40f;
static const float	CharacterEnergyCutoff2 = 0.02f;
static const float	alpha = 0.8f;

static const float  MatchAccepetanceThreshold = 0.2f;

bool				g_EnableInputFeatureLocalization = true;

static const char*  DefaultAnimationSet = "walk";
std::map<string, string> g_DebugLocalMotionAction;

#define BEGIN_TO_END(range) range.begin(), range.end()

template <class Derived>
void ExpandQuadraticTerm(_Inout_ Eigen::DenseBase<Derived> &v, _In_ DenseIndex dim)
{
	assert(v.cols() == dim + (dim * (dim + 1) / 2));

	int k = dim;
	for (int i = 0; i < dim; i++)
	{
		for (int j = i; j < dim; j++)
		{
			v.col(k) = v.col(i).array() * v.col(j).array();
			++k;
		}
	}
}


void PlayerProxy::StreamPlayerFrame(const TrackedBody& body, const TrackedBody::FrameType& frame)
{
	using namespace Eigen;
	using namespace DirectX;
	std::lock_guard<std::mutex> guard(BufferMutex);

	FeatureBuffer.push_back();

	if (FeatureBuffer.size() > RecordFeatures)
		FeatureBuffer.pop_front(); // everything works fine beside it invaliad the memery...

	if (!FeatureBuffer.is_linearized())
		FeatureBuffer.linearize();

	auto& fb = FeatureBuffer.back();
	float* vs = reinterpret_cast<float*>(fb.data());

	for (const auto& block : g_PlayeerBlocks)
	{
		auto vsi = RowVectorXf::Map(
			vs + block->AccumulatedJointCount * InputFeature::Dimension,
			block->Joints.size() * InputFeature::Dimension);

		vsi = pPlayerFeatureExtrator->Get(*block, frame);
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
		std::lock_guard<mutex> guard(BufferMutex);
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

class CcaArmatureTransform : virtual public ArmatureTransform
{
public:

	vector<PcaCcaMap> Maps;

public:
	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		//const auto dF = frame_type::StdFeatureDimension;
		//const auto dY = 3;
		//RowVectorXf X(source_frame.size() * dF);Im
		//RowVectorXf Y(target_frame.size() * dF);
		//source_frame.PopulateStdFeatureVector(X);
		//target_frame.PopulateStdFeatureVector(Y); //! populate non-mapped with default values

		//for (const auto& map : Maps)
		//{
		//	auto& Xp = X.middleCols<dF>(map.Jx * dF);
		//	auto& Yp = Y.middleCols<dY>(map.Jy * dF);
		//	ApplyCcaMap(map, Xp, Yp);
		//}

		//target_frame.RebuildFromStdFeatureVector(Y, *pTarget);
	}
};

class BlockEndEffectorGblPosQuadratic : public BlockFeatureExtractor
{
public:
	BlockEndEffectorGblPosQuadratic()
	{
	}

	typedef BoneFeatures::QuadraticGblPosFeature BoneFeatureType;
	virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override
	{
		RowVectorX Y(BoneFeatureType::Dimension);

		BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[block.Joints.back()->ID()]);

		if (block.parent() != nullptr)
		{
			RowVectorX reference(BoneFeatures::GblPosFeature::Dimension);
			BoneFeatures::GblPosFeature::Get(reference, frame[block.parent()->Joints.back()->ID()]);
			Y.segment<3>(0) -= reference;
		}

		ExpandQuadraticTerm(Y, 3);

		//Y[3] = Y[0] * Y[0];
		//Y[4] = Y[1] * Y[1];
		//Y[5] = Y[2] * Y[2];
		//Y[6] = Y[0] * Y[1];
		//Y[7] = Y[1] * Y[2];
		//Y[8] = Y[2] * Y[0];
		return Y;
	}

	// Inherited via BlockFeatureExtractor
	virtual void Set(const KinematicBlock & block, AffineFrame & frame, const RowVectorX & feature) override
	{
		assert(false);
	}
};

class BlockizedArmatureTransform : virtual public ArmatureTransform
{
protected:
	const BlockArmature *pSblocks, *pTblocks;
public:

	BlockizedArmatureTransform() :pSblocks(nullptr), pTblocks(nullptr)
	{
		pSource = nullptr;
		pTarget = nullptr;
	}

	BlockizedArmatureTransform(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
	{
		SetFrom(pSourceBlock, pTargetBlock);
	}

	void SetFrom(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
	{
		pSblocks = pSourceBlock; pTblocks = pTargetBlock;
		pSource = &pSourceBlock->Armature();
		pTarget = &pTargetBlock->Armature();
	}
};

class BlockizedCcaArmatureTransform : public CcaArmatureTransform, public BlockizedArmatureTransform
{
public:
	uptr<BlockFeatureExtractor> pInputExtractor, pOutputExtractor;
public:

	using BlockizedArmatureTransform::BlockizedArmatureTransform;

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		const auto& sblocks = *pSblocks;
		const auto& tblocks = *pTblocks;
		RowVectorXf X,Y;
		for (const auto& map : Maps)
		{
			if (map.Jx == -1 || map.Jy == -1) continue;

			auto pTb = tblocks[map.Jy];
			Y = pOutputExtractor->Get(*pTb, target_frame);

			if (map.Jx == -2 && map.Jy >= 0)
			{
				vector<RowVectorXf> Xs;
				int Xsize = 0;
				for (auto& pBlock : tblocks)
				{
					if (pBlock->ActiveActionCount > 0)
					{
						Xs.emplace_back(pInputExtractor->Get(*pBlock, source_frame));
						Xsize += Xs.back().size();
					}
				}

				X.resize(Xsize);
				Xsize = 0;
				for (auto& xr : Xs)
				{
					X.segment(Xsize, xr.size()) = xr;
					Xsize += xr.size();
				}
			}
			else
			{
				auto pSb = sblocks[map.Jx];
				X = pInputExtractor->Get(*pSb, source_frame);
			}

			ApplyPcaCcaMap(map, X, Y);

			//cout << " X = " << X << endl;
			//cout << " Yr = " << Y << endl;
			pOutputExtractor->Set(*pTb, target_frame, Y);

		}

		//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
		//target_frame[0].LclRotation = source_frame[0].LclRotation;
		target_frame.RebuildGlobal(*pTarget);
	}

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const AffineFrame& last_frame, float frame_time) const
	{
		// redirect to pose transform
		Transform(target_frame, source_frame);
		return;

		const auto& sblocks = *pSblocks;
		const auto& tblocks = *pTblocks;
		for (const auto& map : Maps)
		{
			if (map.Jx == -1 || map.Jy == -1) continue;

			auto pSb = sblocks[map.Jx];
			auto pTb = tblocks[map.Jy];
			auto X = pInputExtractor->Get(*pSb, source_frame, last_frame, frame_time);
			auto Y = pOutputExtractor->Get(*pTb, target_frame);

			ApplyPcaCcaMap(map, X, Y);

			//cout << " X = " << X << endl;
			//cout << " Yr = " << Y << endl;
			pOutputExtractor->Set(*pTb, target_frame, Y);
		}

		//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
		//target_frame[0].LclRotation = source_frame[0].LclRotation;
		target_frame.RebuildGlobal(*pTarget);
	}
};

class Causality::ClipInfo : public AnimationAnalyzer
{
public:

	ClipInfo(const BlockArmature& pBArm)
		: AnimationAnalyzer(pBArm)
	{
		Phi = -1;
		Score = 0;
	}
	// Information for current 'map'
	int								Phi;
	std::vector<Eigen::DenseIndex>	Matching;
	float							Score;
	MatrixXf						Ra; // Overall correlation
	MatrixXf						Rk; // Kinetic correlation
	MatrixXf						Rs; // Positional correlation

	uptr<CcaArmatureTransform>		pLocalBinding;
};

class PerceptiveVectorBlockFeatureExtractor : public BlockFeatureExtractor
{
	map<string, ClipInfo*>*		pClipInfos;

public:
	float						segma;
	bool						QuadraticInput;

	PerceptiveVectorBlockFeatureExtractor(map<string, ClipInfo*>* pClips)
	{
		QuadraticInput = true;
		segma = 10;
		pClipInfos = pClips;
	}
public:

	typedef BoneFeatures::GblPosFeature InputFeatureType;
	typedef CharacterFeature CharacterFeatureType;
	virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override
	{
		DenseIndex dim = InputFeatureType::Dimension;
		if (QuadraticInput)
			dim += dim * (dim + 1) / 2;

		RowVectorX Y(dim);

		auto& aJoint = *block.Joints.back();

		BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[aJoint.ID()]);

		if (block.parent() != nullptr)
		{
			RowVectorX reference(BoneFeatures::GblPosFeature::Dimension);
			BoneFeatures::GblPosFeature::Get(reference, frame[block.parent()->Joints.back()->ID()]);
			Y.segment<3>(0) -= reference;
		}

		if (QuadraticInput)
		{
			ExpandQuadraticTerm(Y, InputFeatureType::Dimension);
		}

		return Y;
	}

	template <class Derived>
	void distance2prob(_Inout_ ArrayBase<Derived>& dis)
	{
		static const float ProbThreshold = 0.001f;

		dis = (-(dis*dis) / (segma*segma)).exp() / (segma * sqrt(DirectX::XM_2PI));

		float sum = dis.sum();
		if (sum > ProbThreshold)
			dis /= sum;
		else
			dis.setZero();
		//return expf(-(dis * dis) / (segma*segma)) / (segma * sqrt(DirectX::XM_2PI));
	}

	// Inherited via BlockFeatureExtractor
	virtual void Set(const KinematicBlock & block, AffineFrame & frame, const RowVectorX & feature) override
	{
		using namespace DirectX;
		if (block.ActiveActionCount > 0)
		{
			RowVectorXf Y(CharacterFeatureType::Dimension * block.Joints.size());

			if (block.PdDriver.Jx == -1 || block.PdDriver.Jy == -1) return;
			block.PdDriver.Apply(feature, Y);

			for (size_t j = 0; j < block.Joints.size(); j++)
			{
				auto jid = block.Joints[j]->ID();
				auto Xj = Y.segment<CharacterFeatureType::Dimension>(j * CharacterFeatureType::Dimension);
				CharacterFeatureType::Set(frame[jid], Xj);
			}

			return;

			auto pv = feature.segment<3>(0);

			MatrixXf locomotion(block.ActiveActionCount, CharacterFeatureType::Dimension * block.Joints.size());
			VectorXf probs(block.ActiveActionCount);

			int i = 0;
			for (auto& action : block.ActiveActions)
			{
				auto& clipinfo = *(*pClipInfos)[action];

				// input perceptive vector


				// chracter perceptive vectors
				auto cpvs = Eigen::Matrix<float, CLIP_FRAME_COUNT, 3, Eigen::RowMajor>::Map(clipinfo.Pvs.col(block.Index).data());

				// use linear interpolation to evaluate
				// distance from input feature vector to clip trajectory
				float dis = LineSegmentTest::Distance(DirectX::XMLoadFloat3(pv.data()),
					reinterpret_cast<XMFLOAT3*>(cpvs.data()),
					CLIP_FRAME_COUNT);

				//float dis = sqrtf((cpvs.rowwise() - pv).rowwise().squaredNorm().minCoeff());
				probs[i] = dis;

				auto& map = clipinfo.PerceptiveVectorReconstructor[block.Index];
				map.Apply(feature, locomotion.row(i));
				i++;
			}

			distance2prob(probs.array());

			if (probs.maxCoeff() > 0.001)
			{
				locomotion = probs.asDiagonal() * locomotion;

				Y = locomotion.colwise().sum();

				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID();
					auto Xj = Y.segment<CharacterFeatureType::Dimension>(j * CharacterFeatureType::Dimension);
					CharacterFeatureType::Set(frame[jid], Xj);
				}
			}
		}
	}
};

class BlockizedSpatialInterpolateQuadraticTransform : public BlockizedCcaArmatureTransform
{

public:
	BlockizedSpatialInterpolateQuadraticTransform(map<string, ClipInfo*>* pClips)
	{
		pClipInfoMap = pClips;
		auto pIF = new PerceptiveVectorBlockFeatureExtractor(pClips);
		pIF->QuadraticInput = true;
		pInputExtractor.reset(pIF);

		auto pOF = new PerceptiveVectorBlockFeatureExtractor(pClips);
		pOF->segma = 30;
		pOutputExtractor.reset(pOF);

		pDependentBlockFeature.reset(new BlockAllJointsFeatureExtractor<CharacterFeature>(false));
	}

	BlockizedSpatialInterpolateQuadraticTransform(map<string, ClipInfo*>* pClips, const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
		: BlockizedSpatialInterpolateQuadraticTransform(pClips)
	{
		SetFrom(pSourceBlock, pTargetBlock);
		pvs.resize(pTargetBlock->size());
	}

	map<string, ClipInfo*>*		pClipInfoMap;

	mutable vector<pair<Vector3, Vector3>> pvs;
	uptr<BlockFeatureExtractor> pDependentBlockFeature;

	void Render(ID3D11DeviceContext *pContext, DirectX::CXMVECTOR color, DirectX::CXMMATRIX world)
	{
		using namespace DirectX;
		auto& drawer = DirectX::Visualizers::g_PrimitiveDrawer;
		drawer.SetWorld(DirectX::XMMatrixIdentity());
		
		//drawer.Begin();
		for (const auto& map : Maps)
		{
			auto& line = pvs[map.Jy];
			XMVECTOR p0, p1;
			p0 = XMVector3Transform(line.first.Load(), world);
			p1 = XMVector3Transform(line.second.Load(), world);
			//drawer.DrawLine(line.first, line.second, color);

			if (map.Jx >= 0)
			{

				drawer.DrawCylinder(p0, p1, g_DebugArmatureThinkness, color);
				drawer.DrawSphere(p1, g_DebugArmatureThinkness * 1.5f, color);
			}
			else
			{
				drawer.DrawSphere(p0, g_DebugArmatureThinkness * 2.0f, color);
			}
		}
		//drawer.End();
	}

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		const auto& sblocks = *pSblocks;
		const auto& tblocks = *pTblocks;

		int abcount = 0;
		for (const auto& map : Maps)
		{
			//Y = X;
			if (map.Jx < 0 || map.Jy < 0) continue;

			auto pSb = sblocks[map.Jx];
			auto pTb = tblocks[map.Jy];
			auto X = pInputExtractor->Get(*pSb, source_frame);
			auto Y = pOutputExtractor->Get(*pTb, target_frame);


			ApplyPcaCcaMap(map, X.leftCols<3>(), Y);

			Y.conservativeResize(9);
			ExpandQuadraticTerm(Y, 3);

			//cout << " X = " << X << endl;
			//cout << " Yr = " << Y << endl;
			pOutputExtractor->Set(*pTb, target_frame, Y);

			abcount++;
			pvs[map.Jy].second = Vector3(Y.segment<3>(0).data());
		}

		target_frame.RebuildGlobal(*pTarget);

		// Fill Xabpv
		RowVectorXf Xabpv;
		Xabpv.resize(abcount * 9);
		abcount = 0;
		for (const auto& map : Maps)
		{
			if (map.Jx < 0 || map.Jy < 0) continue;

			auto Yi = Xabpv.segment<9>(abcount * 9);
			Yi.segment<3>(0) = Vector3f::Map(&pvs[map.Jy].second.x);
			ExpandQuadraticTerm(Yi, 3);

			abcount++;
		}

		// Dependent blocks
		for (const auto& map : Maps)
		{
			if (map.Jx == -2 && map.Jy >= 0)
			{
				if (map.uXpca.size() == Xabpv.size())
				{
					auto pTb = tblocks[map.Jy];
					auto Y = pOutputExtractor->Get(*pTb, target_frame);
					map.Apply(Xabpv, Y);
					pDependentBlockFeature->Set(*pTb, target_frame, Y);
				}
				else
				{

				}
			}
		}


		for (const auto& map : Maps)
		{
			auto pTb = tblocks[map.Jy];

			if (pTb->parent())
				pvs[map.Jy].first = target_frame[pTb->parent()->Joints.back()->ID()].GblTranslation;
			pvs[map.Jy].second += pvs[map.Jy].first;
		}

	}

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const AffineFrame& last_frame, float frame_time) const override
	{
		Transform(target_frame, source_frame);
	}
};


pair<JointType, JointType> XFeaturePairs[] = {
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

//static const size_t KeyJointCount = ARRAYSIZE(KeyJoints);
//
//static const size_t FeatureCount = (KeyJointCount * (KeyJointCount - 1)) / 2;
//static const size_t FeatureDim = FeatureCount * 3;

VectorXf HumanFeatureFromFrame(const AffineFrame& frame)
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
	: IsInitialized(false),
	playerSelector(nullptr),
	CurrentIdx(-1),
	FeatureBuffer(FeatureBufferCaptcity),
	current_time(0)
{
	pPlayerFeatureExtrator.reset(new BlockAllJointsFeatureExtractor<InputFeature>(g_EnableInputFeatureLocalization));
	pKinect = Devices::KinectSensor::GetForCurrentView();
	pPlayerArmature = &pKinect->Armature();
	if (g_PlayeerBlocks.empty())
	{
		g_PlayeerBlocks.SetArmature(*pPlayerArmature);
	}

	auto fReset = std::bind(&PlayerProxy::ResetPlayer, this, placeholders::_1, placeholders::_2);
	playerSelector.SetPlayerChangeCallback(fReset);

	auto fFrame = std::bind(&PlayerProxy::StreamPlayerFrame, this, placeholders::_1, placeholders::_2);
	playerSelector.SetFrameCallback(fFrame);

	playerSelector.Initialize(pKinect.get(), TrackedBodySelector::SelectionMode::Closest);

	Register();
	IsInitialized = true;
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
		Controllers.emplace_back();
		auto& controller = Controllers.back();
		controller.ID = Controllers.size() - 1;
		controller.Initialize(*pPlayerArmature, *pChara);
		pChara->SetOpticity(1.0f);

		if (g_DebugLocalMotion)
		{
			g_DebugLocalMotionAction[pChara->Name] = pChara->CurrentActionName();
			pChara->StopAction();
		}

		auto glow = pChara->FirstChildOfType<GlowingBorder>();
		if (glow == nullptr)
		{
			glow = new GlowingBorder(DirectX::Colors::LimeGreen.v);
			glow->SetEnabled(false);
			pChara->AddChild(glow);
		}
	}
}

const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void PlayerProxy::SetActiveController(int idx)
{
	if (idx >= 0)
		idx = idx % Controllers.size();

	for (auto& c : Controllers)
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
			auto glow = chara.FirstChildOfType<GlowingBorder>();
			glow->SetEnabled(false);

			if (c.ID == CurrentIdx && CurrentIdx != idx)
				chara.SetPosition(c.CMapRefPos);
		}
	}

	if (CurrentIdx != idx)
	{
		CurrentIdx = idx;
		if (CurrentIdx != -1)
		{
			auto& controller = GetController(CurrentIdx);
			auto& chara = controller.Character();

			if (!g_DebugLocalMotion && !chara.CurrentActionName().empty())
			{
				g_DebugLocalMotionAction[chara.Name] = chara.CurrentActionName();
				chara.StopAction();
			}

			chara.SetOpticity(1.0f);
			auto glow = chara.FirstChildOfType<GlowingBorder>();
			glow->SetEnabled(!g_DebugView);

			controller.MapRefPos = playerSelector->CurrentFrame()[0].GblTranslation;
			controller.CMapRefPos = chara.GetPosition();

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


int PlayerProxy::MapCharacterByLatestMotion()
{
	static const size_t FilterStregth = 6U; //Applies 4 iterates of Laplicaian filter
	static const size_t CropMargin = 1 * SAMPLE_RATE; // Ignore most recent 1 sec
	static const float Cutoff_Correlation = 0.5f;
	time_seconds InputDuration(5);

	auto& player = *playerSelector;
	static ofstream flog;
	if (g_EnableDebugLogging)
		flog.open("Xlog.txt");

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
		//! Smooth the input 
		laplacian_smooth(Xf.col(i), FilterStregth);

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
	Xs = cublic_bezier_resample(
		X.middleRows(N - (2 * T + CropMargin + 1), T * 2),
		StretchedSampleCount * 2,
		Eigen::CloseLoop);

	if (flog.is_open())
	{
		flog << Xs << endl;
		flog.close();
	}

	T = StretchedSampleCount; // Since we have resampled , Time period "T" is now equals to StretchedSampleCount
	int Ti = 1; // 2
	int Ts = T / Ti;

	int Ju = g_PlayeerBlocks.size();

	// Spatial Traits
	Eigen::MatrixXf Xsp(3, Ju);
	vector<vector<MatrixXf>> RawXs(Ts);
	vector<vector<MeanThinQr<MatrixXf>>> QrXs(Ts);
	vector<vector<Pca<MatrixXf>>> PcaXs(Ts);
	for (size_t i = 0; i < Ts; i++)
	{
		RawXs[i].resize(Ju);
		QrXs[i].resize(Ju);
		PcaXs[i].resize(Ju);
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

		for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
		{
			auto Xp = Xk.middleRows(T - phi, T);
			auto& pca = PcaXs[phi / Ti][i];

			//! Important!!! Using Xp.rightCols<3>() instead of Xp for QrXs!!!
			//pca.compute(Xp, true);
			RawXs[phi / Ti][i] = Xp.rightCols<3>();
			pca.compute(Xp.rightCols<3>(), true);
			auto d = pca.reducedRank(DirectX::XM_EPSILON);//pca.reducedRank(PcaCutoff);
			QrXs[phi / Ti][i].compute(pca.coordinates(d), true);
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
		if (Eub(i) > EnergyCutoff)
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

	for (auto& controller : Controllers)			//? <= 5 character
	{
		if (!controller.IsReady)
			continue;
		auto& character = controller.Character();
		auto& chraraBlocks = character.Behavier().Blocks();
		size_t Jc = chraraBlocks.size();
		auto& clips = character.Behavier().Clips();

		//controller.CharacterScore = numeric_limits<float>::min();
		//auto& anim = character.Behavier()[DefaultAnimationSet];
		auto& anim = *character.CurrentAction();
		//for (auto& anim : clips)	//? <= 5 animation per character
		{
			auto& analyzer = controller.GetAnimationInfo(anim.Name);
			if (&analyzer == nullptr)
				continue;

			// Caculate Ecb, Energy of Character Blocks
			auto Ecb = analyzer.Eb;
			MatrixXf Ecb3 = analyzer.Eb3;
			// Independent Active blocks only
			vector<int> Jck;
			for (size_t i = 0; i < Jc; i++)
			{
				if (Ecb(i) > analyzer.EnergyCutoff)
				{
					Ecb(Jck.size()) = Ecb(i);
					Ecb3.col(Jck.size()) = Ecb3.col(i);
					Jck.push_back(i);
				}
			}
			Ecb.conservativeResize(Jck.size());
			Ecb3.conservativeResize(3,Jck.size());

			// Alternate Spatial traits
			Matrix<float, -1, -1> Csp(3, Jck.size());
			MatrixXf Csp2(3 * T, Jck.size());

			for (size_t i = 0; i < Jck.size(); i++)
			{
				Csp.col(i) = analyzer.Sp.block<3, 1>(0, Jck[i]);
				Csp2.col(i) = analyzer.Pvs.col(Jck[i]);

				// Normalize Pvs to caculate directions
				auto cpvs = Eigen::Matrix<float, CLIP_FRAME_COUNT, 3, Eigen::RowMajor>::Map(Csp2.col(i).data());
				cpvs.rowwise().normalize();

				RowVector3f E3 = analyzer.Eb3.col(Jck[i]).transpose();

				// d(X)*d(P) >= h/2, thus, more energy in movement means less important to measure it's position
				E3 = (E3.array() + 0.1f).cwiseInverse();
				E3 /= E3.maxCoeff();
				cpvs.array() *= E3.replicate(T, 1).array();
			}

			// Score for spatial traits !!!
			MatrixXf Asp = Xsp.transpose() * Csp;

			// Memery allocation
			auto CoRSize = Juk.size() + Jck.size();

			std::vector<MatrixXf> Ra(Ts), Rm(Ts), Rs(Ts);

			for (size_t i = 0; i < Ts; i++)
			{
				Ra[i].setZero(Juk.size(), Jck.size());
				Rs[i].setZero(Juk.size(), Jck.size());
				Rm[i].setZero(Juk.size(), Jck.size());
			}

			//MatrixXf CoR;//(Ju + Jc, Ju + Jc);
			//CoR.setZero(CoRSize, CoRSize);


			VectorXf mScores(Ts);
			vector<vector<DenseIndex>> mMatchings(Ts);
			//DenseIndex phi = 0;

			concurrency::parallel_for(0, (int)T, Ti, [&](auto phi)
				//for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
			{
				auto phidx = phi / Ti;

				auto Xp = Xs.middleRows(T - phi, T);


				auto& Mcor = Rm[phidx];//.topLeftCorner(Juk.size(), Jck.size());

				for (int i = 0; i < Juk.size(); ++i)
				{
					int uid = Juk[i];
					auto& qrX = QrXs[phi / Ti][uid];
					auto& rawX = RawXs[phi / Ti][uid];

					for (int j = 0; j < Jck.size(); j++)
					{
						int cid = Jck[j];
						//? Here we uses Qrs or PvQrs is important!!!
						auto& qrY = analyzer.PvQrs[cid];
						auto rawY = MatrixXf::Map(analyzer.Pvs.col(cid).data(), 3, CLIP_FRAME_COUNT).transpose();

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
					}
				}
				// a = (a > cutoff) ? a : -1.0f
				//Mcor = (Mcor.array() > Cutoff_Correlation).select(Mcor, -1.0f);

				auto& Scor = Rs[phidx];
				auto Xsp2 = XDir.middleRows((T - phi) * 3, T * 3);
				Scor = Xsp2.transpose() * Csp2;
				Scor /= (float)T;

				auto& A = Ra[phidx];
				// Cutoff the none-correlated match
				//A *= alpha;

				// Here Eub and Ecb is Kinetic Energy, it should only affect the motion correlation term
				Mcor.array() *= Eub.array().replicate(1, A.cols());
				//Mcor.array() *= Ecb.array().replicate(A.rows(), 1);

				A = Mcor * alpha + Scor;

				//CoR.topLeftCorner(Ju, Jc) = A;

				//auto matching = max_weight_bipartite_matching(A.transpose());
				//auto score = matching_cost(A, matching);
				//auto score = matching_cost(A.transpose(), matching);// / A.cols();

				vector<DenseIndex> matching(A.cols());
				VectorXf XScore(A.rows());
				VectorXi XCount(A.rows());
				XCount.setZero();
				XScore.setZero();
				for (int k = 0; k < A.cols(); k++)
				{
					DenseIndex jx;
					auto score = A.col(k).maxCoeff(&jx);
					if (score < MatchAccepetanceThreshold) // Reject the match if it's less than a threshold
						matching[k] = -1;
					else
					{
						matching[k] = jx;
						XScore(jx) += score;
						++XCount(jx);
					}
				}
				//XScore.array() /= XCount.array().cast<float>();
				auto score = XScore.sum();

				mScores[phidx] = score;
				mMatchings[phidx] = matching;
			}
			);

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

			analyzer.Score = maxScore;
			analyzer.Matching = maxMatching;
			analyzer.Phi = maxPhi;
			if (maxPhi > -1)
			{
				analyzer.Ra = Ra[maxPhi];
				analyzer.Rk = Rm[maxPhi];
				analyzer.Rs = Rs[maxPhi];
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
					cout << pJoint->Name() << ", ";
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
					cout << pJoint->Name() << ", ";
				}
				cout << "\b\b}" << endl;
			}
			cout << "*********************************************" << endl;

			// Setup Binding from matching
			if (maxPhi > -1)//controller.CharacterScore < maxScore)
			{
				//cout << "New best : " << anim.Name << endl;
				controller.CharacterScore = std::max(maxScore, controller.CharacterScore);

				auto pBinding = new BlockizedSpatialInterpolateQuadraticTransform(&controller.GetClipInfos(),
					&g_PlayeerBlocks,
					&controller.Character().Behavier().Blocks());

				//auto pBinding = new BlockizedCcaArmatureTransform(&g_PlayeerBlocks, &controller.Character().Behavier().Blocks());
				//pBinding->pInputExtractor.reset(new BlockAllJointsFeatureExtractor<InputFeature>(g_EnableInputFeatureLocalization));
				//pBinding->pOutputExtractor.reset(new BlockAllJointsFeatureExtractor<CharacterFeature>(false));

				analyzer.pLocalBinding.reset(pBinding);

				//for (int i = 0; i < Juk.size(); ++i)
				for (int j = 0; j < Jck.size(); ++j)
				{
					DenseIndex Jx = maxMatching[j], Jy = j;
					//DenseIndex Jx = i, Jy = maxMatching[i];
					if (Jy == -1 || Jy >= Jc || Jx == -1 || Jx >= Jc) continue;
					Jx = Juk[Jx]; Jy = Jck[Jy];

					const auto& blX = *g_PlayeerBlocks[Jx];
					const auto& blY = *controller.Character().Behavier().Blocks()[Jy];

					//cout << pPlayerArmature->at(Jx)->Name() << " ==> " << controller.Character().Armature()[Jy]->Name() << endl;

					auto& rawX = RawXs[maxPhi / Ti][Jx];
					auto& qrX = QrXs[maxPhi / Ti][Jx];
					auto& pcaX = PcaXs[maxPhi / Ti][Jx];

					//? Again!!! PvQrs vs Qrs
					auto rawY = MatrixXf::Map(analyzer.Pvs.col(Jy).data(),3, CLIP_FRAME_COUNT).transpose();
					auto& qrY = analyzer.PvQrs[Jy];
					auto& pcaY = analyzer.PvPcas[Jy];
					//auto& qrY = analyzer.Qrs[Jy];
					//auto& pcaY = analyzer.Pcas[Jy];

					Cca<float> cca;
					cca.computeFromQr(qrX, qrY, true);

					cout << '{';
					for (auto pJoint : blX.Joints)
					{
						cout << pJoint->Name() << ", ";
					}
					cout << "\b\b} ==> {";
					for (auto pJoint : blY.Joints)
					{
						cout << pJoint->Name() << ", ";
					}
					cout << "\b\b}" << endl;

					// populate map
					pBinding->Maps.emplace_back();
					auto& map = pBinding->Maps.back();
					map.Jx = Jx; map.Jy = Jy;

					if (g_UseGeneralTransform)
					{
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
					}
					else
					{
						assert(rawX.cols() == rawY.cols() && "input X,Y dimension disagree");
						int rank = rawX.cols();

						//sum(Xi1*Yi1,Xi2*Yi2,Xi3*Yi3)
						MatrixXf covXY = rawX.transpose() * rawY;

						// The one axis rotation matrix
						Matrix3f BestRot;
						float BestScale,bestAng;
						int bestPiv = -1;
						float bestErr = numeric_limits<float>::max();

						for (int pivot = 0; pivot < 3; pivot++)
						{
							Matrix3f Rot;
							float scale = 1.0f;
							float ang = GetConstraitedRotationFromSinVector(Rot, covXY, pivot);
							// the isometric scale factor
							scale = ((rawX * Rot).array()*rawY.array()).sum() / rawX.cwiseAbs2().sum();
							float err = (rawX * scale * Rot - rawY).cwiseAbs2().sum();
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
							cout << "Failed to find isometric transform about control handle" << endl;
							continue;
						}
						else
						{
							static char xyz[] = "XYZ";
							cout << "Isometric transform found : Scale [" << BestScale << "] , Rotation along axis [" << xyz[bestPiv] << "] for " << bestAng / DirectX::XM_PI << "pi , Error = " << bestErr << endl;
						}
						//RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
						//float err = (rawY - rawY * alpha.asDiagonal()).cwiseAbs2().sum();

						map.A = BestScale * BestRot; //* MatrixXf::Identity(rank, rank);
						map.B = MatrixXf::Identity(rank, rank);
						map.uX = RowVectorXf::Zero(rank);
						map.uY = RowVectorXf::Zero(rank);
						map.uXpca = RowVectorXf::Zero(rank);
						map.uYpca = RowVectorXf::Zero(rank);
						map.pcX = MatrixXf::Identity(rank, rank);
						map.pcY = MatrixXf::Identity(rank, rank);
						map.useInvB = true;
						map.invB = MatrixXf::Identity(rank, rank);;
					}
				}
			}
		} // Animation clip scope

		//auto pBinding = new BlockizedCcaArmatureTransform(&g_PlayeerBlocks, &controller.Character().Behavier().Blocks());
		//pBinding->pInputExtractor.reset(new BlockAllJointsFeatureExtractor<InputFeature>(g_EnableInputFeatureLocalization));
		//pBinding->pOutputExtractor.reset(new BlockAllJointsFeatureExtractor<CharacterFeature>(false));

		auto pBinding = new BlockizedSpatialInterpolateQuadraticTransform(&controller.GetClipInfos(),
			&g_PlayeerBlocks,
			&controller.Character().Behavier().Blocks());

		controller.SetBinding(pBinding);

		auto clipinfos = controller.m_Analyzers | adaptors::map_values;
		auto maxClip = std::max_element(BEGIN_TO_END(clipinfos), [](const ClipInfo* lhs, const ClipInfo *rhs) {
			return (rhs != nullptr) && (lhs == nullptr || lhs->Score < rhs->Score);
		});

		pBinding->Maps = (*maxClip)->pLocalBinding->Maps;


		if (g_EnableDependentControl)
		{
			for (auto& pBlock : chraraBlocks)
			{
				auto& block = *pBlock;
				//if (block.Index == 0)
				//	continue;
				if (block.ActiveActionCount == 0 && block.PvDriveScore > 0.5f)
				{
					pBinding->Maps.emplace_back(block.PdDriver);
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
		//		float bestScore = MatchAccepetanceThreshold;
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
		//			auto jyid = std::find(BEGIN_TO_END(clipinfo.ActiveBlocks), bid) - clipinfo.ActiveBlocks.begin();
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

	CharacterController* pControl = nullptr;
	for (auto& con : Controllers)
	{
		if (!pControl || con.CharacterScore > pControl->CharacterScore)
			pControl = &con;
	}
	if (!pControl) return -1;
	SetActiveController(pControl->ID);

	return pControl->ID;
}

//void PlayerProxy::LocalizePlayerFeature(Eigen::MatrixXf &X)
//{
//	for (const auto &pBlock : g_PlayeerBlocks) //? <= 25 joint per USER?
//	{
//		auto& block = *pBlock;
//		if (block.Joints[0]->is_root()) continue;
//		auto refId = block.Joints[0]->parent()->ID();
//		auto refCols = X.middleCols<JointDemension>(refId*JointDemension);
//		for (auto& pJ : block.Joints)
//		{
//			auto jid = pJ->ID();
//			X.middleCols<JointDemension>(jid*JointDemension) -= refCols;
//		}
//	}
//}

void PlayerProxy::ClearPlayerFeatureBuffer()
{
	std::lock_guard<std::mutex> guard(BufferMutex);
	FeatureBuffer.clear();
}

// Character Map State

bool PlayerProxy::IsMapped() const { return CurrentIdx >= 0; }

const CharacterController & PlayerProxy::CurrentController() const {
	for (auto& c : Controllers)
	{
		if (c.ID == CurrentIdx)
			return c;
	}
}

CharacterController & PlayerProxy::CurrentController() {
	for (auto& c : Controllers)
	{
		if (c.ID == CurrentIdx)
			return c;
	}
}

const CharacterController & PlayerProxy::GetController(int state) const {
	for (auto& c : Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

CharacterController & PlayerProxy::GetController(int state)
{
	for (auto& c : Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

void PlayerProxy::OnKeyUp(const KeyboardEventArgs & e)
{
	if (e.Key == VK_OEM_PERIOD || e.Key == '.' || e.Key == '>')
	{
		SetActiveController(CurrentIdx + 1);
	}
	else if (e.Key == VK_OEM_COMMA || e.Key == ',' || e.Key == '<')
	{
		SetActiveController(CurrentIdx - 1);
	}
	else if (e.Key == 'L' || e.Key == 'l')
	{
		// this behavier should not change in mapped mode
		if (IsMapped()) return;

		g_DebugLocalMotion = !g_DebugLocalMotion;
		if (g_DebugLocalMotion)
		{
			for (auto& controller : Controllers)
			{
				auto& chara = controller.Character();
				g_DebugLocalMotionAction[chara.Name] = chara.CurrentActionName();
				chara.StopAction();
			}
		}
		else
		{
			for (auto& controller : Controllers)
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
		for (auto& controller : Controllers)
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
	else if (e.Key == 'P' || e.Key == 'p')
	{
		g_EnableDependentControl = !g_EnableDependentControl;
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

void AddNoise(AffineFrame& frame, float sigma)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());

	std::normal_distribution<float> nd(1.0f, sigma);

	for (auto& bone : frame)
	{
		bone.GblTranslation *= nd(gen);
	}
}

void PlayerProxy::Update(time_seconds const & time_delta)
{
	SceneObject::Update(time_delta);
	using namespace std;
	using namespace Eigen;

	if (!IsInitialized)
		return;

	if (g_DebugLocalMotion && !IsMapped())
	{
		current_time += time_delta;
		AffineFrame last_frame;
		for (auto& controller : Controllers)
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

			// Add motion to non-active joints that visualize more about errors for active joints
			//target_frame = frame;
			//AddNoise(frame, .1f);
			controller.m_pSelfBinding->Transform(target_frame, frame, last_frame, time_delta.count());
		}
		return;
	}


	if (!IsInitialized || !playerSelector)
		return;

	static long long frame_count = 0;

	auto& player = *playerSelector;
	if (!player.IsTracked()) return;

	const auto& frame = player.PullLatestFrame();

	if (IsMapped())
	{
		auto& controller = CurrentController();
		controller.UpdateTargetCharacter(frame);
		return;
	}

	if (frame_count++ % SampleRate != 0)
	{
		return;
	}

	if (FeatureBuffer.size() >= 0)
	{
		auto idx = MapCharacterByLatestMotion();
		if (IsMapped())
		{
			CurrentController().Character().StopAction();
			cout << "Mapped!!!!!!!!!" << endl;
		}
	}
}

bool PlayerProxy::UpdateByFrame(const AffineFrame & frame)
{
	AffineFrame tframe;
	// Caculate likilihood
	for (int i = 0, end = Controllers.size(); i < end; ++i)
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

	for (size_t i = 0; i < Controllers.size(); i++)
	{
		GetController(i).Character().SetOpticity(StateProbality(i));
	}

	int maxIdx = -1;
	auto currentP = StateProbality.maxCoeff(&maxIdx);
	if (maxIdx != CurrentIdx)
	{
		int oldIdx = CurrentIdx;
		CurrentIdx = maxIdx;
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

void DrawGuidingVectors(const BlockArmature & barmature, const AffineFrame & frame, const Color & color, const Matrix4x4 & world, float thinkness = 0.015f)
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
			auto& bone = frame[block->Joints.back()->ID()];
			XMVECTOR ep = bone.GblTranslation;

			auto& pbone = frame[block->parent()->Joints.back()->ID()];
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

void PlayerProxy::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	AffineFrame charaFrame;

	if (g_DebugLocalMotion && g_DebugView)
	{
		for (auto& controller : Controllers)
		{
			if (!controller.IsReady)
				continue;
			auto& chara = controller.Character();
			auto& action = controller.Character().Behavier()[g_DebugLocalMotionAction[chara.Name]];
			action.GetFrameAt(charaFrame, current_time);
			auto world = chara.GlobalTransformMatrix();
			DrawArmature(chara.Armature(), charaFrame, DirectX::Colors::LimeGreen.v, world, g_DebugArmatureThinkness / chara.GetGlobalTransform().Scale.x);
			DrawGuidingVectors(chara.Behavier().Blocks(), charaFrame, DirectX::Colors::Pink.v, world);
		}
	}

	if (!playerSelector) return;
	auto& player = *playerSelector;

	Color color = DirectX::Colors::Yellow.v;

	if (player.IsTracked())
	{
		const auto& frame = player.PullLatestFrame();
		if (&frame == nullptr) return;

		if (IsMapped())
			color.A(0.3f);

		DrawArmature(*player.BodyArmature, frame, color);
	}

	// IsMapped() && 
	if (IsMapped() && g_DebugView)
	{
		//auto& controller = this->CurrentController().Character();
		for (auto& controller : Controllers)
		{
			if (!controller.IsReady)
				continue;

			auto& chara = controller.Character();

			//DirectX::Visualizers::g_PrimitiveDrawer.SetView(chara.GlobalTransformMatrix());
			auto pBinding = dynamic_cast<BlockizedSpatialInterpolateQuadraticTransform*>(&controller.Binding());
			//if (pBinding)
			{
				auto world = chara.GlobalTransformMatrix();
				charaFrame = chara.GetCurrentFrame();

				//DrawGuidingVectors(chara.Behavier().Blocks(), charaFrame, DirectX::Colors::Crimson.v, world);

				pBinding->Render(context.Get(), DirectX::Colors::Crimson.v, world);
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
			const auto& frame = player.PullLatestFrame();
			if (&frame == nullptr) return;

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

CharacterController::~CharacterController()
{
	for (auto& p : m_Analyzers)
	{
		delete p.second;
	}
}

void CharacterController::Initialize(const IArmature & player, CharacterObject & character)
{
	IsReady = false;
	CurrentActionIndex = 0;
	SetSourceArmature(player);
	SetTargetCharacter(character);
}

const ArmatureTransform & CharacterController::Binding() const { return *m_pBinding; }

ArmatureTransform & CharacterController::Binding() { return *m_pBinding; }

void CharacterController::SetBinding(ArmatureTransform * pBinding)
{
	m_pBinding.reset(pBinding);
}

const CharacterObject & CharacterController::Character() const { return *m_pCharacter; }

CharacterObject & CharacterController::Character() { return *m_pCharacter; }

void CharacterController::UpdateTargetCharacter(const AffineFrame & frame) const
{
	auto& cframe = m_pCharacter->MapCurrentFrameForUpdate();
	m_pBinding->Transform(cframe, frame);

	float l = 100;
	for (auto& bone : frame)
	{
		if (bone.GblTranslation.y < l)
			l = bone.GblTranslation.y;
	}

	auto pos = frame[0].GblTranslation - MapRefPos + CMapRefPos;

	m_pCharacter->SetPosition(pos);
	m_pCharacter->ReleaseCurrentFrameFrorUpdate();
}

ClipInfo & CharacterController::GetAnimationInfo(const string & name) {
	return *m_Analyzers[name];
}

void CharacterController::SetSourceArmature(const IArmature & armature) {
	if (m_pBinding)
		m_pBinding->SetSourceArmature(armature);
}

void CharacterController::SetTargetCharacter(CharacterObject & object) {
	m_pCharacter = &object;
	if (m_pBinding)
		m_pBinding->SetTargetArmature(object.Armature());
	auto & behavier = object.Behavier();
	PotientialFrame = object.Armature().default_frame();

	for (auto& anim : object.Behavier().Clips())
	{
		if (anim.Name == "die")
			anim.IsCyclic = false;
		else
			anim.IsCyclic = true;

	}

	// Try to unify rotation pivot direction
	auto& armature = object.Armature();
	typedef vector<Vector4, DirectX::AlignedAllocator<Vector4, alignof(DirectX::XMVECTOR)>> aligned_vector_of_vector4;
	aligned_vector_of_vector4 gbl_pivots(armature.size());
	bool gbl_def = true;
	for (auto& anim : object.Behavier().Clips())
	{
		using namespace DirectX;
		aligned_vector_of_vector4 pivots(armature.size());
		for (auto& frame : anim.GetFrameBuffer())
		{
			for (size_t i = 0; i < armature.size(); i++)
			{
				pivots[i] += frame[i].LclRotation.LoadA();
			}
		}

		if (gbl_def)
		{
			for (size_t i = 0; i < armature.size(); i++)
			{
				gbl_pivots[i] = DirectX::XMVector3Normalize(pivots[i].LoadA());
			}
			gbl_def = false;
		}
		else
		{
			for (size_t i = 0; i < armature.size(); i++)
			{
				XMVECTOR pivot = DirectX::XMVector3Normalize(pivots[i].LoadA());
				XMVECTOR gbl_pivot = gbl_pivots[i].Load();
				if (XMVector4Less(XMVector3Dot(gbl_pivot, pivot), XMVectorZero()))
				{
					for (auto& frame : anim.GetFrameBuffer())
					{
						frame[i].LclRotation.StoreA(-frame[i].LclRotation.LoadA());
					}
				}
			}
		}
	}

	using namespace concurrency;
	vector<task<void>> tasks;
	for (auto& anim : object.Behavier().Clips())
	{
		if (!anim.Cyclic())
			continue;

		m_Analyzers[anim.Name] = new ClipInfo(behavier.Blocks());

		auto& analyzer = m_Analyzers[anim.Name];

		analyzer->Phi = -1;
		//analyzer->Score = -1;
		analyzer->PcaCutoff = CharacterPcaCutoff;
		analyzer->EnergyCutoff = CharacterEnergyCutoff;

		tasks.emplace_back(analyzer->ComputeFromFramesAsync(anim.GetFrameBuffer()));
	}

	when_all(tasks.begin(), tasks.end()).then([this]() {
		float globalEnergyMax = 0;
		for (auto pInfo : adaptors::values(m_Analyzers))
		{
			globalEnergyMax = std::max(pInfo->Eb.maxCoeff(), globalEnergyMax);
		}

		auto& blocks = m_pCharacter->Behavier().Blocks();
		for (auto& pair : m_Analyzers)
		{
			auto& key = pair.first;
			auto analyzer = pair.second;
			auto& Eb = analyzer->Eb;
			Eb /= globalEnergyMax;
			analyzer->EnergyCutoff = CharacterEnergyCutoff;
			for (int i = 0; i < Eb.size(); i++)
			{
				if (Eb[i] > analyzer->EnergyCutoff)
				{
					analyzer->ActiveBlocks.push_back(i);
					++blocks[i]->ActiveActionCount;
					blocks[i]->ActiveActions.emplace_back(key);
				}
			}
		}

		cout << "==========Drive correlation for character \"" << m_pCharacter->Name << "\"=================== " << endl;

		cout << setprecision(2);

		std::vector<KinematicBlock*> activeBlocks;
		std::vector<KinematicBlock*> inactiveBlocks;

		// for all inactive blocks, try to sythesis their sutle local motion based on action blocks
		for (auto pBlock : blocks)
		{
			if (pBlock->ActiveActionCount == 0)
				inactiveBlocks.push_back(pBlock);
			else
				activeBlocks.push_back(pBlock);
		}

		Eigen::MatrixXf Xabpv(CLIP_FRAME_COUNT * m_Analyzers.size(), activeBlocks.size() * 9);

		int bid = 0; // active block index
		std::for_each(activeBlocks.begin(), activeBlocks.end(), [&, this](KinematicBlock* pBlock)
		{
			auto& block = *pBlock;

			cout << "{";
			for (auto pJoint : block.Joints)
			{
				cout << pJoint->Name() << ", ";
			}
			cout << "\b\b} : ";

			#pragma region Build Xabpv
			auto Bft = Xabpv.middleCols<9>(bid * 9);
			int cid = 0; // Clip index
			for (auto clipinfo : adaptors::values(m_Analyzers))
			{
				auto map = Eigen::Matrix<float, CLIP_FRAME_COUNT, 3, Eigen::RowMajor>::Map(clipinfo->Pvs.col(block.Index).data());
				auto displacement = Bft.block<CLIP_FRAME_COUNT, 3>(cid*CLIP_FRAME_COUNT, 0);
				displacement = map;

				ExpandQuadraticTerm(Bft, 3);
				++cid;
			}
			++bid;
			#pragma endregion 

			block.X.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
			block.Pd.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, 9);
			for (size_t i = 0; i < block.ActiveActionCount; i++)
			{
				auto &clip = *m_Analyzers[block.ActiveActions[i]];
				auto &anim = m_pCharacter->Behavier()[block.ActiveActions[i]];
				float frametime = (float)(anim.Duration.count() / CLIP_FRAME_COUNT);

				block.X.middleRows<CLIP_FRAME_COUNT>(i*CLIP_FRAME_COUNT) = clip.Xbs[block.Index];

				auto map = Eigen::Matrix<float, CLIP_FRAME_COUNT, 3, Eigen::RowMajor>::Map(clip.Pvs.col(block.Index).data());
				auto displacement = block.Pd.block<CLIP_FRAME_COUNT, 3>(i*CLIP_FRAME_COUNT, 0);
				displacement = map;

				ExpandQuadraticTerm(block.Pd, 3);

				//float dNorm = sqrtf(displacement.rowwise().squaredNorm().maxCoeff());
				//displacement /= dNorm;

				//auto velocity = block.Pd.block<CLIP_FRAME_COUNT, 3>(i*CLIP_FRAME_COUNT, 3);
				////auto acceleration = block.Pd.block<CLIP_FRAME_COUNT, 3>(i*CLIP_FRAME_COUNT, 6);
				////// caculate velocity as center difference
				//velocity.row(0) = displacement.row(1) - displacement.row(CLIP_FRAME_COUNT - 1);
				//velocity.row(CLIP_FRAME_COUNT-1) = displacement.row(0) - displacement.row(CLIP_FRAME_COUNT-2);
				////velocity.row(0) = 2 * (displacement.row(1) - displacement.row(0));
				////velocity.row(CLIP_FRAME_COUNT - 1) = 2 * (displacement.row(CLIP_FRAME_COUNT - 1) - displacement.row(CLIP_FRAME_COUNT - 2));

				//velocity.middleRows<CLIP_FRAME_COUNT - 2>(1) = displacement.middleRows<CLIP_FRAME_COUNT - 2>(2) - displacement.middleRows<CLIP_FRAME_COUNT - 2>(0);

				////float vNorm = sqrtf(velocity.rowwise().squaredNorm().maxCoeff());
				//velocity /= 2 * frametime;



				auto corr = CreatePcaCcaMap(clip.PerceptiveVectorReconstructor[block.Index], block.Pd.middleRows<CLIP_FRAME_COUNT>(i*CLIP_FRAME_COUNT), block.X.middleRows<CLIP_FRAME_COUNT>(i*CLIP_FRAME_COUNT),
					0.01 * PcaCutoff, PcaCutoff);

				cout << '(' << block.ActiveActions[i] << " [" << clip.Eb[block.Index] << "] : " << corr << "),";
				//// caculate acceleration as center difference of velocity
				//acceleration.row(0) = 2 * (velocity.row(1) - velocity.row(0));
				//acceleration.row(CLIP_FRAME_COUNT - 1) = 2 * (velocity.row(CLIP_FRAME_COUNT - 1) - velocity.row(CLIP_FRAME_COUNT - 2));
				//acceleration.middleRows<CLIP_FRAME_COUNT - 2>(1) = velocity.middleRows<CLIP_FRAME_COUNT - 2>(2) - velocity.middleRows<CLIP_FRAME_COUNT - 2>(0);
			}

			//Cca<float> cca;
			//cca.compute(block.Pd, block.X);
			//auto corr = cca.correlaltions().minCoeff();

			auto corr = CreatePcaCcaMap(block.PdDriver, block.Pd, block.X, 0.01 * PcaCutoff, PcaCutoff);
			if (corr < .001f)
			{
				block.PdDriver.Jx = -1;
				block.PdDriver.Jy = -1;
			}
			else
			{
				block.PdDriver.Jx = block.Index;
				block.PdDriver.Jy = block.Index;
			}
			//block.PvCorr.setIdentity(block.ActiveActionCount, block.ActiveActionCount);

			//Cca<float> cca;
			//for (size_t i = 0; i < block.ActiveActionCount; i++)
			//{
			//	auto &clipI = *m_Analyzers[block.ActiveActions[i]];
			//	for (int j = i + 1; j < block.ActiveActionCount; j++)
			//	{
			//		auto &clipJ = *m_Analyzers[block.ActiveActions[j]];
			//		//auto pvX = block.Pd.block<CLIP_FRAME_COUNT, 3>(i*CLIP_FRAME_COUNT, 0);
			//		//auto pvY = block.Pd.block<CLIP_FRAME_COUNT, 3>(j*CLIP_FRAME_COUNT, 0);
			//		//cca.compute(pvX, pvY);
			//		//float corr = cca.correlaltions().minCoeff();


			//		PcaCcaMap map;
			//		float corr = CreatePcaCcaMap(map,clipI.X, clipJ.X);
			//		block.PvCorr(i, j) = corr;
			//		block.PvCorr(j, i) = corr;
			//	}
			//}

			if (g_EnableDebugLogging)
			{
				ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name() + ".pd.csv");
				fout << block.Pd.format(CSVFormat);
				fout.close();

				fout.open("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name() + ".x.csv");
				fout << block.X.format(CSVFormat);
				fout.close();

				fout.open("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name() + ".pvcorr.csv");
				fout << block.PvCorr.format(CSVFormat);
				fout.close();
			}

			cout << "\b | [" << corr << ']';
			cout << endl;

		});

		if (g_EnableDebugLogging)
		{
			ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_Xabpv.pd.csv");
			fout << Xabpv.format(CSVFormat);
			fout.close();
		}

		for (auto pBlock : inactiveBlocks)
		{
			auto &block = *pBlock;

			int subAcitveClipCount = 0;
			for (auto clipinfo : adaptors::values(m_Analyzers))
			{
				if (clipinfo->Eb[block.Index] > CharacterEnergyCutoff2)
				{
					++subAcitveClipCount;
				}
			}

			if (subAcitveClipCount == 0)
			{
				block.PdDriver.Jx = block.PdDriver.Jy = -1;
				block.PvDriveScore = -1;
				continue;
			}

			int cid = 0, cid2 = 0; // Clip index
			block.X.resize(subAcitveClipCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
			block.Pd.resize(subAcitveClipCount*CLIP_FRAME_COUNT, Xabpv.cols());

			for (auto clipinfo : adaptors::values(m_Analyzers))
			{
				if (clipinfo->Eb[block.Index] > CharacterEnergyCutoff2)
				{
					block.X.middleRows<CLIP_FRAME_COUNT>(cid*CLIP_FRAME_COUNT) = clipinfo->Xbs[block.Index];
					block.Pd.middleRows<CLIP_FRAME_COUNT>(cid*CLIP_FRAME_COUNT) = Xabpv.middleRows<CLIP_FRAME_COUNT>(cid2*CLIP_FRAME_COUNT);
					++cid;
				}
				++cid2;
			}

			auto corr = block.PdDriver.CreateFrom(block.Pd, block.X, 0.01 * PcaCutoff, PcaCutoff);
			block.PvDriveScore = corr;

			if (g_EnableDebugLogging)
			{
				ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name() + ".x.csv");
				fout << block.X.format(CSVFormat);
				fout.close();
			}

			cout << "[Inactive] {";
			for (auto pJoint : block.Joints)
			{
				cout << pJoint->Name() << ", ";
			}
			cout << "\b\b} : " << corr << endl;

			block.PdDriver.Jy = block.Index;
			block.PdDriver.Jx = -2;
		}

		cout << "=================================================================" << endl;

		auto pBinding = make_unique<BlockizedCcaArmatureTransform>(&blocks, &blocks);
		pBinding->pInputExtractor.reset(new BlockEndEffectorGblPosQuadratic());
		//pBinding->pInputExtractor.reset(new BlockEndEffectorFeatureExtractor<InputFeature>(true));
		pBinding->pOutputExtractor.reset(new BlockAllJointsFeatureExtractor<CharacterFeature>(false));

		//auto pBinding = make_unique<BlockizedSpatialInterpolateQuadraticTransform>(&m_Analyzers,&blocks, &blocks);

		for (auto& pBlock : blocks)
		{
			auto& block = *pBlock;
			//if (block.Index == 0)
			//	continue;
			if (block.ActiveActionCount > 0)
			{
				pBinding->Maps.emplace_back(block.PdDriver);
			}
		}
		// The dependent block must be specified after independent block
		for (auto& pBlock : blocks)
		{
			auto& block = *pBlock;
			//if (block.Index == 0)
			//	continue;
			if (block.ActiveActionCount == 0 && block.PvDriveScore > 0.5f)
			{
				pBinding->Maps.emplace_back(block.PdDriver);
			}
		}


		m_pSelfBinding = move(pBinding);
		IsReady = true;
	});
}

struct SkeletonBlock
{
	DirectX::XMVECTOR Scale;
	DirectX::XMVECTOR CenterFromParentCenter;
	JointType SkeletonJoint;
	int ParentBlockIndex;
};

const SkeletonBlock g_SkeletonBlocks[] = {
	{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f }, JointType_SpineMid, -1 },      //  0 - lower torso
	{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_SpineShoulder, 0 },  //  1 - upper torso

	{ { 0.25f, 0.25f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Neck, 1 },         //  2 - neck
	{ { 0.5f, 0.5f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Head, 2 },            //  3 - head

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderLeft, 1 },  //  4 - Left shoulderblade
	{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowLeft, 4 },     //  5 - Left upper arm
	{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristLeft, 5 },     //  6 - Left forearm
	{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandLeft, 6 },      //  7 - Left hand

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderRight, 1 },//   8 - Right shoulderblade
	{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowRight, 8 },    //   9 - Right upper arm
	{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristRight, 9 },    //  10 - Right forearm
	{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandRight, 10 },    //  11 - Right hand

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipLeft, 0 },       //  12 - Left hipblade
	{ { .4f, 1.0f, .4f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeLeft, 12 },         //  13 - Left thigh
	{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleLeft, 13 },        //  14 - Left calf
																							 //{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootLeft , 14 },     //     - Left foot

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipRight, 0 },     //  15  - Right hipblade
	{ { .4f, 1.0f, .4f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeRight, 15 },       //  16  - Right thigh
	{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleRight, 16 },       //  17  - Right calf
																							 //{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootRight, 18 },     //      - Right foot

};

const UINT BLOCK_COUNT = _countof(g_SkeletonBlocks);
const UINT BODY_COUNT = 6;
const UINT JOINT_COUNT = 25;

