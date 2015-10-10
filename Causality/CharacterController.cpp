#include "pch_bcl.h"
#include <tinyxml2.h>
#include "CharacterController.h"
#include "CharacterObject.h"
#include "AnimationAnalyzer.h"
#include <boost\format.hpp>
#include "PcaCcaMap.h"
#include "Settings.h"
#include "EigenExtension.h"
#include "ArmatureTransforms.h"
#include "RegressionModel.h"
//#pragma warning (disable:4297)
//#include <dlib\optimization\optimization.h>
#pragma warning (disable:4554)
#include <unsupported\Eigen\CXX11\Tensor>
#include "QudraticAssignment.h"

using namespace Causality;
using namespace std;
using namespace Eigen;
using namespace DirectX;
using namespace ArmaturePartFeatures;

//typedef dlib::matrix<double, 0, 1> dlib_vector;

extern Eigen::RowVector3d g_NoiseInterpolation;
const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

#define BEGIN_TO_END(range) range.begin(), range.end()

bool ReadGprParamXML(tinyxml2::XMLElement * blockSetting, Eigen::Vector3d &param);
void InitGprXML(tinyxml2::XMLElement * settings, const std::string & blockName, gaussian_process_regression& gpr);

void InitializeExtractor(AllJointRltLclRotLnQuatPcad& ft, const ShrinkedArmature& parts)
{
	// Init 
	ft.InitPcas(parts.size());
	ft.SetDefaultFrame(parts.Armature().default_frame());
	for (auto part : parts)
	{
		ft.SetPca(part->Index, part->ChainPcaMatrix, part->ChainPcaMean);
	}
}

class SelfLocalMotionTransform : public ArmatureTransform
{
public:
	const ShrinkedArmature * pBlockArmature;
	std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> * pHandles;

	mutable EndEffector<InputFeature>	inputExtractor;
	mutable AllJointRltLclRotLnQuatPcad outputExtractor;

	mutable MatrixXd m_Xs;

	SelfLocalMotionTransform(ShrinkedArmature * pBlocks)
		: pBlockArmature(pBlocks), inputExtractor(true), pHandles(nullptr)
	{
		pSource = &pBlockArmature->Armature();
		pTarget = &pBlockArmature->Armature();
		m_Xs.resize(pBlockArmature->size(), g_PvDimension);

		InitializeExtractor(outputExtractor, *pBlockArmature);
	}

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		const auto& blocks = *pBlockArmature;
		RowVectorXd X, Y;

		for (auto& block : blocks)
		{
			RowVectorXf xf = inputExtractor.Get(*block, source_frame);
			RowVectorXf yf;
			X = xf.cast<double>();

			if (block->ActiveActionCount > 0)
			{
				block->PdGpr.get_expectation(X, &Y);
				yf = Y.cast<float>();
				yf *= block->Wx.cwiseInverse().asDiagonal();

				outputExtractor.Set(*block, target_frame, yf);
			}
			else if (block->SubActiveActionCount > 0)
			{

			}
		}

		target_frame.RebuildGlobal(*pTarget);
	}

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) const
	{
		//if (!g_UseVelocity)
		//{
		//	Transform(target_frame, source_frame);
		//	return;
		//}

		const auto& blocks = *pBlockArmature;
		RowVectorXd X(g_PvDimension), Y;
		double semga = 1000;
		RowVectorXf yf;

		std::vector<RowVectorXd> Xabs;
		for (auto& block : blocks)
		{
			//X[0] *= 13;

			if (block->ActiveActionCount > 0)
			{
				RowVectorXf xf = inputExtractor.Get(*block, source_frame);
				RowVectorXf xfl = inputExtractor.Get(*block, last_frame);
				yf = outputExtractor.Get(*block, target_frame);

				X.segment<3>(0) = xf.cast<double>();

				auto Xd = X.segment<3>(0);
				auto uXd = block->PdGpr.uX.segment<3>(0);

				//auto uXv = block->PdGpr.uX.segment<3>(3);
				//Xv = (Xv - uXv).array() * g_NoiseInterpolation.array() + uXv.array();

				Xd = (Xd - uXd).array();

				double varZ = (Xd.array() * (g_NoiseInterpolation.array() - 1.0)).cwiseAbs2().sum();
				// if no noise
				varZ = std::max(varZ, 1e-5);

				Xd = Xd.array() * g_NoiseInterpolation.array() + uXd.array();

				RowVector3d Xld = (xfl.cast<double>() - uXd).array() * g_NoiseInterpolation.array() + uXd.array();

				if (g_UseVelocity && g_PvDimension == 6)
				{
					auto Xv = X.segment<3>(3);
					Xv = (Xd - Xld) / (frame_time * g_FrameTimeScaleFactor);
				}

				xf = X.cast<float>();
				if (pHandles)
				{
					pHandles->at(block->Index).first = Vector3(xf.data());
					if (g_UseVelocity && g_PvDimension == 6)
					{
						pHandles->at(block->Index).second = Vector3(xf.data() + 3);
					}
					else
					{
						pHandles->at(block->Index).second = Vector3::Zero;
					}
				}

				m_Xs.row(block->Index) = X;
				Xabs.emplace_back(X);

				// Beyesian majarnlize over X
				//size_t detail = 3;
				//MatrixXd Xs(detail*2+1,g_PvDimension), Ys;
				//Xs = gaussian_sample(X, X, detail);

				//VectorXd Pxs = (Xs - X.replicate(detail * 2 + 1, 1)).cwiseAbs2().rowwise().sum();
				//Pxs = (-Pxs.array() / semga).exp();

				//VectorXd Py_xs = block->PdGpr.get_expectation_and_likelihood(Xs, &Ys);
				//Py_xs = (-Py_xs.array()).exp() * Pxs.array();
				//Py_xs /= Py_xs.sum();

				//Y = (Ys.array() * Py_xs.replicate(1, Ys.cols()).array()).colwise().sum();

				MatrixXd covObsr(g_PvDimension, g_PvDimension);
				covObsr.setZero();
				covObsr.diagonal() = g_NoiseInterpolation.replicate(1, g_PvDimension / 3).transpose() * varZ;

				//block->PdGpr.get_expectation_from_observation(X, covObsr, &Y);
				//block->PdGpr.get_expectation(X, &Y);
				auto baseRot = target_frame[block->parent()->Joints.back()->ID].GblRotation;
				block->PdStyleIk.SetBaseRotation(baseRot);
				block->PdStyleIk.SetChain(block->Joints, target_frame);
				block->PdStyleIk.SetGplvmWeight(block->Wx.cast<double>());
				//auto yc = yf;
				//yf = Y.cast<float>();
				//yf.array() *= block->Wx.cwiseInverse().array().transpose();

				//block->PdStyleIk.SetHint();
				if (!g_UseVelocity)
					Y = block->PdStyleIk.Apply(X.transpose());
				else
					Y = block->PdStyleIk.Apply(X.leftCols<3>().transpose(), X.segment<3>(3).transpose().eval());

				//block->PdStyleIk.SetGoal(X.leftCols<3>());

				//auto scoref = block->PdStyleIk.objective(X, yf.cast<double>());
				//auto scorec = block->PdStyleIk.objective(X, yc.cast<double>());
				//std::cout << "Gpr score : " << scoref << " ; Cannonical score : " << scorec << endl;
				//auto ep = block->PdStyleIk.EndPosition(yf.cast<double>());

				outputExtractor.Set(*block, target_frame, Y.cast<float>());
				for (int i = 0; i < block->Joints.size(); i++)
				{
					target_frame[block->Joints[i]->ID].UpdateGlobalData(target_frame[block->Joints[i]->parent()->ID]);
				}

				auto ep2 = target_frame[block->Joints.back()->ID].GblTranslation -
					target_frame[block->Joints[0]->parent()->ID].GblTranslation;

				//break;
			}
		}

		// Fill Xabpv
		if (g_EnableDependentControl)
		{
			RowVectorXd Xabpv;
			Xabpv.resize(Xabs.size() * g_PvDimension);
			int i = 0;
			for (const auto& xab : Xabs)
			{
				auto Yi = Xabpv.segment(i * g_PvDimension, g_PvDimension);
				Yi = xab;
				++i;
			}


			for (auto& block : blocks)
			{
				if (block->ActiveActionCount == 0 && block->SubActiveActionCount > 0)
				{
					auto _x = (Xabpv.cast<float>() - block->PdCca.uXpca) * block->PdCca.pcX;

					auto lk = block->PdGpr.get_expectation_and_likelihood(_x.cast<double>(), &Y);

					yf = Y.cast<float>();
					yf *= block->Wx.cwiseInverse().asDiagonal();

					outputExtractor.Set(*block, target_frame, yf);
				}

			}
		}

		target_frame[0].LclTranslation = source_frame[0].LclTranslation;
		target_frame[0].GblTranslation = source_frame[0].GblTranslation;
		target_frame.RebuildGlobal(*pTarget);
	}

	void Visualize();

	//virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneVelocityFrame& source_velocity, float frame_time) const
	//{
	//	// redirect to pose transform
	//	Transform(target_frame, source_frame);
	//	return;
	//}
};


CharacterController::~CharacterController()
{
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

void CharacterController::UpdateTargetCharacter(const BoneHiracheryFrame & frame, const BoneHiracheryFrame & lastframe, double deltaTime) const
{
	auto& cframe = m_pCharacter->MapCurrentFrameForUpdate();
	m_pBinding->Transform(cframe, frame, lastframe, deltaTime);

	float l = 100;
	for (auto& bone : frame)
	{
		if (bone.GblTranslation.y < l)
			l = bone.GblTranslation.y;
	}

	using namespace DirectX;

	//auto pos = frame[0].GblTranslation - MapRefPos + CMapRefPos;
	auto pos = Character().GetPosition() + frame[0].GblTranslation - LastPos;

	LastPos = frame[0].GblTranslation;

	// CrefRot * Yaw(HrefRot^-1 * HRot)
	auto rot = XMQuaternionMultiply(
		XMQuaternionConjugate(XMLoad(MapRefRot)),
		XMLoadA(frame[0].GblRotation));

	// extract Yaw rotation only, it's a bad hack here
	rot = XMQuaternionLn(rot);
	rot = XMVectorMultiply(rot, g_XMIdentityR1.v);
	rot = XMQuaternionExp(rot);

	rot = XMQuaternionMultiply(XMLoad(CMapRefRot), rot);


	m_pCharacter->SetPosition(pos);
	m_pCharacter->SetOrientation(rot);

	m_pCharacter->ReleaseCurrentFrameFrorUpdate();
}

const std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>>& Causality::CharacterController::PvHandles() const
{
	return m_PvHandles;
}

ClipInfo & CharacterController::GetClipInfo(const string & name) {
	auto itr = std::find_if(BEGIN_TO_END(m_Clipinfos), [&name](const auto& clip) {
		return clip.ClipName == name;
	});
	if (itr != m_Clipinfos.end())
	{
		return *itr;
	}
	else
	{
		throw std::out_of_range("given name doesn't exist");
	}
}

void CharacterController::SetSourceArmature(const IArmature & armature) {
	if (m_pBinding)
		m_pBinding->SetSourceArmature(armature);
}

template <class DerivedX, class DerivedY, typename Scalar>
void GetVolocity(_In_ const Eigen::DenseBase<DerivedX>& displacement, _Out_ Eigen::DenseBase<DerivedY>& velocity, Scalar frame_time, bool closeLoop = false)
{
	velocity.middleRows(1, CLIP_FRAME_COUNT - 2) = displacement.middleRows(2, CLIP_FRAME_COUNT - 2) - displacement.middleRows(0, CLIP_FRAME_COUNT - 2);

	if (!closeLoop)
	{
		velocity.row(0) = 2 * (displacement.row(1) - displacement.row(0));
		velocity.row(CLIP_FRAME_COUNT - 1) = 2 * (displacement.row(CLIP_FRAME_COUNT - 1) - displacement.row(CLIP_FRAME_COUNT - 2));
	}
	else
	{
		velocity.row(0) = displacement.row(1) - displacement.row(CLIP_FRAME_COUNT - 1);
		velocity.row(CLIP_FRAME_COUNT - 1) = displacement.row(0) - displacement.row(CLIP_FRAME_COUNT - 2);
	}

	velocity /= (2 * frame_time);
}

void CharacterController::SetTargetCharacter(CharacterObject & chara) {
	m_pCharacter = &chara;
	if (m_pBinding)
		m_pBinding->SetTargetArmature(chara.Armature());
	auto & behavier = chara.Behavier();
	PotientialFrame = chara.Armature().default_frame();
	m_PvHandles.resize(chara.Armature().size());

	m_pCharacter->Behavier().ArmatureParts().ComputeWeights();

	for (auto& anim : chara.Behavier().Clips())
	{
		//if (anim.Name == "die")
		//	anim.IsCyclic = false;
		//else
		anim.IsCyclic = true;
	}

	// Try to unify rotation pivot direction
	auto& armature = chara.Armature();
	typedef vector<Vector4, DirectX::AlignedAllocator<Vector4, alignof(DirectX::XMVECTOR)>> aligned_vector_of_vector4;
	aligned_vector_of_vector4 gbl_pivots(armature.size());
	bool gbl_def = true;
	for (auto& anim : chara.Behavier().Clips())
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
			//for (size_t i = 0; i < armature.size(); i++)
			//{
			//	XMVECTOR pivot = DirectX::XMVector3Normalize(pivots[i].LoadA());
			//	XMVECTOR gbl_pivot = gbl_pivots[i].Load();
			//	if (XMVector4Less(XMVector3Dot(gbl_pivot, pivot), XMVectorZero()))
			//	{
			//		for (auto& frame : anim.GetFrameBuffer())
			//		{
			//			frame[i].LclRotation.StoreA(-frame[i].LclRotation.LoadA());
			//		}
			//	}
			//}
		}
	}

	using namespace concurrency;
	vector<task<void>> tasks;
	m_Clipinfos.reserve(chara.Behavier().Clips().size());
	for (auto& anim : chara.Behavier().Clips())
	{
		if (!anim.Cyclic())
			continue;

		m_Clipinfos.emplace_back(behavier.ArmatureParts());
		auto& clipinfo = m_Clipinfos.back();

		clipinfo.ClipName = anim.Name;
		clipinfo.Phi = -1;
		//analyzer->Score = -1;
		clipinfo.PcaCutoff = g_CharacterPcaCutoff;
		clipinfo.ActiveEnergyThreshold = g_CharacterActiveEnergy;

		auto & frames = anim.GetFrameBuffer();
		tasks.emplace_back(create_task([&clipinfo, &frames]() {
			clipinfo.ComputeFromFrames(frames);
		}));
	}

	when_all(tasks.begin(), tasks.end()).then([this]() {

		cout << setprecision(4) << setw(6);

		float globalEnergyMax = 0;

		tinyxml2::XMLDocument paramdoc;
		tinyxml2::XMLElement* settings = nullptr;
		string paramFileName = "CharacterAnalayze\\" + m_pCharacter->Name + ".param.xml";
		string settingName = str(boost::format("clip_rasterize_%1%") % CLIP_FRAME_COUNT);

		for (auto& clipinfo : m_Clipinfos)
		{
			auto &clipname = clipinfo.ClipName;
			assert(clipinfo.IsReady);
			globalEnergyMax = std::max(clipinfo.Eb.maxCoeff(), globalEnergyMax);
			cout << "DimX in Clip [" << clipname << "] " << clipinfo.DimX << endl;
			cout << clipinfo.Eb << endl;
			settingName += '_' + clipname;
		}

		if (g_LoadCharacterModelParameter)
		{
			auto error = paramdoc.LoadFile(paramFileName.c_str());
			tinyxml2::XMLElement* paramStore = nullptr;
			if (error == tinyxml2::XML_SUCCESS)
			{
				paramStore = paramdoc.RootElement();
				//if (!strcmp(paramStore->Name(),"param_store"))
				//{
				//	paramStore = nullptr;
				//	paramdoc.Clear();
				//}
			}

			if (paramStore == nullptr)
			{
				paramStore = paramdoc.NewElement("param_store");
				paramdoc.InsertFirstChild(paramStore);
			}

			settings = paramStore->FirstChildElement(settingName.c_str());

			if (settings == nullptr)
			{
				settings = paramdoc.NewElement(settingName.c_str());
				paramStore->InsertEndChild(settings);
			}
		}


		auto& blocks = m_pCharacter->Behavier().ArmatureParts();
		for (auto& clipinfo : m_Clipinfos)
		{
			auto& key = clipinfo.ClipName;
			auto& Eb = clipinfo.Eb;
			Eb /= globalEnergyMax;
			clipinfo.ActiveEnergyThreshold = g_CharacterActiveEnergy;
			cout << Eb << endl;
			for (int i = 0; i < Eb.size(); i++)
			{
				if (Eb[i] > clipinfo.ActiveEnergyThreshold)
				{
					clipinfo.ActiveParts.push_back(i);
					++blocks[i]->ActiveActionCount;
					blocks[i]->ActiveActions.emplace_back(key);
				}
				else if (Eb[i] > clipinfo.SubactiveEnergyThreshold)
				{
					clipinfo.SubactiveParts.push_back(i);
					++blocks[i]->SubActiveActionCount;
					blocks[i]->SubActiveActions.push_back(key);
				}
			}
		}

		cout << "==========Drive correlation for character \"" << m_pCharacter->Name << "\"=================== " << endl;

		vector<ArmaturePart*> activeBlocks;
		vector<ArmaturePart*> subActiveBlocks;
		//auto activeBlocks = blocks | adaptors::filtered([](auto pBlock) {return pBlock->ActiveActionCount > 0});
		//auto subActiveBlocks = blocks | adaptors::filtered([](auto pBlock) {return pBlock->ActiveActionCount == 0});

		// for all inactive blocks, try to sythesis their sutle local motion based on action blocks
		for (auto pBlock : blocks)
		{
			if (pBlock->ActiveActionCount == 0)
				subActiveBlocks.push_back(pBlock);
			else
				activeBlocks.push_back(pBlock);
		}

		Eigen::MatrixXf Xabpv(CLIP_FRAME_COUNT * m_Clipinfos.size(), size(activeBlocks) * 6);

		int bid = 0; // active block index
					 //concuncy::for_each(activeBlocks.begin(), activeBlocks.end(), [&, this](ArmaturePart* pBlock)
		for (auto pBlock : activeBlocks)
		{
			auto& block = *pBlock;

			auto bj = block.Index;
			auto maxItr = std::max_element(BEGIN_TO_END(m_Clipinfos), [bj](const auto& lhs, const auto& rhs)
			{
				return lhs.DimX[bj] < rhs.DimX[bj];
			});
			auto maxDim = maxItr->DimX[bj];

			cout << "{";
			for (auto pJoint : block.Joints)
			{
				cout << pJoint->Name << ", ";
			}

			cout << "\b\b} [" << maxDim << "] : ";

#pragma region Build Xabpv
			auto Bft = Xabpv.middleCols(bid * g_PvDimension, g_PvDimension);
			int cid = 0; // Clip index
			for (auto& clipinfo : m_Clipinfos)
			{
				auto& anim = m_pCharacter->Behavier()[clipinfo.ClipName];

				auto map = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(clipinfo.Pvs.col(block.Index).data(), CLIP_FRAME_COUNT, 3);
				auto displacement = Bft.block(cid*CLIP_FRAME_COUNT, 0, CLIP_FRAME_COUNT, 3);
				displacement = map;
				float frametime = (float)(anim.Duration.count() / CLIP_FRAME_COUNT);
				if (g_UseVelocity && g_PvDimension == 6)
				{
					auto velocity = Bft.block(cid*CLIP_FRAME_COUNT, 3, CLIP_FRAME_COUNT, 3);
					GetVolocity(displacement, velocity, frametime * g_FrameTimeScaleFactor);
				}
				//ExpandQuadraticTerm(Bft, 3);

				++cid;
			}
			++bid;
#pragma endregion 

			block.X.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
			block.Pd.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, g_PvDimension);
			for (size_t i = 0; i < block.ActiveActionCount; i++)
			{
				auto &clip = GetClipInfo(block.ActiveActions[i]);
				auto &anim = m_pCharacter->Behavier()[block.ActiveActions[i]];
				float frametime = (float)(anim.Duration.count() / CLIP_FRAME_COUNT);

				block.X.middleRows(i*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = clip.Xbs[block.Index];

				auto map = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(clip.Pvs.col(block.Index).data(), CLIP_FRAME_COUNT, 3);
				auto displacement = block.Pd.block(i*CLIP_FRAME_COUNT, 0, CLIP_FRAME_COUNT, 3);
				displacement = map;

				//ExpandQuadraticTerm(block.Pd, 3);

				//float dNorm = sqrtf(displacement.rowwise().squaredNorm().maxCoeff());
				//displacement /= dNorm;

				if (g_UseVelocity && g_PvDimension == 6)
				{
					auto velocity = block.Pd.block(i*CLIP_FRAME_COUNT, 3, CLIP_FRAME_COUNT, 3);
					GetVolocity(displacement, velocity, frametime * g_FrameTimeScaleFactor);
				}
				////float vNorm = sqrtf(velocity.rowwise().squaredNorm().maxCoeff());



				auto corr = CreatePcaCcaMap(clip.PerceptiveVectorReconstructor[block.Index], block.Pd.middleRows(i*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT), block.X.middleRows(i*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT),
					0.01 * g_CharacterPcaCutoff, g_CharacterPcaCutoff);

				cout << '(' << block.ActiveActions[i] << " [" << clip.Eb[block.Index] << "] : " << corr << "),";
				//// caculate acceleration as center difference of velocity
				//acceleration.row(0) = 2 * (velocity.row(1) - velocity.row(0));
				//acceleration.row(CLIP_FRAME_COUNT - 1) = 2 * (velocity.row(CLIP_FRAME_COUNT - 1) - velocity.row(CLIP_FRAME_COUNT - 2));
				//acceleration.middleRows<CLIP_FRAME_COUNT - 2>(1) = velocity.middleRows<CLIP_FRAME_COUNT - 2>(2) - velocity.middleRows<CLIP_FRAME_COUNT - 2>(0);
			}

			//Cca<float> cca;
			//cca.compute(block.Pd, block.X);
			//auto corr = cca.correlaltions().minCoeff();
			block.LimitX.resize(2, block.X.cols());
			block.LimitX.row(0) = block.X.colwise().minCoeff();
			block.LimitX.row(1) = block.X.colwise().maxCoeff();

			block.ChainPca.compute(block.X * block.Wx.asDiagonal(), true);
			int d = block.ChainPca.reducedRank(0.01f);
			block.ChainPcadDim = d;
			block.ChainPcaMean = block.ChainPca.mean().array() * block.Wx.cwiseInverse().transpose().array();
			block.ChainPcaMatrix = block.Wx.asDiagonal() * block.ChainPca.components(d);

			float corr;
			if (g_UseStylizedIK)
			{
				block.PdGpr.initialize(block.Pd, block.ChainPca.coordinates(block.ChainPcadDim));

				// paramter caching 
				const auto&	blockName = block.Joints[0]->Name;
				auto &dframe = Character().Armature().default_frame();

				InitGprXML(settings, blockName, block.PdGpr);

				// initialize stylized IK for active chains
				block.PdStyleIk.SetChain(block.Joints, dframe);
				block.PdStyleIk.SetGplvmWeight(block.Wx.cast<double>());
				//block.PdStyleIk.SetYLimit(block.LimitX.cast<double>());

				auto &pca = block.ChainPca;
				auto& wx = block.Wx;
				auto d = block.ChainPcadDim;
				auto pPcaDecoder = std::make_unique<RelativeLnQuaternionPcaDecoder>();
				pPcaDecoder->meanY = block.ChainPcaMean.cast<double>();
				pPcaDecoder->pcaY = block.ChainPcaMatrix.cast<double>();
				pPcaDecoder->invPcaY = (pca.components(d).transpose() * wx.cwiseInverse().asDiagonal()).cast<double>();
				//MatrixXf valiad = (pca.components() * pca.components().transpose());

				pPcaDecoder->bases.reserve(block.Joints.size());
				for (auto joint : block.Joints)
				{
					auto jid = joint->ID;
					pPcaDecoder->bases.push_back(dframe[jid].LclRotation);
				}
				block.PdStyleIk.SetFeatureDecoder(move(pPcaDecoder));
				block.PdStyleIk.Gplvm() = block.PdGpr;

				cout << "Optimal param : " << block.PdGpr.get_parameters().transpose() << endl;
				corr = 1.0f;
			}
			else
			{
				corr = CreatePcaCcaMap(block.PdCca, block.Pd, block.X, 0.01 * g_CharacterPcaCutoff, g_CharacterPcaCutoff);
			}

			if (corr < .001f)
			{
				block.PdCca.Jx = -1;
				block.PdCca.Jy = -1;
			}
			else
			{
				block.PdCca.Jx = block.Index;
				block.PdCca.Jy = block.Index;
			}
			//block.PvCorr.setIdentity(block.ActiveActionCount, block.ActiveActionCount);

			//Cca<float> cca;
			//for (size_t i = 0; i < block.ActiveActionCount; i++)
			//{
			//	auto &clipI = *m_Clipinfos[block.ActiveActions[i]];
			//	for (int j = i + 1; j < block.ActiveActionCount; j++)
			//	{
			//		auto &clipJ = *m_Clipinfos[block.ActiveActions[j]];
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
				ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name + ".pd.csv");
				fout << block.Pd.format(CSVFormat);
				fout.close();

				fout.open("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name + ".x.csv");
				fout << block.X.format(CSVFormat);
				fout.close();

				fout.open("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name + ".pvcorr.csv");
				fout << block.PvCorr.format(CSVFormat);
				fout.close();
			}

			cout << "\b | [" << corr << ',' << block.PdCca.pcY.cols() << ']';
			cout << endl;

		}//);

		Eigen::MatrixXf Xij(CLIP_FRAME_COUNT, 3);
		Eigen::RowVectorXf uXij;
		PvDifMean.resize(activeBlocks.size(), activeBlocks.size());
		PvDifCov.resize(activeBlocks.size(), activeBlocks.size());
		for (int i = 0; i < activeBlocks.size(); i++)
		{
			for (int j = i + 1; j < activeBlocks.size(); j++)
			{
				PvDifMean(i, j).setZero();
				PvDifCov(i, j).setZero();
			}
		}

		for (auto& clipinfo : m_Clipinfos)
		{
			auto& pvs = clipinfo.Pvs;
			auto cabs = clipinfo.ActiveParts.size();
			clipinfo.PvDifMean.resize(clipinfo.ActiveParts.size(), clipinfo.ActiveParts.size());
			clipinfo.PvDifCov.resize(clipinfo.ActiveParts.size(), clipinfo.ActiveParts.size());

			for (int i = 0; i < activeBlocks.size(); i++)
			{
				int bi = activeBlocks[i]->Index;
				int ci = std::find(BEGIN_TO_END(clipinfo.ActiveParts), bi) - clipinfo.ActiveParts.begin();

				for (int j = i + 1; j < activeBlocks.size(); j++)
				{
					int bj = activeBlocks[j]->Index;
					int cj = std::find(BEGIN_TO_END(clipinfo.ActiveParts), bj) - clipinfo.ActiveParts.begin();

					auto pi = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(pvs.col(bi).data(), CLIP_FRAME_COUNT, 3);
					auto pj = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(pvs.col(bj).data(), CLIP_FRAME_COUNT, 3);

					Matrix3f covij = Xij.transpose() * Xij;
					Xij = pi - pj;
					uXij = Xij.colwise().mean();

					if (ci >= 0 && ci < cabs && cj >= 0 && cj < cabs)
					{
						clipinfo.PvDifMean(i, j) = uXij;
						clipinfo.PvDifCov(i, j) = covij;
					}

					PvDifMean(i, j) += uXij;
					Xij.rowwise() -= uXij;
					PvDifCov(i, j) += covij;
				}
			}
		}

		Pca<MatrixXf> pcaXabpv(Xabpv);
		int dXabpv = pcaXabpv.reducedRank(0.01);
		MatrixXf cordsXabpv = pcaXabpv.coordinates(dXabpv);
		XabpvT = pcaXabpv.components(dXabpv);
		uXabpv = pcaXabpv.mean();

		if (g_EnableDebugLogging)
		{
			ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_Xabpv.pd.csv");
			fout << Xabpv.format(CSVFormat);

			fout.close();
		}

		if (g_EnableDependentControl)
			for (auto pBlock : subActiveBlocks)
			{
				auto &block = *pBlock;

				auto subAcitveClipCount = block.SubActiveActionCount;
				if (subAcitveClipCount == 0)
				{
					block.PdCca.Jx = block.PdCca.Jy = -1;
					block.PvDriveScore = -1;
					continue;
				}

				int cid = 0, cid2 = 0; // Clip index
				block.X.resize(subAcitveClipCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
				block.Pd.resize(subAcitveClipCount*CLIP_FRAME_COUNT, cordsXabpv.cols());

				for (auto& clipinfo : m_Clipinfos)
				{
					if (clipinfo.Eb[block.Index] > g_CharacterSubactiveEnergy)
					{
						block.X.middleRows(cid*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = clipinfo.Xbs[block.Index];
						block.Pd.middleRows(cid*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = cordsXabpv.middleRows(cid2*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT);
						++cid;
					}
					++cid2;
				}

				block.LimitX.resize(2, block.X.cols());
				block.LimitX.row(0) = block.X.colwise().minCoeff();
				block.LimitX.row(1) = block.X.colwise().maxCoeff();

				block.ChainPca.compute(block.X * block.Wx.asDiagonal(), true);
				{
					int d = block.ChainPca.reducedRank(0.01f);
					block.ChainPcadDim = d;
					block.ChainPcaMean = block.ChainPca.mean();
					block.ChainPcaMatrix = block.ChainPca.components(d);
				}

				cout << "[Subactive] {";
				for (auto pJoint : block.Joints)
				{
					cout << pJoint->Name << ", ";
				}

				float corr = 0.0;
				if (!g_UseStylizedIK)
				{
					assert(!"this code pass is not valiad. as block.Pd is already Pca-ed here");
					corr = block.PdCca.CreateFrom(block.Pd, block.X, 0.01 * g_CharacterPcaCutoff, g_CharacterPcaCutoff);
					block.PvDriveScore = corr;
				}
				else
				{
					block.PdGpr.initialize(block.Pd, block.ChainPca.coordinates(block.ChainPcadDim));
					block.PdCca.uXpca = uXabpv;
					block.PdCca.pcX = XabpvT;
					// paramter caching 
					const auto&	blockName = block.Joints[0]->Name;

					InitGprXML(settings, blockName, block.PdGpr);
					corr = exp(-corr);
				}

				if (g_EnableDebugLogging)
				{
					ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name + ".x.csv");
					fout << block.X.format(CSVFormat);
					fout.close();
				}

				auto bid = block.Index;
				auto maxItr = std::max_element(m_Clipinfos.begin(), m_Clipinfos.end(), [bid](const auto& lhs, const auto& rhs)
				{
					return lhs.DimX[bid] < rhs.DimX[bid];
				});
				auto maxDim = maxItr->DimX[bid];

				cout << "\b\b} [" << maxDim << "] : " << corr << endl;

				block.PdCca.Jy = block.Index;
				block.PdCca.Jx = -2;
			}

		cout << "=================================================================" << endl;

		if (!g_UseStylizedIK)
		{
			auto pBinding = make_unique<BlockizedCcaArmatureTransform>(&blocks, &blocks);

			pBinding->pInputExtractor.reset(new EndEffectorGblPosQuadratized());
			//pBinding->pInputExtractor.reset(new EndEffector<InputFeature>(true));
			pBinding->pOutputExtractor.reset(new AllJoints<CharacterFeature>(false));

			//auto pBinding = make_unique<RBFInterpolationTransform>(&m_Clipinfos,&blocks, &blocks);

			for (auto& pBlock : blocks)
			{
				auto& block = *pBlock;
				//if (block.Index == 0)
				//	continue;
				if (block.ActiveActionCount > 0)
				{
					pBinding->Maps.emplace_back(block.PdCca);
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
					pBinding->Maps.emplace_back(block.PdCca);
				}
			}

			m_pSelfBinding = move(pBinding);
		}
		else
		{
			auto pBinding = make_unique<SelfLocalMotionTransform>(&blocks);
			pBinding->pHandles = &m_PvHandles;
			m_pSelfBinding = move(pBinding);
		}

		if (g_LoadCharacterModelParameter)
		{
			auto error = paramdoc.SaveFile(paramFileName.c_str());
			assert(error == tinyxml2::XML_SUCCESS);
		}

		IsReady = true;
	});
}


bool ReadGprParamXML(tinyxml2::XMLElement * blockSetting, Eigen::Vector3d &param)
{
	if (blockSetting && blockSetting->Attribute("alpha") && blockSetting->Attribute("beta") && blockSetting->Attribute("gamma"))
	{
		param(0) = blockSetting->DoubleAttribute("alpha");
		param(1) = blockSetting->DoubleAttribute("beta");
		param(2) = blockSetting->DoubleAttribute("gamma");
		return true;
	}
	return false;
}

void InitGprXML(tinyxml2::XMLElement * settings, const std::string & blockName, gaussian_process_regression& gpr)
{
	gaussian_process_regression::ParamType param;
	bool	paramSetted = false;
	if (g_LoadCharacterModelParameter)
	{
		auto blockSetting = settings->FirstChildElement(blockName.c_str());

		if (paramSetted = ReadGprParamXML(blockSetting, param))
		{
			gpr.set_parameters(param);
		}
	}

	if (!paramSetted)
	{
		gpr.optimze_parameters();
		param = gpr.get_parameters();

		if (g_LoadCharacterModelParameter)
		{
			auto blockSetting = settings->FirstChildElement(blockName.c_str());

			if (blockSetting == nullptr)
			{
				blockSetting = settings->GetDocument()->NewElement(blockName.c_str());
				settings->InsertEndChild(blockSetting);
			}

			blockSetting->SetAttribute("alpha", param[0]);
			blockSetting->SetAttribute("beta", param[1]);
			blockSetting->SetAttribute("gamma", param[2]);
		}
	}
}

// helper functions
extern void CaculateQuadraticDistanceMatrix(Eigen::Tensor<float, 4> &C, const ClipInfo& iclip, const ClipInfo& cclip);

//template <typename IdxType>
//IdxType experiment_fun(_In_ const gsl::array_view<IdxType>& indices)
//{
//	return indices[0];
//}

std::ostream& operator<<(std::ostream& os, const Joint& joint)
{
	os << joint.Name;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Joint* joint)
{
	os << joint->Name;
	return os;
}

template <class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T> &vec)
{
	cout << '{';
		for (auto& t : vec)
		{
			cout << t << ", ";
		}
	cout << "\b}";
	return os;
}

extern Matrix3f FindIsometricTransformXY(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y);

void FindPartToPartTransform(_Inout_ P2PTransform& transform, const InputClipInfo& iclip, const ClipInfo& cclip, size_t phi)
{
	int ju = transform.SrcIdx;
	int jc = transform.DstIdx;
	int T = CLIP_FRAME_COUNT;
	auto rawX = iclip.GetPartPvSequence(ju, phi, T);
	auto rawY = cclip.GetPartPvSequence(jc);

	if (g_PartAssignmentTransform == PAT_CCA)
	{
		//? Again!!! PvQrs vs Qrs

		PcaCcaMap map;
		map.CreateFrom(rawX, rawY, iclip.PcaCutoff, cclip.PcaCutoff);
		transform.HomoMatrix = map.TransformMatrix();
	}
	else if (g_PartAssignmentTransform == PAT_OneAxisRotation)
	{
		//RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
		//float err = (rawY - rawX * alpha.asDiagonal()).cwiseAbs2().sum();

		auto Transf = FindIsometricTransformXY(rawX, rawY);
		auto rank = rawX.cols();

		transform.HomoMatrix.setIdentity(4,4);
		transform.HomoMatrix.topLeftCorner(3,3) = Transf;
	}
	else if (g_PartAssignmentTransform == PAT_AnisometricScale)
	{
		RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
		float err = (rawY - rawY * alpha.asDiagonal()).cwiseAbs2().sum();

		transform.HomoMatrix.setIdentity(4, 4);
		transform.HomoMatrix.topLeftCorner(3, 3) = alpha.asDiagonal();
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

		transform.HomoMatrix.setIdentity(uX.size() + 1, uY.size() + 1);
		transform.HomoMatrix.topLeftCorner(uX.size(), uY.size()) = alpha.asDiagonal();
		transform.HomoMatrix.block(uX.size(), 0, 1, uY.size()) = -uX*alpha.asDiagonal() + uY;
	}
}

std::unique_ptr<ArmatureTransform> CreateControlTransform(CharacterController & controller, const InputClipInfo& iclip)
{
	assert(controller.IsReady && iclip.IsReady);

	// alias setup
	auto& character = controller.Character();
	auto& charaParts = character.Behavier().ArmatureParts();
	auto& userParts = iclip.ArmatureParts();
	size_t Jc = charaParts.size();
	auto& clips = character.Behavier().Clips();
	auto& clipinfos = controller.GetClipInfos();

	controller.CharacterScore = numeric_limits<float>::min();
	//auto& anim = character.Behavier()[DefaultAnimationSet];
	auto& anim = *character.CurrentAction();


	int T = iclip.Period;
	const std::vector<int> &Juk = iclip.ActiveParts;
	int Ti = iclip.TemproalSampleInterval;
	int Ts = T / Ti;

	RowVectorXf Eub;
	selectCols(iclip.Eb, Juk, &Eub);

	// Player Perceptive vector mean normalized
	MatrixXf Xpvnm;
	selectCols(iclip.GetAllPartsPvMean(), Juk, &Xpvnm);
	Xpvnm.colwise().normalize();

	//for (auto& anim : clips)	//? <= 5 animation per character
	{
		auto& cclip = controller.GetClipInfo(anim.Name);

		// Independent Active blocks only
		const auto &Jck = cclip.ActiveParts;

		// Ecb, Energy of Character Active Parts
		RowVectorXf Ecb;
		// Ecb3, Directional Energy of Character Active Parts
		MatrixXf Ecb3;

		selectCols(cclip.Eb, Jck, &Ecb);
		selectCols(cclip.Eb3, Jck, &Ecb3);

		// Character Perceptive vector mean normalized
		MatrixXf Cpvnm;
		selectCols(cclip.GetAllPartsPvMean(), Jck, &Cpvnm);
		Cpvnm.colwise().normalize();

		// Memery allocation
		auto CoRSize = Juk.size() + Jck.size();

		MatrixXf A(Juk.size(), Jck.size());

		// Caculate Bipetral Matching Distance Matrix A
		// Eb3 is ensitially varience matrix here
		for (int i = 0; i < Juk.size(); i++)
		{
			for (int j = 0; j < Jck.size(); j++)
			{
				A(i, j) = ((Xpvnm.col(i) - Cpvnm.col(j)).array() * Ecb3.col(j).array()).cwiseAbs2().sum();
			}
		}

		// Anisometric Gaussian kernal here
		A.array() = (A.array() / (DirectX::XM_PI / 6)).cwiseAbs2().exp();
		//A.noalias() = Xsp.transpose() * Csp;

		Tensor<float, 4> C((int)Juk.size(), (int)Juk.size(), (int)Jck.size(), (int)Jck.size());

		CaculateQuadraticDistanceMatrix(C, iclip, cclip);

		vector<DenseIndex> matching(A.cols());

		float score = max_quadratic_assignment(A, C, matching);

		float maxScore = numeric_limits<float>::min();
		DenseIndex maxPhi = -1;

#pragma region Display Debug Armature Parts Info
		cout << "=============================================" << endl;
		cout << "Best assignment for " << character.Name << " : " << anim.Name << endl;
		cout << "Scores : " << maxScore << endl;

		cout << "*********************************************" << endl;
		cout << "Human Skeleton ArmatureParts : " << endl;
		for (auto i : Juk)
		{
			const auto& blX = *userParts[i];
			cout << "Block " << i << " = " << blX.Joints << endl;
		}

		cout << "*********************************************" << endl;
		cout << "Character " << character.Name << "'s Skeleton ArmatureParts : " << endl;

		for (auto& i : Jck)
		{
			const auto& blY = *charaParts[i];
			cout << "Block " << i << " = " << blY.Joints << endl;
		}
		cout << "__________ Parts Assignment __________"<< endl;
		for (int i = 0; i < matching.size(); i++)
		{
			int ju = Juk[i], jc = Jck[matching[i]];
			for (int phi = 0; phi < T; phi += Ti)
			{
				if (ju >= 0 && jc >= 0)
				{
					cout << userParts[ju]->Joints << " ==> " << charaParts[jc]->Joints << endl;
				}
			}
		}
		cout << "__________ Fin __________" << endl;
#pragma endregion

		Cca<float> cca;
		MatrixXf corrlations(Ts, matching.size());
		for (int i = 0; i < matching.size(); i++)
		{
			int ju = Juk[i], jc = Jck[matching[i]];
			for (int phi = 0; phi < T; phi += Ti)
			{
				if (ju >= 0 && jc >= 0)
				{
					cca.computeFromQr(iclip.GetPartPvPcaQrView(ju, phi, T), cclip.GetPartPvPcaQr(jc), false);
					corrlations(phi/Ti,i) = cca.correlaltions().minCoeff();
				}
				else
					corrlations(phi/Ti,i) = 0;
			}
		}

		//? maybe other reduce function like min?
		//! We should allowed a window of range for phi matching among different parts
		float sumCor = corrlations.rowwise().sum().maxCoeff(&maxPhi);
		maxPhi *= Ti;

		cclip.Score = maxScore;
		cclip.Matching = matching;
		cclip.Phi = maxPhi;

		// Transform pair for active parts
		std::vector<P2PTransform> partTransforms;

		for (int i = 0; i < matching.size(); i++)
		{
			int ju = Juk[i], jc = Jck[matching[i]];
			if (ju >= 0 && jc >= 0)
			{
				partTransforms.emplace_back();
				auto &partTra = partTransforms.back();
				partTra.DstIdx = jc;
				partTra.SrcIdx = ju;

				FindPartToPartTransform(partTra, iclip, cclip, maxPhi);
			}
		}

		auto pBinding = new PartilizedTransform(&userParts, &charaParts);
		pBinding->ActiveParts = move(partTransforms);
		cclip.pLocalBinding.reset(pBinding);

	} // Animation clip scope

	auto pBinding = new PartilizedTransform(&userParts,&charaParts);

	controller.SetBinding(pBinding);

	auto maxClip = std::max_element(BEGIN_TO_END(clipinfos), [](const ClipInfo& lhs, const ClipInfo &rhs) {
		return (lhs.Score < rhs.Score);
	});

	return move(maxClip->pLocalBinding);

	//if (g_EnableDependentControl)
	//{
	//	for (auto& pBlock : charaParts)
	//	{
	//		auto& block = *pBlock;
	//		//if (block.Index == 0)
	//		//	continue;
	//		if (block.ActiveActionCount == 0 && block.SubActiveActionCount > 0)
	//		{
	//			pBinding->Maps.emplace_back(block.PdCca);
	//		}
	//	}
	//}

	//VectorXf JuScore(Juk.size());
	//JuScore.setZero();

	//cout << endl;
	//cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
}
