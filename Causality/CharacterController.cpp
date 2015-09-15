#include "pch_bcl.h"
#include <tinyxml2.h>
#include "CharacterController.h"
#include "CharacterObject.h"
#include "AnimationAnalyzer.h"
#include <boost\format.hpp>
#include "PcaCcaMap.h"
#include "Settings.h"
#include "ArmatureTransforms.h"
#include "RegressionModel.h"
#include <dlib\optimization\optimization.h>

using namespace Causality;
using namespace std;
using namespace Eigen;
using namespace DirectX;

typedef dlib::matrix<double, 0, 1> dlib_vector;

extern Eigen::RowVector3d g_NoiseInterpolation;
const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

#define BEGIN_TO_END(range) range.begin(), range.end()

bool ReadGprParamXML(tinyxml2::XMLElement * blockSetting, Eigen::Vector3d &param);
void InitGprXML(tinyxml2::XMLElement * settings, const std::string & blockName, gaussian_process_regression& gpr);

class StylizedChainIK
{
	// translation of local chain, 128 bit aligned
	vector<Vector4, XMAllocator>						m_chain;
	gaussian_process_regression							m_gplvm;
	double												m_ikWeight;
	double												m_markovWeight;
	bool												m_cValiad;
	RowVectorXd											m_cx;
	RowVectorXd											m_cy;
	Vector3d											m_goal;

public:
	StylizedChainIK(const std::vector<Joint*> &joints, const AffineFrame& defaultframe)
	{
		m_chain.resize(joints.size());
		for (int i = 0; i < joints.size(); i++)
		{
			auto& bone = defaultframe[joints[i]->ID()];
			reinterpret_cast<Vector3&>(m_chain[i]) = bone.LclTranslation;
		}
	}

	void SetIKWeight(double weight)
	{
		m_ikWeight = weight;
	}
	void SetMarkovWeight(double weight)
	{
		m_markovWeight = weight;
	}

	XMVECTOR EndPosition(const RowVectorXd& y)
	{
		const auto n = m_chain.size();

		XMVECTOR q, t , gt, gq;
		Eigen::Vector4f qs;
		qs.setZero();

		gt = XMVectorZero();
		gq = XMQuaternionIdentity();

		for (int i = 0; i < n; i++)
		{
			qs = y.segment<3>(i * 3).cast<float>();
			q = XMLoadFloat4A(qs.data());
			q = XMQuaternionExp(q); // revert the log map

			gq = XMQuaternionMultiply(q, gq);
			t = m_chain[i];
			t = XMVector3Rotate(t, gq);
			gt += t;
		}

		return gt;
	}

	double objective(const RowVectorXd &x, const RowVectorXd &y)
	{
		const auto n = m_chain.size();

		XMVECTOR ep = EndPosition(y);
		Vector3f epf;
		XMStoreFloat3(epf.data(), ep);

		double ikdis = (epf.cast<double>() - m_goal).cwiseAbs2().sum() * m_ikWeight;

		double markovdis = (y - m_cy).cwiseAbs2().sum() * m_markovWeight;

		double fitlikelihood = m_gplvm.likelihood_xy(x, y);

		return fitlikelihood + ikdis + markovdis;
	}


	// Filter alike interface
	// reset history data
	void Reset()
	{
		m_cValiad = false;
	}

	template <class DerivedX, class DerivedY>
	dlib_vector ComposeOptimizeVector(const DenseBase<DerivedX> &x, const DenseBase<DerivedY> &y)
	{
		dlib_vector v(x.size() + y.size());
		RowVectorXd::Map(v.begin(), x.size()) = x;
		RowVectorXd::Map(v.begin() + x.size() , y.size()) = y;
		return v;
	}

	void DecomposeOptimizeVector(const dlib_vector& v, RowVectorXd &x, RowVectorXd &y)
	{
		x = RowVectorXd::Map(v.begin(), x.size());
		y = RowVectorXd::Map(v.begin() + x.size(), y.size());
	}

	// return the joints rotation vector
	const RowVectorXd& Apply(const Vector3d& goal)
	{
		if (!m_cValiad)
		{
			m_cx = goal;
			m_gplvm.get_expectation(m_cx, &m_cy);
			m_cValiad = true;
		}

		auto v = ComposeOptimizeVector(m_cx,m_cy);

		auto f = [this](const dlib_vector& v)->double {
			RowVectorXd x, y;
			DecomposeOptimizeVector(v, x, y);
			return objective(x, y);
		};

		dlib::find_min_using_approximate_derivatives(
			dlib::bfgs_search_strategy(),
			dlib::objective_delta_stop_strategy(1e-5),
			f,
			v,
			numeric_limits<double>::min());

		DecomposeOptimizeVector(v, m_cx, m_cy);
		// auto df = dlib::derivative(f);
		return m_cx;
	}
};


class SelfLocalMotionTransform : public ArmatureTransform
{
public:
	const BlockArmature * pBlockArmature;

	mutable BlockEndEffectorFeatureExtractor<InputFeature>  inputExtractor;
	mutable BlockAllJointsFeatureExtractor<CharacterFeature> outputExtractor;

	mutable MatrixXd m_Xs;

	SelfLocalMotionTransform(BlockArmature * pBlocks)
		: pBlockArmature(pBlocks), inputExtractor(true)
	{
		pSource = &pBlockArmature->Armature();
		pTarget = &pBlockArmature->Armature();
		m_Xs.resize(pBlockArmature->size(), g_PvDimension);
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

	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const AffineFrame& last_frame, float frame_time) const
	{
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

				X.segment<3>(0) = xf.cast<double>();

				auto Xv = X.segment<3>(3);
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

				Xv = (Xd - Xld) / (frame_time * g_FrameTimeScaleFactor);

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
				covObsr.diagonal() = g_NoiseInterpolation.replicate(1, 2).transpose() * varZ;

				block->PdGpr.get_expectation_from_observation(X, covObsr, &Y);
				block->PdGpr.get_expectation(X, &Y);

				yf = Y.cast<float>();
				yf *= block->Wx.cwiseInverse().asDiagonal();

				outputExtractor.Set(*block, target_frame, yf);
			}
		}

		// Fill Xabpv
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
			if (block->SubActiveActionCount > 0)
			{
				auto lk = block->PdGpr.get_expectation_and_likelihood(Xabpv, &Y);

				yf = Y.cast<float>();
				yf *= block->Wx.cwiseInverse().asDiagonal();

				outputExtractor.Set(*block, target_frame, yf);
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
	for (auto& p : m_Clipinfos)
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

void CharacterController::UpdateTargetCharacter(const AffineFrame & frame, const AffineFrame & lastframe, double deltaTime) const
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

	// CrefRot * (HrefRot^-1 * HRot)
	auto rot = XMQuaternionMultiply(CMapRefRot.Load(),
		XMQuaternionMultiply(
			XMQuaternionConjugate(MapRefRot.Load()),
			frame[0].GblRotation.LoadA()));

	m_pCharacter->SetPosition(pos);
	m_pCharacter->SetOrientation(rot);

	m_pCharacter->ReleaseCurrentFrameFrorUpdate();
}

ClipInfo & CharacterController::GetAnimationInfo(const string & name) {
	return *m_Clipinfos[name];
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

void CharacterController::SetTargetCharacter(CharacterObject & object) {
	m_pCharacter = &object;
	if (m_pBinding)
		m_pBinding->SetTargetArmature(object.Armature());
	auto & behavier = object.Behavier();
	PotientialFrame = object.Armature().default_frame();

	m_pCharacter->Behavier().Blocks().ComputeWeights();

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

		m_Clipinfos[anim.Name] = new ClipInfo(behavier.Blocks());

		auto& analyzer = m_Clipinfos[anim.Name];

		analyzer->Phi = -1;
		//analyzer->Score = -1;
		analyzer->PcaCutoff = g_CharacterPcaCutoff;
		analyzer->EnergyCutoff = g_CharacterActiveEnergy;

		tasks.emplace_back(analyzer->ComputeFromFramesAsync(anim.GetFrameBuffer()));
	}

	when_all(tasks.begin(), tasks.end()).then([this]() {

		cout << setprecision(4) << setw(6);

		float globalEnergyMax = 0;

		tinyxml2::XMLDocument paramdoc;
		tinyxml2::XMLElement* settings = nullptr;
		string paramFileName = "CharacterAnalayze\\" + m_pCharacter->Name + ".param.xml";
		string settingName = str(boost::format("clip_rasterize_%1%") % CLIP_FRAME_COUNT);

		for (auto itr : m_Clipinfos)
		{
			auto &clipinfo = *itr.second;
			auto &clipname = itr.first;
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


		auto& blocks = m_pCharacter->Behavier().Blocks();
		for (auto& pair : m_Clipinfos)
		{
			auto& key = pair.first;
			auto analyzer = pair.second;
			auto& Eb = analyzer->Eb;
			Eb /= globalEnergyMax;
			analyzer->EnergyCutoff = g_CharacterActiveEnergy;
			cout << Eb << endl;
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

		vector<KinematicBlock*> activeBlocks;
		vector<KinematicBlock*> inactiveBlocks;
		//auto activeBlocks = blocks | adaptors::filtered([](auto pBlock) {return pBlock->ActiveActionCount > 0});
		//auto inactiveBlocks = blocks | adaptors::filtered([](auto pBlock) {return pBlock->ActiveActionCount == 0});

		// for all inactive blocks, try to sythesis their sutle local motion based on action blocks
		for (auto pBlock : blocks)
		{
			if (pBlock->ActiveActionCount == 0)
				inactiveBlocks.push_back(pBlock);
			else
				activeBlocks.push_back(pBlock);
		}

		Eigen::MatrixXf Xabpv(CLIP_FRAME_COUNT * m_Clipinfos.size(), size(activeBlocks) * 6);

		int bid = 0; // active block index
					 //concuncy::for_each(activeBlocks.begin(), activeBlocks.end(), [&, this](KinematicBlock* pBlock)
		for (auto pBlock : activeBlocks)
		{
			auto& block = *pBlock;

			auto bj = block.Index;
			auto maxItr = std::max_element(BEGIN_TO_END(m_Clipinfos), [bj](const auto& lhs, const auto& rhs)
			{
				return lhs.second->DimX[bj] < rhs.second->DimX[bj];
			});
			auto maxDim = maxItr->second->DimX[bj];

			cout << "{";
			for (auto pJoint : block.Joints)
			{
				cout << pJoint->Name() << ", ";
			}

			cout << "\b\b} [" << maxDim << "] : ";

#pragma region Build Xabpv
			auto Bft = Xabpv.middleCols<g_PvDimension>(bid * g_PvDimension);
			int cid = 0; // Clip index
			for (auto& paitr : m_Clipinfos)
			{
				auto clipinfo = paitr.second;
				auto& anim = m_pCharacter->Behavier()[paitr.first];

				auto map = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(clipinfo->Pvs.col(block.Index).data(), CLIP_FRAME_COUNT, 3);
				auto displacement = Bft.block(cid*CLIP_FRAME_COUNT, 0, CLIP_FRAME_COUNT, 3);
				displacement = map;
				float frametime = (float)(anim.Duration.count() / CLIP_FRAME_COUNT);

				auto velocity = Bft.block(cid*CLIP_FRAME_COUNT, 3, CLIP_FRAME_COUNT, 3);
				GetVolocity(displacement, velocity, frametime * g_FrameTimeScaleFactor);

				//ExpandQuadraticTerm(Bft, 3);

				++cid;
			}
			++bid;
#pragma endregion 

			block.X.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
			block.Pd.resize(block.ActiveActionCount*CLIP_FRAME_COUNT, g_PvDimension);
			for (size_t i = 0; i < block.ActiveActionCount; i++)
			{
				auto &clip = *m_Clipinfos[block.ActiveActions[i]];
				auto &anim = m_pCharacter->Behavier()[block.ActiveActions[i]];
				float frametime = (float)(anim.Duration.count() / CLIP_FRAME_COUNT);

				block.X.middleRows(i*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = clip.Xbs[block.Index];

				auto map = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(clip.Pvs.col(block.Index).data(), CLIP_FRAME_COUNT, 3);
				auto displacement = block.Pd.block(i*CLIP_FRAME_COUNT, 0, CLIP_FRAME_COUNT, 3);
				displacement = map;

				//ExpandQuadraticTerm(block.Pd, 3);

				//float dNorm = sqrtf(displacement.rowwise().squaredNorm().maxCoeff());
				//displacement /= dNorm;

				auto velocity = block.Pd.block(i*CLIP_FRAME_COUNT, 3, CLIP_FRAME_COUNT, 3);
				GetVolocity(displacement, velocity, frametime * g_FrameTimeScaleFactor);

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

			float corr;
			if (g_UseStylizedIK)
			{
				block.PdGpr.initialize(block.Pd, block.X * block.Wx.asDiagonal());

				// paramter caching 
				const auto&	blockName = block.Joints[0]->Name();

				InitGprXML(settings, blockName, block.PdGpr);

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

			cout << "\b | [" << corr << ',' << block.PdCca.pcY.cols() << ']';
			cout << endl;

		}//);

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
			for (auto& pair : m_Clipinfos)
			{
				auto& clipinfo = pair.second;
				if (clipinfo->Eb[block.Index] > g_CharacterSubactiveEnergy)
				{
					++subAcitveClipCount;
					block.SubActiveActions.push_back(pair.first);
				}
			}

			block.SubActiveActionCount = subAcitveClipCount;

			if (subAcitveClipCount == 0)
			{
				block.PdCca.Jx = block.PdCca.Jy = -1;
				block.PvDriveScore = -1;
				continue;
			}

			int cid = 0, cid2 = 0; // Clip index
			block.X.resize(subAcitveClipCount*CLIP_FRAME_COUNT, block.Joints.size() * CharacterFeature::Dimension);
			block.Pd.resize(subAcitveClipCount*CLIP_FRAME_COUNT, Xabpv.cols());

			for (auto clipinfo : adaptors::values(m_Clipinfos))
			{
				if (clipinfo->Eb[block.Index] > g_CharacterSubactiveEnergy)
				{
					block.X.middleRows(cid*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = clipinfo->Xbs[block.Index];
					block.Pd.middleRows(cid*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT) = Xabpv.middleRows(cid2*CLIP_FRAME_COUNT, CLIP_FRAME_COUNT);
					++cid;
				}
				++cid2;
			}

			cout << "[Inactive] {";
			for (auto pJoint : block.Joints)
			{
				cout << pJoint->Name() << ", ";
			}

			float corr = 0.0;
			if (!g_UseStylizedIK)
			{
				corr = block.PdCca.CreateFrom(block.Pd, block.X, 0.01 * g_CharacterPcaCutoff, g_CharacterPcaCutoff);
				block.PvDriveScore = corr;
			}
			else
			{
				block.PdGpr.initialize(block.Pd, block.X * block.Wx.asDiagonal());

				// paramter caching 
				const auto&	blockName = block.Joints[0]->Name();

				InitGprXML(settings, blockName, block.PdGpr);
				corr = exp(-corr);
			}

			if (g_EnableDebugLogging)
			{
				ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + block.Joints[0]->Name() + ".x.csv");
				fout << block.X.format(CSVFormat);
				fout.close();
			}

			auto bid = block.Index;
			auto maxItr = std::max_element(m_Clipinfos.begin(), m_Clipinfos.end(), [bid](const auto& lhs, const auto& rhs)
			{
				return lhs.second->DimX[bid] < rhs.second->DimX[bid];
			});
			auto maxDim = maxItr->second->DimX[bid];

			cout << "\b\b} [" << maxDim << "] : " << corr << endl;

			block.PdCca.Jy = block.Index;
			block.PdCca.Jx = -2;
		}

		cout << "=================================================================" << endl;

		if (!g_UseStylizedIK)
		{
			auto pBinding = make_unique<BlockizedCcaArmatureTransform>(&blocks, &blocks);

			pBinding->pInputExtractor.reset(new BlockEndEffectorGblPosQuadratic());
			//pBinding->pInputExtractor.reset(new BlockEndEffectorFeatureExtractor<InputFeature>(true));
			pBinding->pOutputExtractor.reset(new BlockAllJointsFeatureExtractor<CharacterFeature>(false));

			//auto pBinding = make_unique<BlockizedSpatialInterpolateQuadraticTransform>(&m_Clipinfos,&blocks, &blocks);

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
			m_pSelfBinding = make_unique<SelfLocalMotionTransform>(&blocks);
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
