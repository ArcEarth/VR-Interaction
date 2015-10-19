#include "pch_bcl.h"
#include <tinyxml2.h>
#include "CharacterController.h"
#include "CharacterObject.h"
#include "ClipMetric.h"
#include <boost\format.hpp>
#include "PcaCcaMap.h"
#include "Settings.h"
#include "EigenExtension.h"
#include "ArmatureTransforms.h"
//#pragma warning (disable:4297)
//#include <dlib\optimization\optimization.h>
#pragma warning (disable:4554)
#include <unsupported\Eigen\CXX11\Tensor>
#include "QudraticAssignment.h"
#include <ppl.h>
#include "GaussianProcess.h"
#include "StylizedIK.h"
#include "ArmaturePartFeatures.h"

using namespace Causality;
using namespace std;
using namespace Eigen;
using namespace DirectX;
using namespace ArmaturePartFeatures;
using namespace BoneFeatures;

//typedef dlib::matrix<double, 0, 1> dlib_vector;

extern Eigen::RowVector3d g_NoiseInterpolation;
const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

#define BEGIN_TO_END(range) range.begin(), range.end()

#ifdef _DEBUG
#define DEBUGOUT(x) std::cout << #x << " = " << x << std::endl
#else
#define DEBUGOUT(x)
#endif

bool ReadGprParamXML(tinyxml2::XMLElement * blockSetting, Eigen::Vector3d &param);
void InitGprXML(tinyxml2::XMLElement * settings, const std::string & blockName, gaussian_process_regression& gpr);

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

void InitializeExtractor(AllJointRltLclRotLnQuatPcad& ft, const ShrinkedArmature& parts)
{
	// Init 
	ft.InitPcas(parts.size());
	ft.SetDefaultFrame(parts.Armature().default_frame());
	for (auto part : parts)
	{
		//ft.SetPca(part->Index, part->ChainPcaMatrix, part->ChainPcaMean);
	}
}

class SelfLocalMotionTransform : public ArmatureTransform
{
public:
	const ShrinkedArmature *	pBlockArmature;
	CharacterController*		pController;

	std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> * pHandles;

	mutable
		Localize<EndEffector<InputFeature>>
		inputExtractor;

	mutable
		Pcad <
		Weighted <
		RelativeDeformation <
		AllJoints < LclRotLnQuatFeature > > > >
		outputExtractor;

	//mutable MatrixXd m_Xs;

	SelfLocalMotionTransform(CharacterController & controller)
		: pController(&controller), pHandles(nullptr)
	{
		pBlockArmature = &controller.Character().Behavier().ArmatureParts();
		pSource = &pBlockArmature->Armature();
		pTarget = &pBlockArmature->Armature();

		InitializeOutputFeature();
	}

	void InitializeOutputFeature()
	{
		auto& parts = *pBlockArmature;
		auto& ucinfo = pController->GetUnitedClipinfo();

		outputExtractor.InitPcas(parts.size());
		outputExtractor.SetDefaultFrame(pBlockArmature->Armature().default_frame());
		outputExtractor.InitializeWeights(parts);

		auto& facade = ucinfo.RcFacade;
		auto cutoff = facade.PcaCutoff();
		for (auto part : parts)
		{
			int pid = part->Index;
			if (part->ActiveActions.size() > 0 || part->SubActiveActions.size() > 0)
			{
				auto &pca = facade.GetPartPca(pid);
				auto d = facade.GetPartPcaDim(pid);
				outputExtractor.SetPca(pid, pca.components(d), pca.mean());
			}
			else
			{
				int odim = facade.GetPartDimension(pid);
				outputExtractor.SetPca(pid, MatrixXf::Identity(odim, odim), facade.GetPartMean(pid));
			}
		}
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

			if (block->ActiveActions.size() > 0)
			{
				auto& sik = pController->GetStylizedIK(block->Index);
				auto& gpr = sik.Gplvm();
				gpr.get_expectation(X, &Y);
				yf = Y.cast<float>();
				yf *= block->Wx.cwiseInverse().asDiagonal();

				outputExtractor.Set(*block, target_frame, yf);
			}
			else if (block->SubActiveActions.size() > 0)
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

		int pvDim = inputExtractor.GetDimension(*blocks[0]);

		RowVectorXd X(g_UseVelocity ? pvDim * 2 : pvDim), Y;

		double semga = 1000;
		RowVectorXf yf;

		std::vector<RowVectorXd> Xabs;
		for (auto& block : blocks)
		{
			//X[0] *= 13;

			if (block->Index >0 && block->ActiveActions.size() > 0)
			{
				auto& sik = pController->GetStylizedIK(block->Index);
				auto& gpr = sik.Gplvm();
				auto& joints = block->Joints;

				RowVectorXf xf = inputExtractor.Get(*block, source_frame);
				RowVectorXf xfl = inputExtractor.Get(*block, last_frame);

				yf = outputExtractor.Get(*block, target_frame);
				auto pDecoder = sik.GetDecoder();
				auto baseRot = target_frame[block->parent()->Joints.back()->ID].GblRotation;
				sik.SetBaseRotation(baseRot);
				sik.SetChain(block->Joints, target_frame);

				//sik.SetGplvmWeight(block->Wx.cast<double>());

				//std::vector<DirectX::Quaternion, XMAllocator> corrrots(joints.size());
				//std::vector<DirectX::Quaternion, XMAllocator> rots(joints.size());

				//for (int i = 0; i < joints.size(); i++)
				//{
				//	corrrots[i] = target_frame[joints[i]->ID].LclRotation;
				//}

				//(*pDecoder)(rots.data(), yf.cast<double>());
				////outputExtractor.Set(*block, target_frame, yf);
				//auto ep = sik.EndPosition(reinterpret_cast<XMFLOAT4A*>(rots.data()));


				X.segment(0, pvDim) = xf.cast<double>();


				auto Xd = X.segment(0, pvDim);
				auto uXd = gpr.uX.segment(0, pvDim);

				//auto uXv = block->PdGpr.uX.segment<3>(3);
				//Xv = (Xv - uXv).array() * g_NoiseInterpolation.array() + uXv.array();

				Xd = (Xd - uXd).array();

				double varZ = (Xd.array() * (g_NoiseInterpolation.array() - 1.0)).cwiseAbs2().sum();
				// if no noise
				varZ = std::max(varZ, 1e-5);

				Xd = Xd.array() * g_NoiseInterpolation.array() + uXd.array();

				RowVector3d Xld = (xfl.cast<double>() - uXd).array() * g_NoiseInterpolation.array() + uXd.array();

				if (g_UseVelocity)
				{
					auto Xv = X.segment(pvDim, pvDim);
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

				//m_Xs.row(block->Index) = X;
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
				//auto yc = yf;
				//yf = Y.cast<float>();
				//yf.array() *= block->Wx.cwiseInverse().array().transpose();

				//block->PdStyleIk.SetHint();
				if (!g_UseVelocity)
					Y = sik.Apply(X.transpose());
				else
					Y = sik.Apply(X.segment(0, pvDim).transpose(), Vector3d(X.segment(pvDim, pvDim).transpose()));

				//block->PdStyleIk.SetGoal(X.leftCols<3>());

				//auto scoref = block->PdStyleIk.objective(X, yf.cast<double>());
				//auto scorec = block->PdStyleIk.objective(X, yc.cast<double>());
				//std::cout << "Gpr score : " << scoref << " ; Cannonical score : " << scorec << endl;
				//auto ep = sik.EndPosition(yf.cast<double>());

				//Y = yf.cast<double>();
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
			Xabpv.resize(pController->uXabpv.size());
			int i = 0;
			for (const auto& xab : Xabs)
			{
				auto Yi = Xabpv.segment(i, xab.size());
				Yi = xab;
				i += xab.size();
			}

			auto _x = (Xabpv.cast<float>() - pController->uXabpv) * pController->XabpvT;
			auto _xd = _x.cast<double>();

			for (auto& block : blocks)
			{
				if (block->ActiveActions.size() == 0 && block->SubActiveActions.size() > 0)
				{

					auto& sik = pController->GetStylizedIK(block->Index);
					auto& gpr = sik.Gplvm();

					auto lk = gpr.get_expectation_and_likelihood(_xd, &Y);

					yf = Y.cast<float>();
					//yf *= block->Wx.cwiseInverse().asDiagonal();

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
	CharacterScore = 0;
	CurrentActionIndex = 0;
	SetSourceArmature(player);
	SetTargetCharacter(character);
}

const ArmatureTransform & CharacterController::Binding() const { return *m_pBinding; }

ArmatureTransform & CharacterController::Binding() { return *m_pBinding; }

void CharacterController::SetBinding(std::unique_ptr<ArmatureTransform> &&pBinding)
{
	lock_guard<mutex> guard(m_bindMutex);
	m_pBinding = move(pBinding);
}

const CharacterObject & CharacterController::Character() const { return *m_pCharacter; }

CharacterObject & CharacterController::Character() { return *m_pCharacter; }

const IArmature & Causality::CharacterController::Armature() const
{
	return Character().Behavier().Armature();
}

IArmature & Causality::CharacterController::Armature()
{
	return Character().Behavier().Armature();
}

const ShrinkedArmature & Causality::CharacterController::ArmatureParts() const
{
	return Character().Behavier().ArmatureParts();
}

ShrinkedArmature & Causality::CharacterController::ArmatureParts()
{
	return Character().Behavier().ArmatureParts();
}

float CharacterController::UpdateTargetCharacter(const BoneHiracheryFrame & frame, const BoneHiracheryFrame & lastframe, double deltaTime) const
{
	if (m_pBinding == nullptr)
		return 0;
	{
		std::lock_guard<mutex> guard(m_bindMutex);
		auto& cframe = m_pCharacter->MapCurrentFrameForUpdate();
		m_pBinding->Transform(cframe, frame, lastframe, deltaTime);
	}

	float l = 100;
	for (auto& bone : frame)
	{
		if (bone.GblTranslation.y < l)
			l = bone.GblTranslation.y;
	}

	using namespace DirectX;

	auto& bone = frame[0];
	//auto pos = frame[0].GblTranslation - MapRefPos + CMapRefPos;
	SychronizeRootDisplacement(bone);

	m_pCharacter->ReleaseCurrentFrameFrorUpdate();

	return 1.0;
}

void Causality::CharacterController::SychronizeRootDisplacement(const Causality::Bone & bone) const
{
	auto pos = Character().GetPosition() + bone.GblTranslation - LastPos;

	LastPos = bone.GblTranslation;

	// CrefRot * Yaw(HrefRot^-1 * HRot)
	auto rot = XMQuaternionMultiply(
		XMQuaternionConjugate(XMLoad(MapRefRot)),
		XMLoadA(bone.GblRotation));

	// extract Yaw rotation only, it's a bad hack here
	rot = XMQuaternionLn(rot);
	rot = XMVectorMultiply(rot, g_XMIdentityR1.v);
	rot = XMQuaternionExp(rot);

	rot = XMQuaternionMultiply(XMLoad(CMapRefRot), rot);


	m_pCharacter->SetPosition(pos);
	m_pCharacter->SetOrientation(rot);
}

float CreateControlTransform(CharacterController & controller, const ClipFacade& iclip);

float CharacterController::CreateControlBinding(const ClipFacade& inputClip)
{
	return CreateControlTransform(*this, inputClip);
}

const std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>>& Causality::CharacterController::PvHandles() const
{
	return m_PvHandles;
}

std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>>& CharacterController::PvHandles()
{
	return m_PvHandles;
}

CharacterClipinfo & CharacterController::GetClipInfo(const string & name) {
	auto itr = std::find_if(BEGIN_TO_END(m_Clipinfos), [&name](const auto& clip) {
		return clip.ClipName() == name;
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

void ShiftFrame(_Inout_ BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& ref, float scale)
{
	XMVECTOR sv = XMVectorReplicate(scale);
	for (size_t i = 0; i < frame.size(); i++)
	{
		auto& lt = frame[i].LocalTransform();
		auto& rt = ref[i].LocalTransform();
		IsometricTransform::LerpV(lt, rt, lt, sv);
	}

}

vector<BoneHiracheryFrame> CreateReinforcedFrames(const BehavierSpace& behavier)
{
	auto& clips = behavier.Clips();
	auto& parts = behavier.ArmatureParts();
	auto& armature = behavier.Armature();
	auto& dframe = armature.default_frame();

	float factors[] = { /*0.5f,0.75f,*/1.0f/*,1.25f */ };
	int k = size(factors);
	int n = CLIP_FRAME_COUNT;

	std::vector<int> cyclips;
	cyclips.reserve(clips.size());
	for (int i = 0; i < clips.size(); i++)
	{
		if (clips[i].Cyclic())
			cyclips.push_back(i);
	}

	vector<BoneHiracheryFrame> frames(n * cyclips.size() * k);

	int ci = 0;
	double totalTime = 0;

	//concurrency::parallel_for((int)0, (int)clips.size() * k, [&](int cik) 
	for (int cik = 0; cik < cyclips.size() * k; cik++)
	{
		int ci = cik / k;
		int i = cik % k;

		auto animBuffer = clips[cyclips[ci]].GetFrameBuffer();
		int stidx = cik * n;
		copy_n(animBuffer.begin(), n, frames.begin() + stidx);
		if (fabsf(factors[i] - 1.0f) > 1e-5)
		{
			for (int j = 0; j < n; j++)
			{
				ShiftFrame(frames[stidx + j], dframe, factors[i]);
				frames[stidx + j].RebuildGlobal(armature);
			}
		}
	}
	//);

	return frames;
}


void CharacterController::SetTargetCharacter(CharacterObject & chara) {

	m_pCharacter = &chara;

	if (m_pBinding)
		m_pBinding->SetTargetArmature(chara.Armature());

	auto& behavier = chara.Behavier();
	auto& armature = chara.Armature();
	auto& clips = behavier.Clips();
	auto& parts = behavier.ArmatureParts();

	m_SIKs.resize(parts.size());
	PotientialFrame = armature.default_frame();
	m_PvHandles.resize(armature.size());

	parts.ComputeWeights();
	if (!g_UseJointLengthWeight)
	{
		for (auto& part : parts)
		{
			part->Wx.setOnes();
			part->Wxj.setOnes();
		}
	}

	behavier.UniformQuaternionsBetweenClips();

	for (auto& anim : clips)
	{
		if (anim.Name == "idle" || anim.Name == "die")
			anim.IsCyclic = false;
		else
			anim.IsCyclic = true;
	}

	//clips.erase(std::remove_if(BEGIN_TO_END(clips), [](const auto& anim) ->bool {return !anim.IsCyclic;}), clips.end());

	using namespace concurrency;
	vector<task<void>> tasks;
	m_Clipinfos.reserve(clips.size() + 1);
	{
		tasks.emplace_back(create_task([this]() {
			auto& chara = *m_pCharacter;
			auto& behavier = chara.Behavier();
			auto& parts = behavier.ArmatureParts();

			auto allFrames = CreateReinforcedFrames(behavier);

			//? To-Do Setup proper feature for m_cpxClipinfo
			m_cpxClipinfo.Initialize(parts);
			// set subactive energy to almost zero that make sure all part's pca is caculated
			//m_cpxClipinfo.RcFacade.SetActiveEnergy(g_CharacterActiveEnergy, g_CharacterSubactiveEnergy * 0.01f);
			//m_cpxClipinfo.PvFacade.SetActiveEnergy(g_CharacterActiveEnergy, g_CharacterSubactiveEnergy * 0.01f);
			m_cpxClipinfo.AnalyzeSequence(allFrames, 0);
		}));
	}

	for (auto& anim : clips)
	{
		if (!anim.Cyclic())
			continue;

		m_Clipinfos.emplace_back(behavier.ArmatureParts());
		auto& clipinfo = m_Clipinfos.back();

		clipinfo.SetClipName(anim.Name);

		tasks.emplace_back(create_task([&clipinfo, &anim, &parts]() {
			clipinfo.Initialize(parts);
			auto & frames = anim.GetFrameBuffer();
			clipinfo.AnalyzeSequence(frames, anim.Length().count());
		}));
	}

	when_all(tasks.begin(), tasks.end()).then([this]() {

		auto& chara = *m_pCharacter;
		auto& behavier = chara.Behavier();
		auto& clips = behavier.Clips();
		auto& parts = behavier.ArmatureParts();

		cout << setprecision(4) << setw(6);
		auto& allClipinfo = m_cpxClipinfo;

		tinyxml2::XMLDocument paramdoc;
		tinyxml2::XMLElement* settings = nullptr;
		string paramFileName = "CharacterAnalayze\\" + m_pCharacter->Name + ".param.xml";
		string settingName = str(boost::format("cr_%1%_vel%2%_wj%3%") % CLIP_FRAME_COUNT % g_UseVelocity % g_UseJointLengthWeight);

		for (auto& cinfo : m_Clipinfos)
			settingName += '_' + cinfo.ClipName();

		if (g_LoadCharacterModelParameter)
		{
			auto error = paramdoc.LoadFile(paramFileName.c_str());
			tinyxml2::XMLElement* paramStore = nullptr;
			if (error == tinyxml2::XML_SUCCESS)
			{
				paramStore = paramdoc.RootElement();
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

		float globalEnergyMax = 0;
		for (auto& clipinfo : m_Clipinfos)
		{
			assert(clipinfo.IsReady());

			auto& Eb = clipinfo.PvFacade.GetAllPartsEnergy();
			globalEnergyMax = std::max(Eb.maxCoeff(), globalEnergyMax);

			DEBUGOUT(Eb);
		}

		std::set<int> avtiveSet;
		std::set<int> subactSet;
		for (auto& clipinfo : m_Clipinfos)
		{
			auto& key = clipinfo.ClipName();
			auto& Eb = clipinfo.PvFacade.GetAllPartsEnergy();

			for (int i = 0; i < Eb.size(); i++)
			{
				if (Eb[i] > g_CharacterActiveEnergy * globalEnergyMax)
				{
					parts[i]->ActiveActions.push_back(key);
					avtiveSet.insert(i);

					// if a part is alreay marked as subactive, promote it
					auto itr = subactSet.find(i);
					if (itr != subactSet.end())
						subactSet.erase(itr);
				}
				else if (avtiveSet.find(i) == avtiveSet.end() && Eb[i] > g_CharacterSubactiveEnergy * globalEnergyMax)
				{
					parts[i]->SubActiveActions.push_back(key);
					subactSet.insert(i);
				}
			}
		}

		// Remove Root from caculation
		avtiveSet.erase(0);
		subactSet.erase(0);
		parts[0]->ActiveActions.clear();
		parts[0]->SubActiveActions.clear();

		m_ActiveParts.assign(BEGIN_TO_END(avtiveSet));
		m_SubactiveParts.assign(BEGIN_TO_END(subactSet));
		vector<int>& activeParts = m_ActiveParts;
		vector<int>& subactParts = m_SubactiveParts;

		cout << "== Active parts ==" << endl;
		for (auto& ap : activeParts)
			cout << parts[ap]->Joints << endl;
		cout << "== Subactive parts ==" << endl;
		for (auto& ap : subactParts)
			cout << parts[ap]->Joints << endl;
		cout << "== End Parts =" << endl;

		// Active parts Pv s
		MatrixXf Xabpv = GenerateXapv(activeParts);
		int dXabpv = Xabpv.cols();

		parallel_for(0, (int)activeParts.size(), 1, [&, this](int apid)
		{
			InitializeAcvtivePart(*parts[activeParts[apid]], settings);
		}
		);

		if (g_LoadCharacterModelParameter)
		{
			auto error = paramdoc.SaveFile(paramFileName.c_str());
			assert(error == tinyxml2::XML_SUCCESS);
		}

		if (g_EnableDependentControl)
			parallel_for_each(BEGIN_TO_END(subactParts), [&, this](int sapid)
		{
			InitializeSubacvtivePart(*parts[sapid], Xabpv, settings);
		}
		);

		cout << "=================================================================" << endl;

		assert(g_UseStylizedIK);
		{
			//? To-Do, Fix this
			auto pBinding = make_unique<SelfLocalMotionTransform>(*this);
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

MatrixXf CharacterController::GenerateXapv(const std::vector<int> &activeParts)
{
	// pvDim without
	int pvDim = m_cpxClipinfo.PvFacade.GetAllPartDimension();
	assert(pvDim > 0);

	auto& allClipinfo = m_cpxClipinfo;

	MatrixXf Xabpv(allClipinfo.ClipFrames(), size(activeParts) * pvDim);

	MatrixXi apMask = VectorXi::Map(activeParts.data(), activeParts.size()).replicate(1, pvDim).transpose();
	auto maskVec = VectorXi::Map(apMask.data(), apMask.size());
	selectCols(allClipinfo.PvFacade.GetAllPartsSequence(), maskVec, &Xabpv);

	Pca<MatrixXf> pcaXabpv(Xabpv);
	int dXabpv = pcaXabpv.reducedRank(g_CharacterPcaCutoff);
	Xabpv = pcaXabpv.coordinates(dXabpv);
	XabpvT = pcaXabpv.components(dXabpv);
	uXabpv = pcaXabpv.mean();

	if (g_EnableDebugLogging)
	{
		ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_Xabpv.pd.csv");
		fout << Xabpv.format(CSVFormat);

		fout.close();
	}


	return Xabpv;
}

void CharacterController::InitializeAcvtivePart(ArmaturePart & part, tinyxml2::XMLElement * settings)
{
	auto pid = part.Index;
	auto& aactions = part.ActiveActions;
	auto& joints = part.Joints;
	auto& allClipinfo = m_cpxClipinfo;

	auto& rcFacade = allClipinfo.RcFacade;
	auto& pvFacade = allClipinfo.PvFacade;

	if (rcFacade.GetPartPcaDim(pid) == -1)
		rcFacade.CaculatePartPcaQr(pid);

	auto& X = rcFacade.GetPartPcadSequence(pid);
	auto& Pv = pvFacade.GetPartSequence(pid);

	//? To-do Select Active Rows from allClipFacade

	if (g_EnableDebugLogging)
	{
		ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + part.Joints[0]->Name + ".pd.csv");
		fout << Pv.format(CSVFormat);
		fout.close();

		fout.open("CharacterAnalayze\\" + m_pCharacter->Name + "_" + part.Joints[0]->Name + ".x.csv");
		fout << rcFacade.GetPartSequence(pid).format(CSVFormat);
		fout.close();
	}

	assert(g_UseStylizedIK && "This build is settled on StylizedIK");
	{
		// paramter caching 
		const auto&	partName = joints[0]->Name;
		auto &dframe = Character().Armature().default_frame();

		auto& sik = m_SIKs[pid];
		auto& gpr = sik.Gplvm();

		gpr.initialize(Pv, X);
		InitGprXML(settings, partName, gpr);

		auto &pca = rcFacade.GetPartPca(pid);
		auto d = rcFacade.GetPartPcaDim(pid);

		//auto pDecoder = std::make_unique<RelativeLnQuaternionDecoder>();
		//! PCA Decoder configration
		auto Wjx = part.Wx.cast<double>().eval();

		auto pDecoder = std::make_unique<RelativeLnQuaternionPcaDecoder>();
		pDecoder->meanY = pca.mean().cast<double>() * Wjx.cwiseInverse().asDiagonal();
		pDecoder->pcaY = pca.components(d).cast<double>();
		pDecoder->invPcaY = pDecoder->pcaY.transpose();
		pDecoder->pcaY = Wjx.asDiagonal() * pDecoder->pcaY;
		pDecoder->invPcaY *= Wjx.cwiseInverse().asDiagonal();

		pDecoder->bases.reserve(joints.size());
		for (auto joint : joints)
		{
			auto jid = joint->ID;
			pDecoder->bases.push_back(dframe[jid].LclRotation);
		}

		sik.SetFeatureDecoder(move(pDecoder));
		// initialize stylized IK for active chains
		sik.SetChain(part.Joints, dframe);

		cout << "Optimal param : " << gpr.get_parameters().transpose() << endl;

	}
}

void CharacterController::InitializeSubacvtivePart(ArmaturePart & part, const Eigen::MatrixXf& Xabpv, tinyxml2::XMLElement * settings)
{
	int pid = part.Index;
	auto& allClipinfo = m_cpxClipinfo;

	auto& rcFacade = allClipinfo.RcFacade;
	//auto& pvFacade = allClipinfo.PvFacade;

	auto& Pv = Xabpv;

	if (rcFacade.GetPartPcaDim(pid) == -1)
		rcFacade.CaculatePartPcaQr(pid);

	auto d = rcFacade.GetPartPcaDim(pid);
	auto X = rcFacade.GetPartPcadSequence(pid);

	if (g_EnableDebugLogging)
	{
		ofstream fout("CharacterAnalayze\\" + m_pCharacter->Name + "_" + part.Joints[0]->Name + ".x.csv");
		fout << X.format(CSVFormat);
		fout.close();
	}

	if (!g_UseStylizedIK)
	{
		assert(!"this code pass is not valiad. as part.Pd is already Pca-ed here");
	}
	else
	{
		auto& sik = m_SIKs[pid];
		auto& gpr = sik.Gplvm();
		gpr.initialize(Pv, X);

		// paramter caching 
		const auto&	partName = part.Joints[0]->Name;

		InitGprXML(settings, partName, gpr);
	}
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

extern Matrix3f FindIsometricTransformXY(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y);

Eigen::PermutationMatrix<Dynamic> upRotatePermutation(int rows, int rotation)
{
	Eigen::PermutationMatrix<Dynamic> perm(rows);

	for (int i = 0; i < rotation; i++)
	{
		perm.indices()[i] = rows - rotation + i;
	}

	for (int i = 0; i < rows - rotation; i++)
	{
		perm.indices()[rotation + i] = i;
	}
	return perm;
}

void FindPartToPartTransform(_Inout_ P2PTransform& transform, const ClipFacade& iclip, const ClipFacade& cclip, size_t phi)
{
	int ju = transform.SrcIdx;
	int jc = transform.DstIdx;
	int T = CLIP_FRAME_COUNT;

	// Up-rotate X to phi
	auto rotX = upRotatePermutation(T, phi);
	auto rawX = (rotX * iclip.GetPartSequence(ju)).eval();

	auto rawY = cclip.GetPartSequence(jc);

	assert(rawX.rows() == rawY.rows() && rawY.rows() == T);

	if (g_PartAssignmentTransform == PAT_CCA)
	{
		PcaCcaMap map;
		map.CreateFrom(rawX, rawY, iclip.PcaCutoff(), cclip.PcaCutoff());
		transform.HomoMatrix = map.TransformMatrix();
	}
	else if (g_PartAssignmentTransform == PAT_OneAxisRotation)
	{
		//RowVectorXf alpha = (rawY.cwiseAbs2().colwise().sum().array() / rawX.cwiseAbs2().colwise().sum().array()).cwiseSqrt();
		//float err = (rawY - rawX * alpha.asDiagonal()).cwiseAbs2().sum();

		auto Transf = FindIsometricTransformXY(rawX, rawY);
		auto rank = rawX.cols();

		transform.HomoMatrix.setIdentity(4, 4);
		transform.HomoMatrix.topLeftCorner(3, 3) = Transf;
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

bool is_symetric(const ArmaturePart& lhs, const ArmaturePart& rhs)
{
	return
		(lhs.SymetricPair != nullptr && lhs.SymetricPair->Index == rhs.Index)
		|| (rhs.SymetricPair != nullptr && rhs.SymetricPair->Index == lhs.Index);
}

void CaculateQuadraticDistanceMatrix(Eigen::Tensor<float, 4> &C, const ClipFacade& iclip, const ClipFacade& cclip)
{
	C.setZero();

	auto& Juk = iclip.ActiveParts();
	auto& Jck = cclip.ActiveParts();
	//const std::vector<int> &Juk, const std::vector<int> &Jck, const Eigen::Array<Eigen::RowVector3f, -1, -1> &XpMean, const Eigen::Array<Eigen::Matrix3f, -1, -1> &XpCov, const Causality::CharacterController & controller);

	auto& cparts = cclip.ArmatureParts();
	auto& sparts = iclip.ArmatureParts();

	for (int i = 0; i < Juk.size(); i++)
	{
		for (int j = i + 1; j < Juk.size(); j++)
		{
			for (int si = 0; si < Jck.size(); si++)
			{
				for (int sj = si + 1; sj < Jck.size(); sj++)
				{
					auto xu = iclip.GetPartsDifferenceMean(Juk[i], Juk[j]);
					auto xc = cclip.GetPartsDifferenceMean(Jck[si], Jck[sj]);
					auto cu = iclip.GetPartsDifferenceCovarience(Juk[i], Juk[j]);
					auto cc = cclip.GetPartsDifferenceCovarience(Jck[si], Jck[sj]);

					auto& sparti = *sparts[i];
					auto& spartj = *sparts[j];
					auto& cparti = *cparts[si];
					auto& cpartj = *cparts[sj];

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

						// structrual bounus
						if (is_symetric(cparti,cpartj))
						{
							if (is_symetric(sparti, spartj))
								val += 0.3f;
						}
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

	DEBUGOUT(C);
}


float CreateControlTransform(CharacterController & controller, const ClipFacade& iclip)
{
	assert(controller.IsReady && iclip.IsReady());

	const size_t pvDim = iclip.GetAllPartDimension();
	// alias setup
	auto& character = controller.Character();
	auto& charaParts = character.Behavier().ArmatureParts();
	auto& userParts = iclip.ArmatureParts();
	size_t Jc = charaParts.size();
	auto& clips = character.Behavier().Clips();
	auto& clipinfos = controller.GetClipInfos();

	controller.CharacterScore = numeric_limits<float>::min();
	//auto& anim = character.Behavier()[DefaultAnimationSet];

	if (character.CurrentAction() == nullptr)
		return 0.0f;

	auto& anim = *character.CurrentAction();

	int T = iclip.ClipFrames(); //? /2 Maybe?
	const std::vector<int> &Juk = iclip.ActiveParts();
	int Ti = g_PhaseMatchingInterval;
	int Ts = T / Ti + 1;

	RowVectorXf Eub(Juk.size());
	selectCols(iclip.GetAllPartsEnergy(), Juk, &Eub);

	// Player Perceptive vector mean normalized
	MatrixXf Xpvnm(pvDim, Juk.size());

	selectCols(reshape(iclip.GetAllPartsMean(), pvDim, -1), Juk, &Xpvnm);
	Xpvnm.colwise().normalize();

	std::vector<unique_ptr<PartilizedTransform>> clipTransforms;
	clipTransforms.reserve(clips.size());
	Eigen::VectorXf clipTransformScores(clips.size());

	//for (auto& cclip : controller.GetClipInfos())	//? <= 5 animation per character
	{
		auto& cclip = controller.GetClipInfo(anim.Name);
		auto& cpv = cclip.PvFacade;

		// Independent Active blocks only
		const auto &Jck = cpv.ActiveParts();

		// Ecb, Energy of Character Active Parts
		RowVectorXf Ecb(Jck.size());
		// Ecb3, Directional Energy of Character Active Parts
		MatrixXf Ecb3(pvDim, Jck.size());

		selectCols(cpv.GetAllPartsEnergy(), Jck, &Ecb);
		//selectCols(cclip.Eb3, Jck, &Ecb3);
		for (size_t i = 0; i < Jck.size(); i++)
			Ecb3.col(i) = cpv.GetPartDimEnergy(Jck[i]);
		Ecb3.colwise().normalize();

		// Character Perceptive vector mean normalized
		MatrixXf Cpvnm(pvDim, Jck.size());
		selectCols(reshape(cpv.GetAllPartsMean(), pvDim, -1), Jck, &Cpvnm);
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
				A(i, j) = sqrtf(((Xpvnm.col(i) - Cpvnm.col(j)).array() * Ecb3.col(j).array()).cwiseAbs2().sum());
			}
		}

		// Anisometric Gaussian kernal here
		A.array() = (-(A.array() / (DirectX::XM_PI / 6)).cwiseAbs2()).exp();
		//A.noalias() = Xsp.transpose() * Csp;

		Tensor<float, 4> C((int)Juk.size(), (int)Juk.size(), (int)Jck.size(), (int)Jck.size());

		CaculateQuadraticDistanceMatrix(C, iclip, cpv);

		vector<DenseIndex> matching(A.cols());

		float score = max_quadratic_assignment(A, C, matching);

		float maxScore = score;

#pragma region Display Debug Armature Parts Info
		cout << "=============================================" << endl;
		cout << "Best assignment for " << character.Name << " : " << anim.Name << endl;
		cout << "Scores : " << maxScore << endl;

		cout << "*********************************************" << endl;
		cout << "Human Skeleton ArmatureParts : " << endl;
		for (auto i : Juk)
		{
			const auto& blX = *userParts[i];
			cout << "Part[" << i << "]= " << blX.Joints << endl;
		}

		cout << "*********************************************" << endl;
		cout << "Character " << character.Name << "'s Skeleton ArmatureParts : " << endl;

		for (auto& i : Jck)
		{
			const auto& blY = *charaParts[i];
			cout << "Part[" << i << "] = " << blY.Joints << endl;
		}
		cout << "__________ Parts Assignment __________" << endl;
		for (int i = 0; i < matching.size(); i++)
		{
			int ju = Juk[i], jc = Jck[matching[i]];
			if (ju >= 0 && jc >= 0)
			{
				cout << userParts[ju]->Joints << " ==> " << charaParts[jc]->Joints << endl;
			}
		}
		cout << "__________ Fin __________" << endl;
#pragma endregion

		Cca<float> cca;
		MatrixXf corrlations(Ts, matching.size());
		corrlations.setZero();

		for (int i = 0; i < matching.size(); i++)
		{
			int ju = Juk[i], jc = Jck[matching[i]];
			for (int phi = 0; phi < T; phi += Ti)
			{
				if (ju >= 0 && jc >= 0)
				{
					cca.computeFromQr(iclip.GetPartPcadQrView(ju), cpv.GetPartPcadQrView(jc), false, phi);
					corrlations(phi / Ti, i) = cca.correlaltions().minCoeff();
				}
				else
					corrlations(phi / Ti, i) = 0;
			}
		}

		VectorXi maxPhi(matching.size());

		//float sumCor = corrlations.rowwise().sum().maxCoeff(&maxPhi);

		int misAlign = Ts / 5;
		//? maybe other reduce function like min?
		//! We should allowed a window of range for phi matching among different parts
		corrlations.conservativeResize(Ts + misAlign, corrlations.cols());
		corrlations.bottomRows(misAlign) = corrlations.topRows(misAlign);
		{
			int mPhis = 0;
			float mScore = numeric_limits<float>::min();;
			for (int i = 0; i < Ts; i++)
			{
				float score = corrlations.middleRows(i, misAlign).colwise().maxCoeff().sum();
				if (score > mScore)
				{
					mPhis = i;
					mScore = score;
				}
			}

			for (int i = 0; i < corrlations.cols(); i++)
			{
				corrlations.middleRows(mPhis, misAlign).col(i).maxCoeff(&maxPhi[i]);
				maxPhi[i] += mPhis;
				if (maxPhi[i] >= Ts) maxPhi[i] -= Ts;
				maxPhi[i] *= Ti;
			}

			// Combine the score from qudratic assignment with phase matching
			maxScore = maxScore * mScore;
		}


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

				FindPartToPartTransform(partTra, iclip, cpv, maxPhi[i]);
			}
		}

		auto pBinding = new PartilizedTransform(userParts, controller);
		pBinding->ActiveParts = move(partTransforms);

		clipTransformScores[clipTransforms.size()] = maxScore;
		clipTransforms.emplace_back(std::move(pBinding));

	} // Animation clip scope

	DenseIndex maxClipIdx = 0;
	float maxScore = clipTransformScores.maxCoeff(&maxClipIdx);
	if (maxScore > controller.CharacterScore * 2.0)
	{

		auto pBinding = move(clipTransforms[maxClipIdx]);

		controller.SetBinding(move(pBinding));
		controller.CharacterScore = maxScore;
	}

	return maxScore;

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

}
