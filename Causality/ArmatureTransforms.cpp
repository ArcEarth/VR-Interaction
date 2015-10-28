#include "pch_bcl.h"
#include "ArmatureTransforms.h"
#include <iostream>
#include "Settings.h"
#include <PrimitiveVisualizer.h>
#include "CharacterController.h"
#include "EigenExtension.h"
#include <set>

extern Eigen::RowVector3d	g_NoiseInterpolation;
namespace Causality
{
	extern float g_DebugArmatureThinkness;
}

using namespace Causality;
using namespace ArmaturePartFeatures;
using namespace Eigen;

typedef
Localize<EndEffector<InputFeature>>
InputExtractorType;

typedef
Pcad <
	Weighted <
	RelativeDeformation <
	AllJoints < CharacterFeature > > > >
	OutputExtractorType;


#define BEGIN_TO_END(range) range.begin(), range.end()

#ifdef _DEBUG
#define DEBUGOUT(x) std::cout << #x << " = " << x << std::endl
#else
#define DEBUGOUT(x)
#endif

template <class Derived>
void ExpandQuadraticTerm(_Inout_ Eigen::DenseBase<Derived> &v, _In_ Eigen::DenseIndex dim)
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


EndEffectorGblPosQuadratized::EndEffectorGblPosQuadratized()
{
}

int EndEffectorGblPosQuadratized::GetDimension(const ArmaturePart& block)
{
	return 9;
}

Eigen::RowVectorXf EndEffectorGblPosQuadratized::Get(const ArmaturePart & block, const BoneHiracheryFrame & frame)
{
	using namespace Eigen;
	RowVectorXf Y(BoneFeatureType::Dimension);

	BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[block.Joints.back()->ID]);

	if (block.parent() != nullptr)
	{
		RowVectorXf reference(BoneFeatures::GblPosFeature::Dimension);
		BoneFeatures::GblPosFeature::Get(reference, frame[block.parent()->Joints.back()->ID]);
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

// Inherited via IArmaturePartFeature

void EndEffectorGblPosQuadratized::Set(const ArmaturePart & block, BoneHiracheryFrame & frame, const Eigen::RowVectorXf & feature)
{
	assert(false);
}

BlockizedArmatureTransform::BlockizedArmatureTransform() :pSblocks(nullptr), pTblocks(nullptr)
{
	pSource = nullptr;
	pTarget = nullptr;
}

BlockizedArmatureTransform::BlockizedArmatureTransform(const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock)
{
	SetFrom(pSourceBlock, pTargetBlock);
}

void BlockizedArmatureTransform::SetFrom(const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock)
{
	pSblocks = pSourceBlock; pTblocks = pTargetBlock;
	pSource = &pSourceBlock->Armature();
	pTarget = &pTargetBlock->Armature();
}

void BlockizedCcaArmatureTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
{
	using namespace Eigen;
	using namespace std;
	const auto& sblocks = *pSblocks;
	const auto& tblocks = *pTblocks;
	RowVectorXf X, Y;
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
				if (pBlock->ActiveActions.size() > 0)
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

		map.Apply(X, Y);

		//cout << " X = " << X << endl;
		//cout << " Yr = " << Y << endl;
		pOutputExtractor->Set(*pTb, target_frame, Y);

	}

	//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
	//target_frame[0].LclRotation = source_frame[0].LclRotation;
	target_frame.RebuildGlobal(*pTarget);
}

void BlockizedCcaArmatureTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const BoneHiracheryFrame & last_frame, float frame_time) const
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

		map.Apply(X, Y);

		//cout << " X = " << X << endl;
		//cout << " Yr = " << Y << endl;
		pOutputExtractor->Set(*pTb, target_frame, Y);
	}

	//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
	//target_frame[0].LclRotation = source_frame[0].LclRotation;
	target_frame.RebuildGlobal(*pTarget);
}

Causality::ArmaturePartFeatures::PerceptiveVector::PerceptiveVector(CharacterController & controller)
	: m_pController(&controller)
{

}

Eigen::RowVectorXf PerceptiveVector::Get(const ArmaturePart & block, const BoneHiracheryFrame & frame)
{
	using namespace Eigen;

	DenseIndex dim = InputFeatureType::Dimension;
	if (Quadratic)
		dim += dim * (dim + 1) / 2;

	RowVectorXf Y(dim);

	auto& aJoint = *block.Joints.back();

	BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[aJoint.ID]);

	if (block.parent() != nullptr)
	{
		RowVectorXf reference(BoneFeatures::GblPosFeature::Dimension);
		BoneFeatures::GblPosFeature::Get(reference, frame[block.parent()->Joints.back()->ID]);
		Y.segment<3>(0) -= reference;
	}

	if (Quadratic)
	{
		ExpandQuadraticTerm(Y, InputFeatureType::Dimension);
	}

	return Y;
}

// Inherited via IArmaturePartFeature


template <class CharacterFeatureType>
Eigen::Vector3f GetChainEndPositionFromY(const BoneHiracheryFrame & dFrame, const std::vector<Joint*> & joints, const Eigen::RowVectorXf & Y)
{
	assert(joints.size() * CharacterFeatureType::Dimension == Y.size());

	// a double buffer
	Bone bones[2];
	int sw = 0;

	for (int i = 0; i < joints.size(); i++)
	{
		auto jid = joints[i]->ID;

		// set scale and translation
		bones[sw].LocalTransform() = dFrame[jid].LocalTransform();

		auto Yj = Y.segment<CharacterFeatureType::Dimension>(i * CharacterFeatureType::Dimension);
		// set local rotation
		CharacterFeatureType::Set(bones[sw], Yj);

		// update global data
		bones[sw].UpdateGlobalData(bones[1 - sw]);
	}

	Eigen::Vector3f pos = Eigen::Vector3f::MapAligned(&bones[sw].GblTranslation.x);
	return pos;
}


void PerceptiveVector::Set(const ArmaturePart & block, BoneHiracheryFrame & frame, const Eigen::RowVectorXf& feature)
{
	using namespace DirectX;
	using namespace Eigen;
	if (block.ActiveActions.size() > 0)
	{
		RowVectorXf Y(CharacterFeatureType::Dimension * block.Joints.size());

		if (block.ActiveActions.size() + block.SubActiveActions.size() == 0) return;

		if (!g_UseStylizedIK)
		{
			//cpart.PdCca.Apply(feature, Y);
		}
		else
		{
			RowVectorXd dY;//(CharacterFeatureType::Dimension * cpart.Joints.size());
						   // since the feature is expanded to quadritic form, we remove the extra term
			double varZ = 10;

			//MatrixXd covObsr(g_PvDimension, g_PvDimension);
			//covObsr = g_NoiseInterpolation.replicate(1, 2).transpose().asDiagonal() * varZ;

			if (g_PvDimension == 6 && g_UseVelocity)
			{
				auto dX = feature.cast<double>().eval();
				auto& sik = m_pController->GetStylizedIK(block.Index);
				auto& gpr = sik.Gplvm();
				sik.SetBaseRotation(frame[block.parent()->Joints.back()->ID].GblRotation);
				dY = sik.Apply(dX.segment<3>(0).transpose().eval(), dX.segment<3>(3).transpose().eval());
			}
			double likilyhood = 1.0;

			//auto likilyhood = cpart.PdGpr.get_expectation_from_observation(feature.cast<double>(), covObsr, &dY);
			//auto likilyhood = cpart.PdGpr.get_expectation_and_likelihood(feature.cast<double>(), &dY);
			Y = dY.cast<float>();
			//Y *= cpart.Wx.cwiseInverse().asDiagonal(); // Inverse scale feature

			if (g_StepFrame)
				std::cout << block.Joints[0]->Name << " : " << likilyhood << std::endl;
		}

		for (size_t j = 0; j < block.Joints.size(); j++)
		{
			auto jid = block.Joints[j]->ID;
			auto Xj = Y.segment<CharacterFeatureType::Dimension>(j * CharacterFeatureType::Dimension);
			CharacterFeatureType::Set(frame[jid], Xj);
		}
	}
}

RBFInterpolationTransform::RBFInterpolationTransform(gsl::array_view<CharacterClipinfo> clips)
{
	Clipinfos = clips;
	auto pIF = new PerceptiveVector(*m_pController);
	pIF->Quadratic = true;
	pInputExtractor.reset(pIF);

	auto pOF = new PerceptiveVector(*m_pController);
	pOF->Segma = 30;
	pOutputExtractor.reset(pOF);

	pDependentBlockFeature.reset(new AllJoints<CharacterFeature>());
}

RBFInterpolationTransform::RBFInterpolationTransform(gsl::array_view<CharacterClipinfo> clips, const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock)
	: RBFInterpolationTransform(clips)
{
	SetFrom(pSourceBlock, pTargetBlock);
	pvs.resize(pTargetBlock->size());
}

void RBFInterpolationTransform::Render(DirectX::CXMVECTOR color, DirectX::CXMMATRIX world)
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

void RBFInterpolationTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
{
	const auto& sblocks = *pSblocks;
	const auto& tblocks = *pTblocks;

	using namespace Eigen;

	int abcount = 0;
	for (const auto& map : Maps)
	{
		//Y = X;
		if (map.Jx < 0 || map.Jy < 0) continue;

		auto pSb = sblocks[map.Jx];
		auto pTb = tblocks[map.Jy];
		auto X = pInputExtractor->Get(*pSb, source_frame);
		auto Y = pOutputExtractor->Get(*pTb, target_frame);


		map.Apply(X.leftCols<3>(), Y);

		Y.conservativeResize(9);
		ExpandQuadraticTerm(Y, 3);

		//cout << " X = " << X << endl;
		//cout << " Yr = " << Y << endl;
		pOutputExtractor->Set(*pTb, target_frame, Y);

		abcount++;
		pvs[map.Jy].second = DirectX::Vector3(Y.segment<3>(0).data());
	}

	target_frame.RebuildGlobal(*pTarget);

	// Fill Xabpv
	RowVectorXf Xabpv;
	Xabpv.resize(abcount * g_PvDimension);
	abcount = 0;
	for (const auto& map : Maps)
	{
		if (map.Jx < 0 || map.Jy < 0) continue;

		auto Yi = Xabpv.segment(abcount * g_PvDimension, g_PvDimension);

		Yi.segment<3>(0) = Vector3f::Map(&pvs[map.Jy].second.x);
		if (g_PvDimension == 9)
		{
			ExpandQuadraticTerm(Yi, 3);
		}

		abcount++;
	}

	// Dependent cparts
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
			pvs[map.Jy].first = target_frame[pTb->parent()->Joints.back()->ID].GblTranslation;
		pvs[map.Jy].second += pvs[map.Jy].first;
	}

}

void RBFInterpolationTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const BoneHiracheryFrame & last_frame, float frame_time) const
{
	using namespace Eigen;
	using namespace DirectX;
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
		auto Xl = pInputExtractor->Get(*pSb, last_frame);
		auto Y = pOutputExtractor->Get(*pTb, target_frame);
		auto Yl = Y;

		map.Apply(X.leftCols<3>(), Y);
		map.Apply(Xl.leftCols<3>(), Yl);

		if (g_PvDimension == 9)
		{
			Y.conservativeResize(9);
			ExpandQuadraticTerm(Y, 3);
		}
		if (g_UseVelocity)
		{
			assert(Y.size() == Yl.size() && Y.size() == 3);
			Y.conservativeResize(6);
			Y.segment<3>(3) = (Y.segment<3>(0) - Yl) / (frame_time * g_FrameTimeScaleFactor);
		}

		//cout << " X = " << X << endl;
		//cout << " Yr = " << Y << endl;
		pOutputExtractor->Set(*pTb, target_frame, Y);

		abcount++;
		pvs[map.Jy].first = Vector3(Yl.data());
		pvs[map.Jy].second = Vector3(Y.segment<3>(0).data());
	}

	//target_frame.RebuildGlobal(*pTarget);

	// Fill Xabpv
	RowVectorXf Xabpv;
	Xabpv.resize(abcount * g_PvDimension);
	abcount = 0;
	for (const auto& map : Maps)
	{
		if (map.Jx < 0 || map.Jy < 0) continue;

		auto Yi = Xabpv.segment(abcount * g_PvDimension, g_PvDimension);

		Yi.segment<3>(0) = Vector3f::Map(&pvs[map.Jy].second.x);
		if (g_PvDimension == 9)
		{
			ExpandQuadraticTerm(Yi, 3);
		}
		else if (g_UseVelocity)
		{
			Yi.segment<3>(3) = Vector3f::Map(&pvs[map.Jy].first.x);
		}

		abcount++;
	}

	// Dependent cparts
	for (const auto& map : Maps)
	{
		if (map.Jx == -2 && map.Jy >= 0)
		{
			auto pTb = tblocks[map.Jy];
			RowVectorXf Y = pDependentBlockFeature->Get(*pTb, target_frame);

			if (!g_UseStylizedIK)
			{
				if (map.uXpca.size() == Xabpv.size())
				{
					map.Apply(Xabpv, Y);
					pDependentBlockFeature->Set(*pTb, target_frame, Y);
				}
			}
			else
			{
				if (pTb->SubActiveActions.size() > 0 && map.uXpca.size() == Xabpv.size())
				{
					auto _x = (Xabpv - map.uXpca) * map.pcX;

					RowVectorXd Yd;
					auto& sik = m_pController->GetStylizedIK(pTb->Index);
					auto& gpr = sik.Gplvm();
					auto lk = gpr.get_expectation_and_likelihood(_x.cast<double>(), &Yd);

					Yd.array() /= pTb->Wx.array().cast<double>();
					Y = Yd.cast<float>();// *pTb->Wx.cwiseInverse().asDiagonal();
					//Y.setOnes();

					pDependentBlockFeature->Set(*pTb, target_frame, Y);
				}
			}
		}
	}

	target_frame.RebuildGlobal(*pTarget);

	for (const auto& map : Maps)
	{
		auto pTb = tblocks[map.Jy];

		if (pTb->parent())
			pvs[map.Jy].first = target_frame[pTb->parent()->Joints.back()->ID].GblTranslation;
		pvs[map.Jy].second += pvs[map.Jy].first;
	}

}

PartilizedTransform::PartilizedTransform(const ShrinkedArmature& sParts, CharacterController & controller)
	: m_pHandles(nullptr), m_pController(nullptr)
{
	m_pController = &controller;
	m_pHandles = &controller.PvHandles();
	auto& parts = controller.ArmatureParts();
	auto& armature = controller.Armature();
	pSource = &sParts.Armature();
	pTarget = &armature;
	pTblocks = &parts;
	pSblocks = &sParts;

	auto pIF = std::make_shared<InputExtractorType>();
	auto pOF = std::make_shared<OutputExtractorType>();

	auto& ucinfo = controller.GetUnitedClipinfo();

	pOF->InitPcas(parts.size());
	pOF->SetDefaultFrame(armature.default_frame());
	pOF->InitializeWeights(parts);

	auto& facade = ucinfo.RcFacade;
	auto cutoff = facade.PcaCutoff();
	for (auto part : parts)
	{
		int pid = part->Index;
		if (part->ActiveActions.size() > 0 || part->SubActiveActions.size() > 0)
		{
			auto &pca = facade.GetPartPca(pid);
			auto d = facade.GetPartPcaDim(pid);
			pOF->SetPca(pid, pca.components(d), pca.mean());
		}
		else
		{
			int odim = facade.GetPartDimension(pid);
			pOF->SetPca(pid, MatrixXf::Identity(odim, odim), facade.GetPartMean(pid));
		}
	}

	m_pActiveF = pOF;
	m_pInputF = pIF;
}

void Causality::PartilizedTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
{
	Transform(target_frame, source_frame, source_frame, 0);
}

void PartilizedTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const BoneHiracheryFrame & last_frame, float frame_time) const
{
	const auto& cparts = *pTblocks;
	const auto& sparts = *pSblocks;

	auto pvDim = m_pInputF->GetDimension(*sparts[0]);

	RowVectorXd Xd(g_UseVelocity ? pvDim * 2 : pvDim), Xld(g_UseVelocity ? pvDim * 2 : pvDim), Y;
	double semga = 1000;
	RowVectorXf yf;

	RowVectorXd AvtiveVector(m_pController->uXabpv.size());
	RowVectorXd DrivenActiveVector;

	int xstdim = 0;
	for (auto& ctrl : this->ActiveParts)
	{
		assert(ctrl.DstIdx >= 0 && ctrl.SrcIdx >= 0);
		auto& cpart = const_cast<ArmaturePart&>(*cparts[ctrl.DstIdx]);
		auto& spart = *sparts[ctrl.SrcIdx];

		auto& sik = m_pController->GetStylizedIK(cpart.Index);
		auto& gpr = sik.Gplvm();
		auto& joints = cpart.Joints;

		auto baseRot = target_frame[cpart.parent()->Joints.back()->ID].GblRotation;

		sik.SetBaseRotation(baseRot);
		sik.SetChain(cpart.Joints, target_frame);

		RowVectorXf xf = m_pInputF->Get(spart, source_frame);//,last_frame,frame_time
		RowVectorXf xlf = m_pInputF->Get(spart, last_frame);

		TransformCtrlHandel(xf, ctrl);
		TransformCtrlHandel(xlf, ctrl);

		yf = m_pActiveF->Get(cpart, target_frame);

		Xd.segment(0, pvDim) = xf.cast<double>();

		if (g_UseVelocity)
		{
			auto Xv = Xd.segment(pvDim, pvDim);
			Xv = (xf - xlf).cast<double>() / (frame_time * g_FrameTimeScaleFactor);
		}

		if (m_pHandles)
		{
			m_pHandles->at(cpart.Index).first = Vector3(xf.data());
			if (g_UseVelocity && g_PvDimension == 6)
			{
				m_pHandles->at(cpart.Index).second = Vector3(xf.data() + 3);
			}
			else
			{
				m_pHandles->at(cpart.Index).second = Vector3::Zero;
			}
		}

		AvtiveVector.segment(xstdim, Xd.size()) = Xd;
		xstdim += Xd.size();

		if (!g_UseVelocity)
			Y = sik.Apply(Xd.transpose());
		else
			Y = sik.Apply(Xd.segment(0, pvDim).transpose(), Vector3d(Xd.segment(pvDim, pvDim).transpose()));

		m_pActiveF->Set(cpart, target_frame, Y.cast<float>());

		for (int i = 0; i < joints.size(); i++)
		{
			target_frame[joints[i]->ID].UpdateGlobalData(target_frame[joints[i]->parent()->ID]);
		}

		auto ep2 = target_frame[joints.back()->ID].GblTranslation -
			target_frame[joints[0]->parent()->ID].GblTranslation;

		//break;

	}

	// Fill Xabpv
	if (g_EnableDependentControl)
	{
		auto _x = (AvtiveVector.cast<float>() - m_pController->uXabpv) * m_pController->XabpvT;
		auto _xd = _x.cast<double>();

		for (auto ctrl : DrivenParts)
		{
			auto& cpart = const_cast<ArmaturePart&>(*cparts[ctrl.DstIdx]);

			auto& sik = m_pController->GetStylizedIK(cpart.Index);
			auto& gpr = sik.Gplvm();

			auto lk = gpr.get_expectation_and_likelihood(_xd, &Y);

			yf = Y.cast<float>();

			m_pDrivenF->Set(cpart, target_frame, yf);

		}
	}

	target_frame[0].LclTranslation = source_frame[0].LclTranslation;
	target_frame[0].GblTranslation = source_frame[0].GblTranslation;
	target_frame.RebuildGlobal(*pTarget);

}

void Causality::PartilizedTransform::TransformCtrlHandel(Eigen::RowVectorXf &xf, const Causality::P2PTransform & ctrl) const
{
	auto& homo = ctrl.HomoMatrix;
	xf *= homo.topLeftCorner(homo.rows() - 1, homo.cols() - 1);
	xf += homo.block(homo.rows() - 1, 0, 1, homo.cols() - 1);
}

using namespace std;

void Causality::PartilizedTransform::GenerateDrivenAccesseryControl()
{
	auto& allclip = m_pController->GetUnitedClipinfo();
	auto& controller = *m_pController;

	int pvDim = allclip.PvFacade.GetAllPartDimension();

	// Build aparts, dparts
	VectorXi aparts(this->ActiveParts.size());
	set<int> drivSet(BEGIN_TO_END(allclip.ActiveParts()));
	for (int i = 0; i < this->ActiveParts.size(); i++)
	{
		aparts[i] = this->ActiveParts[i].DstIdx;
		drivSet.erase(aparts[i]);
	}
	VectorXi dparts(drivSet.size());
	int i = 0;
	for (auto& pid : drivSet)
		dparts[i++] = pid;

	// Select Xapvs & dpavs
	MatrixXf Xapvs(allclip.ClipFrames(), pvDim * aparts.size());
	MatrixXi cache = aparts.replicate(1, 3).transpose();
	selectCols(allclip.PvFacade.GetAllPartsSequence(), VectorXi::Map(cache.data(), cache.size()), &Xapvs);

	//MatrixXf Xdpvs(allclip.ClipFrames(), pvDim * dparts.size());
	//cache = dparts.replicate(1, 3).transpose();
	//selectCols(allclip.PvFacade.GetAllPartsSequence(), VectorXi::Map(cache.data(), cache.size()), &Xdpvs);

	Pca<MatrixXf> pcaXapvs(Xapvs);
	int dXapv = pcaXapvs.reducedRank(g_CharacterPcaCutoff);
	QrStore<MatrixXf> qrXabpvs(pcaXapvs.coordinates(dXapv));
	Cca<float> cca;

	PcaCcaMap map;
	DrivenParts.clear();
	for (int i = 0; i < dparts.size(); i++)
	{
		auto pid = dparts[i];
		DrivenParts.emplace_back();
		auto &ctrl = DrivenParts.back();
		ctrl.DstIdx = pid;
		ctrl.SrcIdx = PvInputTypeEnum::ActiveParts;

		int d = allclip.PvFacade.GetPartPcaDim(pid);
		auto& pca = allclip.PvFacade.GetPartPca(pid);
		auto qr = allclip.PvFacade.GetPartPcadQrView(pid);

		cca.computeFromQr(qrXabpvs, qr, true, 0);
		float corr = cca.correlaltions().minCoeff();

		map.A = cca.matrixA();
		map.B = cca.matrixB();
		map.uX = qrXabpvs.mean();
		map.uY = qr.mean();
		map.pcX = pcaXapvs.components(dXapv);
		map.uXpca = pcaXapvs.mean();
		map.pcY = pca.components(d);
		map.uYpca = pca.mean();

		ctrl.HomoMatrix = map.TransformMatrix();
	}
}
