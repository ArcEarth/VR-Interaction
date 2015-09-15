#include "pch_bcl.h"
#include "ArmatureTransforms.h"
#include <iostream>
#include "Settings.h"
#include <PrimitiveVisualizer.h>

extern Eigen::RowVector3d	g_NoiseInterpolation;
namespace Causality
{
	extern float g_DebugArmatureThinkness;
}

using namespace Causality;

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


Causality::BlockEndEffectorGblPosQuadratic::BlockEndEffectorGblPosQuadratic()
{
}

Eigen::RowVectorXf Causality::BlockEndEffectorGblPosQuadratic::Get(const KinematicBlock & block, const AffineFrame & frame)
{
	using namespace Eigen;
	RowVectorXf Y(BoneFeatureType::Dimension);

	BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[block.Joints.back()->ID()]);

	if (block.parent() != nullptr)
	{
		RowVectorXf reference(BoneFeatures::GblPosFeature::Dimension);
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

void Causality::BlockEndEffectorGblPosQuadratic::Set(const KinematicBlock & block, AffineFrame & frame, const Eigen::RowVectorXf & feature)
{
	assert(false);
}

Causality::BlockizedArmatureTransform::BlockizedArmatureTransform() :pSblocks(nullptr), pTblocks(nullptr)
{
	pSource = nullptr;
	pTarget = nullptr;
}

Causality::BlockizedArmatureTransform::BlockizedArmatureTransform(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
{
	SetFrom(pSourceBlock, pTargetBlock);
}

void Causality::BlockizedArmatureTransform::SetFrom(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
{
	pSblocks = pSourceBlock; pTblocks = pTargetBlock;
	pSource = &pSourceBlock->Armature();
	pTarget = &pTargetBlock->Armature();
}

void Causality::BlockizedCcaArmatureTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
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

		map.Apply(X, Y);

		//cout << " X = " << X << endl;
		//cout << " Yr = " << Y << endl;
		pOutputExtractor->Set(*pTb, target_frame, Y);

	}

	//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
	//target_frame[0].LclRotation = source_frame[0].LclRotation;
	target_frame.RebuildGlobal(*pTarget);
}

void Causality::BlockizedCcaArmatureTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const AffineFrame & last_frame, float frame_time) const
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

Causality::PerceptiveVectorBlockFeatureExtractor::PerceptiveVectorBlockFeatureExtractor(std::map<std::string, ClipInfo*>* pClips)
{
	QuadraticInput = true;
	segma = 10;
	pClipInfos = pClips;
}

Eigen::RowVectorXf Causality::PerceptiveVectorBlockFeatureExtractor::Get(const KinematicBlock & block, const AffineFrame & frame)
{
	using namespace Eigen;

	DenseIndex dim = InputFeatureType::Dimension;
	if (QuadraticInput)
		dim += dim * (dim + 1) / 2;

	RowVectorXf Y(dim);

	auto& aJoint = *block.Joints.back();

	BoneFeatures::GblPosFeature::Get(Y.segment<3>(0), frame[aJoint.ID()]);

	if (block.parent() != nullptr)
	{
		RowVectorXf reference(BoneFeatures::GblPosFeature::Dimension);
		BoneFeatures::GblPosFeature::Get(reference, frame[block.parent()->Joints.back()->ID()]);
		Y.segment<3>(0) -= reference;
	}

	if (QuadraticInput)
	{
		ExpandQuadraticTerm(Y, InputFeatureType::Dimension);
	}

	return Y;
}

// Inherited via BlockFeatureExtractor


template <class CharacterFeatureType>
Eigen::Vector3f GetChainEndPositionFromY(const AffineFrame & dFrame, const std::vector<Joint*> & joints, const Eigen::RowVectorXf & Y)
{
	assert(joints.size() * CharacterFeatureType::Dimension == Y.size());

	// a double buffer
	Bone bones[2];
	int sw = 0;

	for (int i = 0; i < joints.size(); i++)
	{
		auto jid = joints[i]->ID();

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


void Causality::PerceptiveVectorBlockFeatureExtractor::Set(const KinematicBlock & block, AffineFrame & frame, const RowVectorX & feature)
{
	using namespace DirectX;
	using namespace Eigen;
	if (block.ActiveActionCount > 0)
	{
		RowVectorXf Y(CharacterFeatureType::Dimension * block.Joints.size());

		if (block.PdCca.Jx == -1 || block.PdCca.Jy == -1) return;

		if (!g_UseStylizedIK)
		{
			block.PdCca.Apply(feature, Y);
		}
		else
		{
			RowVectorXd dY;//(CharacterFeatureType::Dimension * block.Joints.size());
						   // since the feature is expanded to quadritic form, we remove the extra term
			double varZ = 10;

			MatrixXd covObsr(g_PvDimension, g_PvDimension);
			covObsr = g_NoiseInterpolation.replicate(1, 2).transpose().asDiagonal() * varZ;


			auto likilyhood = block.PdGpr.get_expectation_from_observation(feature.cast<double>(), covObsr, &dY);
			//auto likilyhood = block.PdGpr.get_expectation_and_likelihood(feature.cast<double>(), &dY);
			Y = dY.cast<float>();
			Y *= block.Wx.cwiseInverse().asDiagonal(); // Inverse scale feature

			if (g_StepFrame)
				std::cout << block.Joints[0]->Name() << " : " << likilyhood << std::endl;
		}

		for (size_t j = 0; j < block.Joints.size(); j++)
		{
			auto jid = block.Joints[j]->ID();
			auto Xj = Y.segment<CharacterFeatureType::Dimension>(j * CharacterFeatureType::Dimension);
			CharacterFeatureType::Set(frame[jid], Xj);
		}
	}
}

Causality::BlockizedSpatialInterpolateQuadraticTransform::BlockizedSpatialInterpolateQuadraticTransform(std::map<std::string, ClipInfo*>* pClips)
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

Causality::BlockizedSpatialInterpolateQuadraticTransform::BlockizedSpatialInterpolateQuadraticTransform(std::map<std::string, ClipInfo*>* pClips, const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock)
	: BlockizedSpatialInterpolateQuadraticTransform(pClips)
{
	SetFrom(pSourceBlock, pTargetBlock);
	pvs.resize(pTargetBlock->size());
}

void Causality::BlockizedSpatialInterpolateQuadraticTransform::Render(DirectX::CXMVECTOR color, DirectX::CXMMATRIX world)
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

void Causality::BlockizedSpatialInterpolateQuadraticTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
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

void Causality::BlockizedSpatialInterpolateQuadraticTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const AffineFrame & last_frame, float frame_time) const
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
		else if (g_UseVelocity)
		{
			Yi.segment<3>(3) = Vector3f::Map(&pvs[map.Jy].first.x);
		}

		abcount++;
	}

	// Dependent blocks
	for (const auto& map : Maps)
	{
		if (map.Jx == -2 && map.Jy >= 0)
		{
			if (!g_UseStylizedIK)
			{
				if (map.uXpca.size() == Xabpv.size())
				{
					auto pTb = tblocks[map.Jy];
					auto Y = pOutputExtractor->Get(*pTb, target_frame);
					map.Apply(Xabpv, Y);
					pDependentBlockFeature->Set(*pTb, target_frame, Y);
				}
			}
			else
			{
				auto pTb = tblocks[map.Jy];
				auto Y = pOutputExtractor->Get(*pTb, target_frame);
				pOutputExtractor->Set(*pTb, target_frame, Xabpv);
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
