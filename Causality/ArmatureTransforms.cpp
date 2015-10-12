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
using namespace ArmaturePartFeatures;
using namespace Eigen;

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

PerceptiveVector::PerceptiveVector(gsl::array_view<ClipInfo>		clipinfos)
	: Clipinfos(clipinfos)
{
	QuadraticInput = true;
	segma = 10;
}

Eigen::RowVectorXf PerceptiveVector::Get(const ArmaturePart & block, const BoneHiracheryFrame & frame)
{
	using namespace Eigen;

	DenseIndex dim = InputFeatureType::Dimension;
	if (QuadraticInput)
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

	if (QuadraticInput)
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

			//MatrixXd covObsr(g_PvDimension, g_PvDimension);
			//covObsr = g_NoiseInterpolation.replicate(1, 2).transpose().asDiagonal() * varZ;

			if (g_PvDimension == 6 && g_UseVelocity)
			{
				auto dX = feature.cast<double>().eval();
				auto& sik = const_cast<StylizedChainIK&>(block.PdStyleIk);
				sik.SetBaseRotation(frame[block.parent()->Joints.back()->ID].GblRotation);
				dY = sik.Apply(dX.segment<3>(0).transpose().eval(), dX.segment<3>(3).transpose().eval());
			}
			double likilyhood = 1.0;

			//auto likilyhood = block.PdGpr.get_expectation_from_observation(feature.cast<double>(), covObsr, &dY);
			//auto likilyhood = block.PdGpr.get_expectation_and_likelihood(feature.cast<double>(), &dY);
			Y = dY.cast<float>();
			//Y *= block.Wx.cwiseInverse().asDiagonal(); // Inverse scale feature

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

RBFInterpolationTransform::RBFInterpolationTransform(gsl::array_view<ClipInfo> clips)
{
	Clipinfos = clips;
	auto pIF = new PerceptiveVector(clips);
	pIF->QuadraticInput = true;
	pInputExtractor.reset(pIF);

	auto pOF = new PerceptiveVector(clips);
	pOF->segma = 30;
	pOutputExtractor.reset(pOF);

	pDependentBlockFeature.reset(new AllJoints<CharacterFeature>());
}

RBFInterpolationTransform::RBFInterpolationTransform(gsl::array_view<ClipInfo> clips, const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock)
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

	// Dependent blocks
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
				if (pTb->SubActiveActionCount > 0 && map.uXpca.size() == Xabpv.size())
				{
					auto _x = (Xabpv - map.uXpca) * map.pcX;

					RowVectorXd Yd;
					auto lk = pTb->PdGpr.get_expectation_and_likelihood(_x.cast<double>(), &Yd);

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

PartilizedTransform::PartilizedTransform()
{
	m_pInputF.reset(new EndEffector<InputFeature>());
	m_pActiveF.reset(new AllJointRltLclRotLnQuatPcad());
	m_pDrivenF.reset(new AllJointRltLclRotLnQuatPcad());
	m_pAccesseryF.reset(new AllJointRltLclRotLnQuatPcad());
}

void Causality::PartilizedTransform::Transform(frame_type & target_frame, const frame_type & source_frame) const
{
	Transform(target_frame, source_frame, source_frame, 0);
}

void PartilizedTransform::Transform(frame_type & target_frame, const frame_type & source_frame, const BoneHiracheryFrame & last_frame, float frame_time) const
{
	const auto& blocks = *pTblocks;
	RowVectorXd Xd(g_PvDimension),Xld, Y;
	double semga = 1000;
	RowVectorXf yf;

	RowVectorXf AvtiveVector;
	RowVectorXf DrivenActiveVector;

	for (auto& ctrl : this->AccesseryParts)
	{
		auto block = const_cast<ArmaturePart*>(blocks[ctrl.DstIdx]);

		if (block->ActiveActionCount > 0)
		{
			RowVectorXf xf = m_pInputF->Get(*block, source_frame);//,last_frame,frame_time
			RowVectorXf xlf = m_pInputF->Get(*block, last_frame);
			yf = m_pAccesseryF->Get(*block, target_frame);

			Xd.segment<3>(0) = xf.cast<double>();
			Xld.segment<3>(0) = xlf.cast<double>();

			if (g_UseVelocity && g_PvDimension == 6)
			{
				auto Xv = Xd.segment<3>(3);
				Xv = (Xd - Xld) / (frame_time * g_FrameTimeScaleFactor);
			}

			xf = Xd.cast<float>();

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

			//m_Xs.row(block->Index) = Xd;
			//Xabs.emplace_back(Xd);

			MatrixXd covObsr(g_PvDimension, g_PvDimension);
			covObsr.setZero();
			//! This is not correct!!!
			covObsr.diagonal() = g_NoiseInterpolation.replicate(1, g_PvDimension / 3).transpose();

			auto baseRot = target_frame[block->parent()->Joints.back()->ID].GblRotation;
			block->PdStyleIk.SetBaseRotation(baseRot);
			block->PdStyleIk.SetChain(block->Joints, target_frame);
			block->PdStyleIk.SetGplvmWeight(block->Wx.cast<double>());

			if (!g_UseVelocity)
				Y = block->PdStyleIk.Apply(Xd.transpose());
			else
				Y = block->PdStyleIk.Apply(Xd.leftCols<3>().transpose(), Xd.segment<3>(3).transpose().eval());

			m_pActiveF->Set(*block, target_frame, Y.cast<float>());
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
	//if (g_EnableDependentControl)
	//{
	//	RowVectorXd Xabpv;
	//	Xabpv.resize(Xabs.size() * g_PvDimension);
	//	int i = 0;
	//	for (const auto& xab : Xabs)
	//	{
	//		auto Yi = Xabpv.segment(i * g_PvDimension, g_PvDimension);
	//		Yi = xab;
	//		++i;
	//	}


	//	for (auto& ctrl : this->AccesseryParts)
	//	{
	//		auto block = blocks[ctrl.DstIdx];
	//		if (block->ActiveActionCount == 0 && block->SubActiveActionCount > 0)
	//		{
	//			auto _x = (Xabpv.cast<float>() - block->PdCca.uXpca) * block->PdCca.pcX;

	//			auto lk = block->PdGpr.get_expectation_and_likelihood(_x.cast<double>(), &Y);

	//			yf = Y.cast<float>();
	//			yf *= block->Wx.cwiseInverse().asDiagonal();

	//			m_pAccesseryF->Set(*block, target_frame, yf);
	//		}

	//	}
	//}

	target_frame[0].LclTranslation = source_frame[0].LclTranslation;
	target_frame[0].GblTranslation = source_frame[0].GblTranslation;
	target_frame.RebuildGlobal(*pTarget);

}