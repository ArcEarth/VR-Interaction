#include "pch_bcl.h"
#include "AnimationAnalyzer.h"
#include <algorithm>
#include <Eigen\fft>

using namespace Causality;
using namespace std;

Causality::AnimationAnalyzer::AnimationAnalyzer(BlockArmature * pBArm)
	:pBlockArmature(pBArm), PcaCutoff(0.02f)
{
}

AnimationAnalyzer::~AnimationAnalyzer()
{
}

void Causality::AnimationAnalyzer::ComputeFromFrames(const std::vector<AffineFrame>& frames)
{
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();
	auto frameCount = frames.size();

	DirectX::Vector3 sq[2];
	auto mapped = Eigen::Matrix<float, 1, DimPerBone>::Map(&sq[0].x);
	for (size_t i = 0; i < frameCount; i++)
	{
		for (size_t j = 0; j < numBones; j++)
		{
			using namespace DirectX;
			using namespace Eigen;
			auto& feature = X.block<1, DimPerBone>(i, j * DimPerBone);
			auto& bone = frames[i][j];
			CharacterFeature::Get(feature, bone);
		}
	}

	BlocklizationAndComputeEnergy();
	ComputePcaQr();
	ComputeSpatialTraits(frames);
}

void Causality::AnimationAnalyzer::ComputeFromBlocklizedMat(const Eigen::MatrixXf & mat)
{
	X = mat;
	ComputePcaQr();
}

void Causality::AnimationAnalyzer::BlocklizationAndComputeEnergy()
{
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();

	Eigen::FFT<float> fft;
	Eigen::MatrixXcf Xf(X.rows(), X.cols());
	for (size_t i = 0; i < X.cols(); i++)
	{
		fft.fwd(Xf.col(i).data(), X.col(i).data(), X.rows());
	}

	Eigen::VectorXf Ecd = Xf.middleRows(1, 5).cwiseAbs2().colwise().sum();
	auto Ecjm = Eigen::Matrix<float, DimPerBone, -1>::Map(Ecd.data(), DimPerBone, numBones);
	Ej = Ecjm.colwise().sum();

	const auto& blocks = *pBlockArmature;
	Eb.resize(blocks.size());
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;

		Eigen::MatrixXf Yb(X.rows(), block->GetFeatureDim<CharacterFeature>());
		for (size_t j = 0; j < joints.size(); j++)
		{
			Yb.middleCols<DimPerBone>(j * DimPerBone) = X.middleCols<DimPerBone>(joints[j]->ID() * DimPerBone);
			Eb(i) = max(Eb(i), Ej(joints[j]->ID()));
		}

		Xbs[i] = Yb;
	}
}

void Causality::AnimationAnalyzer::ComputePcaQr()
{
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();

	const auto& blocks = *pBlockArmature;
	auto bSize = blocks.size();

	Qrs.resize(bSize);
	Pcas.resize(bSize);
	Xbs.resize(bSize);
	Sp.setZero(DimPerBone, bSize);
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;

		auto &Yb = Xbs[i];
		auto & pca = Pcas[i];
		pca.compute(Yb, true);
		auto d = pca.reducedRank(PcaCutoff);
		Qrs[i].compute(pca.coordinates(d), true);
	}
}

void Causality::AnimationAnalyzer::ComputeSpatialTraits(const std::vector<AffineFrame> &frames)
{
	auto frameCount = frames.size();
	const auto& blocks = *pBlockArmature;
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;
		auto lastJid = joints.back()->ID();

		auto vtc = Sp.col(block->Index);
		for (const auto& frame : frames)
		{
			vtc += Eigen::Vector3f::MapAligned(&frame[lastJid].EndPostion.x);
		}
		vtc /= frameCount;

		if (block->parent() != nullptr)
		{
			auto pi = block->parent()->Index;
			Sp.col(i) -= Sp.col(pi);
		}
	}
	Sp.colwise().normalize();
}
