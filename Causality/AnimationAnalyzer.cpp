#include "pch_bcl.h"
#include "AnimationAnalyzer.h"
#include <algorithm>
#include <Eigen\fft>
#include <ppl.h>

using namespace Causality;
using namespace std;
using namespace Concurrency;

AnimationAnalyzer::AnimationAnalyzer(const BlockArmature & bArm)
	:pBlockArmature(&bArm), PcaCutoff(0.01f), IsReady(false), EnergyCutoff(0.35f), PerceptiveVectorReconstructor(pBlockArmature->size())
{
}

AnimationAnalyzer::~AnimationAnalyzer()
{
}

task<void> AnimationAnalyzer::ComputeFromFramesAsync(const std::vector<AffineFrame>& frames)
{
	return create_task([this, &frames]() {
		this->ComputeFromFrames(frames);
	});
	//pComputingThread = make_unique<thread>(&AnimationAnalyzer::ComputeFromFrames,this, std::cref(frames));
}

void AnimationAnalyzer::ComputeFromFrames(const std::vector<AffineFrame>& frames)
{
	//std::cout << "Computation start" << endl;
	IsReady = false;
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();
	auto frameCount = frames.size();
	X.resize(frameCount, numBones * DimPerBone);

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

	ComputeSpatialTraits(frames);
	BlocklizationAndComputeEnergy(frames);
	ComputePcaQr();
	IsReady = true;
	//std::cout << "Computation finished" << endl;
}

void AnimationAnalyzer::ComputeFromBlocklizedMat(const Eigen::MatrixXf & mat)
{
	X = mat;
	ComputePcaQr();
}

void AnimationAnalyzer::BlocklizationAndComputeEnergy(const std::vector<AffineFrame>& frames)
{
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();
	auto frameCount = frames.size();

	//Eigen::FFT<float> fft;
	//Eigen::MatrixXcf Xf(X.rows(), X.cols());
	//for (size_t i = 0; i < X.cols(); i++)
	//{
	//	fft.fwd(Xf.col(i).data(), X.col(i).data(), X.rows());
	//}

	//Eigen::VectorXf Ecd = Xf.middleRows(1, 5).cwiseAbs2().colwise().sum();
	//auto Ecjm = Eigen::Matrix<float, DimPerBone, -1>::Map(Ecd.data(), DimPerBone, numBones);
	//Ej = Ecjm.colwise().sum();

	// Fx3J, Use to caculate Varience of Joints position
	MatrixX Ex(frameCount, numBones * 3);
	vector<int> bParents(numBones);
	for (auto block : *pBlockArmature)
	{
		int bpid = 0;
		if (block->parent())
			bpid = block->parent()->Joints.back()->ID();

		for (auto joint : block->Joints)
		{
			bParents[joint->ID()] = bpid;
		}
	}

	for (Eigen::DenseIndex i = 0; i < frameCount; i++)
	{
		for (Eigen::DenseIndex j = 0; j < numBones; j++)
		{
			Vector3 v = frames[i][j].GblTranslation - frames[i][bParents[j]].GblTranslation;
			Ex.block<1,3>(i,j*3) = Eigen::Vector3f::Map(&v.x);
		}
	}
	Ex.rowwise() -= Ex.colwise().mean();
	RowVectorX E3j = Ex.cwiseAbs2().colwise().mean();
	// 3xJ, Translation Energy(Variance) among 3 axis
	Ej3 = Eigen::MatrixXf::Map(E3j.data(), 3, numBones);
	Ej = Ej3.colwise().sum();
	Ejrot = Ej.replicate(3, 1) - Ej3;

	const auto& blocks = *pBlockArmature;
	Eb.resize(blocks.size());
	Eb3.resize(3, blocks.size());
	Ebrot.resize(3,blocks.size());
	Xbs.resize(blocks.size());

	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;

		Eigen::MatrixXf Yb(X.rows(), block->Joints.size() * CharacterFeature::Dimension);
		for (size_t j = 0; j < joints.size(); j++)
		{
			auto jid = joints[j]->ID();
			Yb.middleCols<DimPerBone>(j * DimPerBone) = X.middleCols<DimPerBone>(jid * DimPerBone);
			if (Eb(i) < Ej(jid))
			{
				Eb(i) = Ej(jid);
				Ebrot(i) =Ejrot(jid);
				Eb3.col(i) = Ej3.col(jid);
			}
		}

		Xbs[i] = Yb;
	}

	Eb = Eb.cwiseSqrt();

	// Why do another sqrt ? it's too aggresive
	// Eb = Eb.cwiseSqrt();

	//Eb /= Eb.maxCoeff();
	//float maxCoeff = Eb.maxCoeff();

	//for (size_t i = 0; i < Eb.size(); i++)
	//{
	//	if (Eb(i) > EnergyCutoff * maxCoeff)
	//	{
	//		ActiveBlocks.push_back(i);
	//	}
	//}
}

void AnimationAnalyzer::ComputePcaQr()
{
	static const auto DimPerBone = CharacterFeature::Dimension;
	auto numBones = pBlockArmature->Armature().size();

	const auto& blocks = *pBlockArmature;
	auto bSize = blocks.size();

	Qrs.resize(bSize);
	Pcas.resize(bSize);
	Xbs.resize(bSize);
	PvPcas.resize(bSize);
	PvQrs.resize(bSize);

	concurrency::parallel_for_each(blocks.begin(), blocks.end(), [this](auto block)
		//for (auto& block : blocks)
	{
		//std::cout << "Pca start" << endl;
		auto i = block->Index;
		auto& joints = block->Joints;

		auto &Yb = Xbs[i];
		auto & pca = Pcas[i];
		pca.compute(Yb, true);
		auto d = pca.reducedRank(PcaCutoff);
		Qrs[i].compute(pca.coordinates(d), true);

		auto pvs = Eigen::Matrix<float, CLIP_FRAME_COUNT, 3, Eigen::RowMajor>::Map(Pvs.col(i).data());
		PvPcas[i].compute(pvs);
		PvQrs[i].compute(PvPcas[i].coordinates(3),true);
		//std::cout << "Pca finish" << endl;
	});
}

void AnimationAnalyzer::ComputeSpatialTraits(const std::vector<AffineFrame> &frames)
{
	int gblRefId = (*pBlockArmature)[0]->Joints.back()->ID();
	auto frameCount = frames.size();
	const auto& blocks = *pBlockArmature;
	Sp.setZero(6, blocks.size());
	Pvs.setZero(frameCount * 3, blocks.size());
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;
		auto lastJid = joints.back()->ID();

		auto vtc = Sp.block<3, 1>(0, block->Index);
		int fid = 0;
		for (const auto& frame : frames)
		{
			auto v = Pvs.block<3, 1>(fid * 3, i) = Eigen::Vector3f::MapAligned(&frame[lastJid].GblTranslation.x);;
			vtc += v;
			++fid;
		}
		vtc /= frameCount;

		if (i != gblRefId)
			vtc -= Sp.block<3, 1>(0, gblRefId);

		Sp.block<3, 1>(3, block->Index) = vtc;

		if (block->parent() != nullptr)
		{
			auto pi = block->parent()->Index;
			vtc -= Sp.block<3, 1>(3, pi);
		}
	}

	for (auto& block : boost::make_iterator_range(blocks.rbegin(),blocks.rend()))
	{
		auto i = block->Index;
		if (block->parent() != nullptr)
		{
			auto pi = block->parent()->Index;
			for (size_t fid = 0; fid < frameCount; fid++)
			{
				auto v = Pvs.block<3, 1>(fid * 3, i) -= Pvs.block<3, 1>(fid * 3, pi);
				// v.normalize();
			}

		}
	}

	for (size_t i = 0; i < Sp.cols(); i++)
	{
		Sp.block<3, 1>(0, i).normalize();
		Sp.block<3, 1>(3, i).normalize();
	}
}
