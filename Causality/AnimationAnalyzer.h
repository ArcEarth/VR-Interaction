#pragma once
#include <atomic>
#include <Eigen\Dense>
#include "EigenExtension.h"
#include "ArmatureBlock.h"
#include "BoneFeatures.h"
#include "Animations.h"
#include <ppltasks.h>

namespace std
{
	class thread;
}

namespace Causality
{
	class AnimationAnalyzer
	{
	public:
		typedef CharacterFeature FeatureType;

		explicit AnimationAnalyzer(const BlockArmature& pBArm);
		~AnimationAnalyzer();

		concurrency::task<void> ComputeFromFramesAsync(const std::vector<AffineFrame> &frames);
		void ComputeFromFrames(const std::vector<AffineFrame> &frames);
		void ComputeFromBlocklizedMat(const Eigen::MatrixXf& mat);
		void BlocklizationAndComputeEnergy();
		void ComputePcaQr();
		void ComputeSpatialTraits(const std::vector<AffineFrame> &frames);

		std::atomic_bool				IsReady;
		std::unique_ptr<std::thread>	pComputingThread;

		const BlockArmature*			pBlockArmature;

		std::vector<int>	ActiveBlocks; // it's a set
		Eigen::RowVectorXf	Weithts;
		Eigen::Array<Eigen::Cca<float>,-1,-1> SlefCorrs;

		float				EnergyCutoff;
		float				PcaCutoff;
		Eigen::MatrixXf		X;  // Fx sum(d_Bi), data matrix, d = Block dimension, F = Frame count, J = Block count
		Eigen::RowVectorXf  Ej;	// 1xJ, Jointwise Energy
		Eigen::RowVectorXf	Eb;	// 1xB, Blockwise Energy
		Eigen::MatrixXf		Sp;	// 6xB, Spatial traits, B = block count
		Eigen::MatrixXf		Dirs; // 3FxB, block end-effector displacements, F = frame count, B = block count

		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> Qrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		Pcas;		 // Blockwise Pca
		std::vector<Eigen::MatrixXf>	Xbs;					 // Blockwise Data
		std::vector<Eigen::PcaCcaMap>	PerceptiveVectorReconstructor;
		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> PvQrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		PvPcas;		 // Blockwise Pca

		DirectX::BoundingBox			BoundingBox;
	};
}
