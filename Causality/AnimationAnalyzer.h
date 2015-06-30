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
		Eigen::MatrixXf		X;  // 20N x F data matrix
		Eigen::RowVectorXf  Ej;	// Jointwise Energy
		Eigen::RowVectorXf	Eb;	// Blockwise Energy
		Eigen::Matrix<float, -1, -1> Sp;	// Spatial traits
		Eigen::MatrixXf		Dirs;

		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> Qrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>> Pcas;		 // Blockwise Pca
		std::vector<Eigen::MatrixXf> Xbs;					 // Blockwise Data

		DirectX::BoundingBox			BoundingBox;
		// Extra storage
		int								BestPhi;
		std::vector<Eigen::DenseIndex>	BestMatching;
		float							BestMatchingScore;
	};
}
