#pragma once
#include <Eigen\Dense>
#include "EigenExtension.h"
#include "ArmatureBlock.h"
#include "BoneFeatures.h"
#include "Animations.h"

namespace Causality
{
	class AnimationAnalyzer
	{
	public:
		typedef CharacterFeature FeatureType;

		AnimationAnalyzer(BlockArmature* pBArm);
		~AnimationAnalyzer();

		void ComputeFromFrames(const std::vector<AffineFrame> &frames);
		void ComputeFromBlocklizedMat(const Eigen::MatrixXf& mat);
		void BlocklizationAndComputeEnergy();
		void ComputePcaQr();
		void ComputeSpatialTraits(const std::vector<AffineFrame> &frames);

		BlockArmature*		pBlockArmature;

		float				PcaCutoff;
		Eigen::MatrixXf		X;  // 20N x F data matrix
		Eigen::RowVectorXf  Ej;	// Jointwise Energy
		Eigen::RowVectorXf	Eb;	// Blockwise Energy
		Eigen::Matrix<float, 3, -1> Sp;	// Spatial traits
		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> Qrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>> Pcas;		 // Blockwise Pca
		std::vector<Eigen::MatrixXf> Xbs;					 // Blockwise Data
	};
}
