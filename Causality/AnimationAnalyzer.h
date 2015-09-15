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
	class CcaArmatureTransform;

	class ClipInfo
	{
	public:
		typedef CharacterFeature FeatureType;

		explicit ClipInfo(const BlockArmature& pBArm);
		~ClipInfo();

		concurrency::task<void> ComputeFromFramesAsync(const std::vector<AffineFrame> &frames);
		void ComputeFromFrames(const std::vector<AffineFrame> &frames);
		void ComputeFromBlocklizedMat(const Eigen::MatrixXf& mat);
		void BlocklizationAndComputeEnergy(const std::vector<AffineFrame>& frames);
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
		Eigen::RowVectorXi	DimX;	//Apprarixed Dimension of X
		Eigen::RowVectorXf  Ej;	// 1xJ, Jointwise Translation (Varience) Energy
		Eigen::RowVectorXf	Epj; // 1xJ, Jointwise Potiential Energy
		Eigen::MatrixXf		Ej3;// 3xJ, Jointwise Translation (Varience) Energy among axis
		Eigen::RowVectorXf	Eb;	// 1xB, Blockwise Translation Energy
		Eigen::RowVectorXf	Epb; // 1xB, Blockwise Potiential Energy
		Eigen::MatrixXf		Eb3;// 3xB, Blockwise Translation Energy among axis
		Eigen::MatrixXf		Ejrot; // 3xJ, Jointwise Rotation Energy 
		Eigen::MatrixXf		Ebrot; // 3xB, Blockwise Rotation Energy 
		Eigen::MatrixXf		Sp;	// 6xB, Spatial traits, B = block count
		Eigen::MatrixXf		Pvs; // 3FxB, block end-effector displacements, F = frame count, B = block count

		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> Qrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		Pcas;		 // Blockwise Pca
		std::vector<Eigen::MatrixXf>	Xbs;					 // Blockwise Data
		std::vector<PcaCcaMap>	PerceptiveVectorReconstructor;
		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> PvQrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		PvPcas;		 // Blockwise Pca

		// Information for current 'map'
		int												Phi;
		std::vector<Eigen::DenseIndex>					Matching;
		float											Score;
		Eigen::MatrixXf									Ra; // Overall correlation
		Eigen::MatrixXf									Rk; // Kinetic correlation
		Eigen::MatrixXf									Rs; // Positional correlation

		std::unique_ptr<ArmatureTransform>				pLocalBinding;

		DirectX::BoundingBox							BoundingBox;
	};
}
