#pragma once
#include <atomic>
#include <Eigen\Dense>
#include "ArmatureBlock.h"
#include "BoneFeatures.h"
#include "Animations.h"
//#include <ppltasks.h>
#include <mutex>
#include <gsl.h>
//#include <boost\icl\interval_set.hpp>

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

		explicit ClipInfo(const ShrinkedArmature& pBArm);
		~ClipInfo();

		void ProcessFrames(gsl::array_view<BoneHiracheryFrame> frames);
		void ComputeFromFrames(const std::vector<BoneHiracheryFrame> &frames);
		void ComputeFromBlocklizedMat(const Eigen::MatrixXf& mat);
		void BlocklizationAndComputeEnergy(const std::vector<BoneHiracheryFrame>& frames);
		void ComputePcaQr();
		void ComputeSpatialTraits(const std::vector<BoneHiracheryFrame> &frames);

		std::string						ClipName;
		std::atomic_bool				IsReady;
		//std::unique_ptr<std::thread>	pComputingThread;

		const ShrinkedArmature*			pBlockArmature;

		std::vector<int>	ActiveParts; // it's a set
		std::vector<int>	SubactiveParts;

		Eigen::RowVectorXf	Weithts;
		Eigen::Array<Eigen::Cca<float>,-1,-1> SlefCorrs;

		float				ActiveEnergyThreshold;
		float				SubactiveEnergyThreshold;

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
		std::vector<Eigen::VectorXf>	MaxBs;
		std::vector<Eigen::VectorXf>	MinBs;

		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> Qrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		Pcas;// Blockwise Pca
		std::vector<Eigen::MatrixXf>	Xbs;				 // Blockwise Data
		std::vector<PcaCcaMap>	PerceptiveVectorReconstructor;
		std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> PvQrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		PvPcas;// Blockwise Pca

		Eigen::Array<Eigen::RowVector3f, Eigen::Dynamic, Eigen::Dynamic>	PvDifMean;
		Eigen::Array<Eigen::Matrix3f, Eigen::Dynamic, Eigen::Dynamic>		PvDifCov;

		float						GetPartEnergy() const;
		auto						GetActivePartsDifferenceSequence(int pi, int pj);
		const Eigen::RowVector3f&	GetActivePartsDifferenceAverage(int pi, int pj);
		const Eigen::Matrix3f&		GetActivePartsDifferenceCovarience(int pi, int pj);

		const Eigen::RowVector3f&	XpvMean(int i, int j) const { return PvDifMean(i, j); }
		const Eigen::Matrix3f&		XpvCov(int i, int j) const { return PvDifCov(i, j); }
		Eigen::RowVector3f&			XpvMean(int i, int j) { return PvDifMean(i, j); }
		Eigen::Matrix3f&			XpvCov(int i, int j) { return PvDifCov(i, j); }

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

	class InputClipInfo : public ClipInfo
	{
	public:
		using ClipInfo::ClipInfo;

		int	Period; // T
		int TemproalSampleInterval; // Ti
		vector<vector<Eigen::MeanThinQr<Eigen::MatrixXf>>> qrXs;

		const Eigen::MeanThinQr<Eigen::MatrixXf>& QrXs(int tid, int pid) const { return qrXs[tid][pid]; }

		auto RawXs(int tid, int pid) const
		{
			return Xbs[pid].middleRows(tid * TemproalSampleInterval, Period);
		}


		// New interfaces
		auto							GetPartSequence(int pid) const
		{
			return X.middleCols(pid * 3, 3);
		}
		const Eigen::RowVectorXf&		GetPartMean(int pid) const
		{
			return Pcas[pid].mean();
		}
		auto&							GetPartPca(int pid) const
		{
			return Pcas[pid];
		}
		auto							GetPartPcaedSequence(int pid, int d) const
		{
			return Pcas[pid].coordinates(d);
		}
		auto							GetPartDirectionSequence(int pid) const
		{
			return Pvs.middleCols(pid * 3, 3);
		}
		Eigen::QrView<Eigen::MatrixXf>	GetPartQrView(int pid, int stFrame, int frames) const
		{
			auto& qr = m_thickQrs[pid];
			return Eigen::QrView<Eigen::MatrixXf>(qr.Qr, qr.Q, qr.Mean, stFrame, frames);
		}

		// process the clipinfo by using input from X
		// T : the actual Period
		void CaculatePartsMatricFromX(size_t T);

	protected:
		Eigen::MatrixXf			m_cX; // Centered X
		Eigen::RowVectorXf		m_uX; // X mean
		std::vector<Eigen::MeanThickQr<Eigen::MatrixXf>>			
								m_thickQrs;
	};

	class CyclicStreamClipinfo : public InputClipInfo
	{
	public:
		typedef IArmatureStreamAnimation::frame_type FrameType;

		CyclicStreamClipinfo(ShrinkedArmature* pArmature);

		// set interval_frames == 0 to automaticly estimate based on windows size and sample rate
		CyclicStreamClipinfo(ShrinkedArmature* pArmature, time_seconds minT, time_seconds maxT, double sampleRateHz, size_t interval_frames = 0);

		void SetExpectedPeriod(time_seconds minT, time_seconds maxT);
		void SetSampleRate(double sampleRateHz);
		void SetCyclicDetectionInterval(size_t interval_frames);

		void Initalize();

		// Important, input frame use this method
		void StreamFrame(const FrameType& frame);

		void AnaylzeRecentStream();

		// Result stored in this->m_Spectrum
		void CaculateSpecturum(size_t head, size_t windowSize);

		// Crop and resample the interst segment in the input stream
		// Returns 2TxD Matrix stores input features partwise, T == framePerCycle
		// Result is stored in this->X
		void CropResampleInput(size_t head, size_t inputPeriod, size_t framePerCycle, float smoothStrength);

		struct FrequencyResolveResult
		{
			float Frequency;
			float Support;
			int	  PeriodInFrame;
		};

		FrequencyResolveResult CaculatePeekFrequency(const Eigen::MatrixXcf& spectrum);

		void ComputeClipinfo(size_t cliplength);

		void EnableCyclicMotionDetection(bool is_enable = true) { m_enableCyclicDtc = is_enable; }

	private:
		double		m_minT, m_maxT;
		double		m_sampleRate;
		int			m_cropMargin;

		int			m_minHz, m_maxHz, m_HzWidth;
		int			m_frameWidth; // logical feature frame width, in floats
		int			m_windowSize; // expected windows size

		int			m_analyzeInterval; // analyze should invoke for every $m_analyzeInterval$ frames arrived

		std::unique_ptr<IArmaturePartFeature>
					m_pFeature;
		int			m_featureDim;

		atomic_bool	m_enableCyclicDtc;
		int			m_frameCounter;
		float		m_cyclicDtcThr; // The threshold to classify as Cyclic motion

		// thread sychronization
		//boost::icl::interval_set<int> m_bufferAccess;
		std::mutex	m_mutex;
		int			m_readerHead;
		int			m_readerSize;

		// Feature buffer
		// This buffer stores the time-re-sampled frame data as feature matrix
		// This buffer is allocated as 30x30 frames size
		// thus , it would be linearized every 30 seconds
		Eigen::MatrixXf		m_buffer;

		Eigen::MatrixXcf	m_Spectrum;
		Eigen::MatrixXf		m_Spmod;// Length of Spectrum
		Eigen::VectorXf		m_SpectrumEnergy;//In each frequency
		Eigen::MatrixXf		m_SmoothedBuffer;

		int					m_bufferHead;
		int					m_bufferSize;
		int					m_bufferWidth;	//actual floats per frame in buffer, considered with alignments
		int					m_bufferCapacity;

		void*				m_fftplan;		// impleamentation detail, handle to fft

		int					CaculateWindowSize();
	};
}
