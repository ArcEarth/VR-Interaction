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

	// A ClipFacade is a anayls base on certian given feature
	class ClipFacade
	{
	public:
		enum ComputationFlag
		{
			NotInitialize = 0,
			ComputeBasic = 1,
			ComputeNormalize = 0x2,
			ComputePca = 0x4,
			ComputePcaQr = 0x8 | ComputePca,
			ComputePairDif = 0x10,
			ComputeAll = ComputePcaQr | ComputeNormalize | ComputePairDif,
		};

		enum PairDifLevelEnum
		{
			NonePair = 0,
			ActivePartPairs = 1,
			SubactivePartPairs = 2,
			AllPartPairs = 3,
		};

		// Construction & Meta-data acess
	public:
		ClipFacade();
		~ClipFacade();

		const ShrinkedArmature&			ArmatureParts() const { return *m_pParts; }
		auto&							ActiveParts() const { return m_ActiveParts; }
		auto&							SubactiveParts() const { return m_SubactiveParts; }

		const auto&						ClipName() const { return m_clipName; }
		void							SetClipName(const std::string& name) { m_clipName = name; }
		int								ClipFrames() const { return m_X.rows(); }
		// Clip time length in seconds
		double							ClipTime() const { return m_clipTime; }
		void							SetClipTime(double time_seconds) { m_clipTime = time_seconds; }

		bool							IsReady() const { return m_inited; }
		// return -1 if variable feature dimension
		bool							GetAllPartDimension() const { return m_pdFix ? m_dimP : -1; }
		PairDifLevelEnum				GetPartPairInfoLevel() const { return m_pairInfoLevl; }

		float							PcaCutoff() const { return m_pcaCutoff; }
		void							SetPcaCutoff(float cutoff) { m_pcaCutoff = cutoff; }

		const IArmaturePartFeature*		GetFeature() const { return m_pFeature.get(); }
		void							SetFeature(std::unique_ptr<IArmaturePartFeature> && pFeature)
		{
			m_pFeature = std::move(pFeature);
		}

		template <typename _Ty, class... _Types>
		void							SetFeature(_Types&&... _Args)
		{
			m_pFeature.reset( new _Ty( _STD forward<_Types>(_Args)... ));
		}

		void							Prepare(const ShrinkedArmature& parts, int clipLength = -1, int flag = ComputeAll);

		void							AnalyzeSequence(gsl::array_view<BoneHiracheryFrame> frames, double sequenceTime);

		void							SetFeatureMatrix(gsl::array_view<BoneHiracheryFrame> frames);
		void							SetFeatureMatrix(const Eigen::MatrixXf& X) { m_inited = false; m_X = X; }
		void							SetFeatureMatrix(Eigen::MatrixXf&& X) { m_inited = false; m_X = std::move(X); }
		Eigen::MatrixXf&				SetFeatureMatrix() { m_inited = false; return m_X; }

		// Caculate energy and Pca and Qr for parts
		void							CaculatePartsMetric();

	protected:
		void							CaculatePartsPairMetric(PairDifLevelEnum level = ActivePartPairs);

		// Part-wise metric accessers
	public:
		Eigen::DenseIndex				GetPartStartIndex(int pid) const
		{
			return m_pdFix ? pid * m_dimP : m_partSt[pid];
		}

		Eigen::DenseIndex				GetPartDimension(int pid) const
		{
			return m_pdFix ? m_dimP : m_partDim[pid];
		}

		float							GetPartEnergy(int pid) const { return m_Eb[pid]; }
		auto&							GetAllPartsEnergy() const
		{
			return m_Eb;
		}
		auto&							GetPartDimEnergy(int pid) const
		{
			return m_Edim[pid];
		}

		const Eigen::MatrixXf&			GetAllPartsSequence() const
		{
			return m_X;
		}
		auto							GetPartSequence(int pid) const
		{
			return m_X.middleCols(GetPartStartIndex(pid), GetPartDimension(pid));
		}

		auto&							GetAllPartsNormalizedSequence() const
		{
			return m_Xnor;
		}
		auto							GetPartNormalizedSequence(int pid) const
		{
			return m_Xnor.middleCols(GetPartStartIndex(pid), GetPartDimension(pid));
		}

		auto							GetPartMean(int pid) const
		{
			return m_uX.segment(GetPartStartIndex(pid), GetPartDimension(pid));
		}
		auto&							GetAllPartsMean() const
		{
			return m_uX;
		}

		// Only Active and Subactive part have Pca / Qr
		auto&							GetPartPca(int pid) const
		{
			return m_Pcas[pid];
		}
		auto							GetPartPcadSequence(int pid, int d = -1) const
		{
			if (d < 0)
				d = m_PcaDims[pid];
			return m_Pcas[pid].coordinates(d);
		}
		// QrView ofpPart Pcad Sequence
		auto							GetPartPcadQrView(int pid, int stFrame = 0, int frames = -1) const
		{
			return Eigen::QrView<Eigen::MatrixXf>(m_thickQrs[pid], stFrame, frames);
		}

		auto							GetPartsDifferenceSequence(int pi, int pj) const
		{
			assert(m_partDim[pi] == m_partDim[pj]);
			return GetPartSequence(pi) - GetPartSequence(pj);
		}

		// return the row vector of E(X(pi)-X(pj))
		auto							GetPartsDifferenceMean(int pi, int pj) const
		{
			return m_difMean.block(pi*m_dimP, pj, m_dimP, 1).transpose();
		}
		auto							GetPartsDifferenceCovarience(int pi, int pj) const
		{
			return m_difMean.block(pi*m_dimP, pj*m_dimP, m_dimP, m_dimP);
		}

	protected:
		std::string				m_clipName;

		const ShrinkedArmature*	m_pParts;

		std::unique_ptr<IArmaturePartFeature>
								m_pFeature;

		bool					m_pdFix;		// Is Part feature fixed size
		std::vector<int>		m_partSt;		// Part feature start index for part i
		std::vector<int>		m_partDim;		// Part feature dimension for part i
		int						m_dimP;			// Part feature dimension

		double					m_clipTime;
		unsigned				m_flag;
		PairDifLevelEnum		m_pairInfoLevl;
		bool					m_inited;

		// Feature data matrix
		Eigen::MatrixXf			m_X;	// Raw data
		Eigen::RowVectorXf		m_uX;	// X mean
		Eigen::MatrixXf			m_cX;	// Centered X

		Eigen::MatrixXf			m_Xnor;	// Partiwise Rowwise noramlized X

		Eigen::RowVectorXf		m_Eb;	// 1xB, Blockwise Energy
		std::vector<Eigen::VectorXf>	
								m_Edim;// Dimension energy for parts i , for variable size

		Eigen::VectorXi			m_PcaDims;		
		std::vector<Eigen::Pca<Eigen::MatrixXf>>
								m_Pcas;			// Pca of the raw data, partiwise

		std::vector<Eigen::QrStore<Eigen::MatrixXf>>
								m_thickQrs;		// Qr of the Pca of the raw data, partiwise

		float					m_pcaCutoff;

		std::vector<int>		m_ActiveParts;  // it's a set
		std::vector<int>		m_SubactiveParts;

		float					m_ActiveEnergyThreshold;
		float					m_SubactiveEnergyThreshold;

		Eigen::MatrixXf			m_difMean;
		Eigen::MatrixXf			m_difCov;
	};

	typedef
		ArmaturePartFeatures::RelativeDeformation <
		ArmaturePartFeatures::AllJoints <
		BoneFeatures::LclRotLnQuatFeature > >
		CharacterJRSFeature;

	typedef
		//ArmaturePartFeatures::WithVelocity<
		ArmaturePartFeatures::Localize<
		ArmaturePartFeatures::EndEffector<
		BoneFeatures::GblPosFeature>>
		PVSFeature;

	class CharacterClipinfo
	{
	public:
		ClipFacade RcFacade;
		ClipFacade PvFacade;

		CharacterClipinfo()
		{
			m_isReady = false;
			m_pParts = nullptr;
		}

		void Initialize(const ShrinkedArmature& parts)
		{
			m_pParts = &parts;
			auto pIF = std::make_unique<CharacterJRSFeature>();
			pIF->SetDefaultFrame(parts.Armature().default_frame());

			RcFacade.SetFeature(std::move(pIF));
			PvFacade.SetFeature<PVSFeature>();

			RcFacade.Prepare(parts, -1, ClipFacade::ComputePcaQr);
			PvFacade.Prepare(parts, -1, ClipFacade::ComputeAll);
		}

		void AnalyzeSequence(gsl::array_view<BoneHiracheryFrame> frames, double sequenceTime)
		{
			RcFacade.AnalyzeSequence(frames, sequenceTime);
			PvFacade.AnalyzeSequence(frames, sequenceTime);
		}

		explicit CharacterClipinfo(const ShrinkedArmature& parts)
		{
			Initialize(parts);
		}

		const ShrinkedArmature&			ArmatureParts() const { return *m_pParts; }
		auto&							ActiveParts() const { return	PvFacade.ActiveParts(); }
		auto&							SubactiveParts() const { return PvFacade.SubactiveParts(); }

		const std::string&				ClipName() const { return m_clipName; }
		void							SetClipName(const ::std::string& name)
		{
			m_clipName = name;
			PvFacade.SetClipName(name);
			RcFacade.SetClipName(name);
		}

		int								ClipFrames() const { return PvFacade.ClipFrames(); }
		// Clip time length in seconds
		double							ClipTime() const { return PvFacade.ClipTime(); }
	protected:
		std::string				m_clipName;
		const ShrinkedArmature*	m_pParts;
		bool					m_isReady;
	};

	class ClipInfo
	{
	public:
		typedef CharacterFeature FeatureType;

		ClipInfo();
		explicit ClipInfo(const ShrinkedArmature& pBArm);
		ClipInfo(const ClipInfo& rhs) = default;
		ClipInfo(ClipInfo&& rhs) = default;
		~ClipInfo();

		bool HasInputFeature() const;
		bool HasOutputFeature() const;

		void ProcessFrames(gsl::array_view<BoneHiracheryFrame> frames);
		void ComputeFromFrames(const std::vector<BoneHiracheryFrame> &frames);
		void ComputeFromBlocklizedMat(const Eigen::MatrixXf& mat);
		void BlocklizationAndComputeEnergy(const std::vector<BoneHiracheryFrame>& frames);
		void ComputePcaQr();
		void ComputeSpatialTraits(const std::vector<BoneHiracheryFrame> &frames);

		std::string						ClipName;
		bool							IsReady;
		//std::unique_ptr<std::thread>	pComputingThread;

		const ShrinkedArmature*			pBlockArmature;
		const ShrinkedArmature&			ArmatureParts() const { return *pBlockArmature; }

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
		Eigen::MatrixXf		uPvs;// 3xB, part wise pv mean
		Eigen::MatrixXf		PvNormals; // Normalized Perceptive vectors
		std::vector<Eigen::VectorXf>	MaxBs;
		std::vector<Eigen::VectorXf>	MinBs;

		std::vector<Eigen::QrStore<Eigen::MatrixXf>> Qrs;  // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		Pcas; // Blockwise Pca
		std::vector<Eigen::MatrixXf>					Xbs;  // Blockwise Data
		std::vector<PcaCcaMap>	PerceptiveVectorReconstructor;
		std::vector<Eigen::QrStore<Eigen::MatrixXf>> PvQrs; // Blockwise Qr Decomposition 
		std::vector<Eigen::Pca<Eigen::MatrixXf>>		PvPcas;// Blockwise Pca

		Eigen::Array<Eigen::RowVector3f, Eigen::Dynamic, Eigen::Dynamic>	PvDifMean;
		Eigen::Array<Eigen::Matrix3f, Eigen::Dynamic, Eigen::Dynamic>		PvDifCov;

		float						GetPartEnergy() const;
		auto						GetActivePartsDifferenceSequence(int pi, int pj) const;
		const Eigen::RowVector3f&	GetActivePartsDifferenceAverage(int pi, int pj) const;
		const Eigen::Matrix3f&		GetActivePartsDifferenceCovarience(int pi, int pj) const;

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

		auto&							GetAllPartsPvMean() const
		{
			return uPvs;
		}
		auto							GetPartPvSequence(int pid) const
		{
			return Pvs.middleCols(pid * 3, 3);
		}
		auto&							GetAllPartsNormalizedPvSequence() const
		{
			return Pvs;
		}
		auto							GetPartNormalizedPvSequence(int pid) const
		{
			return Pvs.middleCols(pid * 3, 3);
		}

		auto&							GetPartPvPca(int pid) const
		{
			return Pcas[pid];
		}
		auto&							GetPartPvPcaQr(int pid) const
		{
			return Qrs[pid];
		}

	protected:
		Eigen::MatrixXf			m_cX; // Centered X
		Eigen::RowVectorXf		m_uX; // X mean
		std::vector<Eigen::QrStore<Eigen::MatrixXf>>
			m_thickQrs;
	};

	class InputClipInfo : public ClipInfo
	{
	public:

		using ClipInfo::ClipInfo;

		int	Period; // T
		int TemproalSampleInterval; // Ti
		vector<vector<Eigen::QrStore<Eigen::MatrixXf>>> qrXs;

		const Eigen::QrStore<Eigen::MatrixXf>& QrXs(int tid, int pid) const { return qrXs[tid][pid]; }

		auto RawXs(int tid, int pid) const
		{
			return Xbs[pid].middleRows(tid * TemproalSampleInterval, Period);
		}

		auto	GetPartPvSequence(int pid) const
		{
			return X.middleCols(pid * 3, 3);
		}
		auto	GetPartPvSequence(int pid, int stFrame, int frames) const
		{
			return X.block(stFrame,pid * 3,frames, 3);
		}
		auto	GetPartNormalizedPvSequence(int pid, int stFrame, int frames) const
		{
			return Pvs.block(stFrame, pid * 3, frames, 3);
		}

		// New interfaces
		auto	GetPartPvPcaQrView(int pid, int stFrame, int frames) const
		{
			auto& qr = m_thickQrs[pid];
			return Eigen::QrView<Eigen::MatrixXf>(qr, stFrame, frames);
		}
	};

	class CyclicStreamClipinfo : protected ClipFacade
	{
	public:
		typedef PVSFeature PartsFeatureType;
		typedef IArmatureStreamAnimation::frame_type FrameType;

		void InitializePvFacade(ShrinkedArmature& parts);

		// set interval_frames == 0 to automaticly estimate based on windows size and sample rate
		CyclicStreamClipinfo(ShrinkedArmature& parts, time_seconds minT, time_seconds maxT, double sampleRateHz, size_t interval_frames = 0);

		void InitializeStreamView(ShrinkedArmature& parts, time_seconds minT, time_seconds maxT, double sampleRateHz, size_t interval_frames);

		// Important, input frame use this method
		void StreamFrame(const FrameType& frame);

		void AnaylzeRecentStream();

		// Result stored in this->m_Spectrum
		void CaculateSpecturum(size_t head, size_t windowSize);

		// Crop and resample the interst segment in the input stream
		// Returns 2TxD Matrix stores input features partwise, T == framePerCycle
		// Result is stored in this->X
		void CropResampleInput(_Out_ Eigen::MatrixXf& X, size_t head, size_t inputPeriod, size_t framePerCycle, float smoothStrength);

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

		std::atomic_bool	
					m_enableCyclicDtc;
		int			m_frameCounter;
		float		m_cyclicDtcThr; // The threshold to classify as Cyclic motion

		// thread sychronization
		//boost::icl::interval_set<int> m_bufferAccess;
		std::mutex	m_bfMutex;	// buffer access mutex
		int			m_readerHead;
		int			m_readerSize;

		// Feature buffer
		//! Column majored feature matrix, 1 column = 1 frame in time
		Eigen::MatrixXf		m_buffer;

		std::mutex			m_spMutex;
		//! Column major spectrum, 1 column = 1 frame in time
		Eigen::MatrixXcf	m_Spectrum;	// wSize x fSize
		Eigen::VectorXf		m_SpectrumEnergy;//In each frequency
		Eigen::MatrixXf		m_SmoothedBuffer;

		int					m_bufferHead;
		int					m_bufferSize;
		int					m_bufferWidth;	// actual floats per frame in buffer, considered with alignments
		int					m_bufferCapacity;

		void*				m_fftplan;		// impleamentation detail, handle to fft

		std::mutex			m_facadeMutex;	// Facade access mutex
	};
}
