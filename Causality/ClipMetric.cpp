#include "pch_bcl.h"
#include "ClipMetric.h"
#include <algorithm>
#include <ppl.h>
#include "CCA.h"
#include "Settings.h"
#include "EigenExtension.h"
#include <unsupported\Eigen\fft>

#define FFTW

#ifdef FFTW
#include <fftw3.h>
#pragma comment(lib, "libfftw3f-3.lib")
#endif

#ifdef _DEBUG
#define DEBUGOUT(x) std::cout << #x << " = " << x << std::endl
#else
#define DEBUGOUT(x)
#endif

using namespace Causality;
using namespace std;
using namespace Concurrency;
using namespace Eigen;

ClipInfo::ClipInfo()
:pBlockArmature(nullptr), PcaCutoff(0.01f), IsReady(false), ActiveEnergyThreshold(0.35f)
{
}

ClipInfo::ClipInfo(const ShrinkedArmature & bArm)
	: ClipInfo()
{
	pBlockArmature = &bArm;
	PerceptiveVectorReconstructor.resize(pBlockArmature->size());
}

ClipInfo::~ClipInfo()
{
}

void ClipInfo::ProcessFrames(gsl::array_view<BoneHiracheryFrame> frames)
{
}

void ClipInfo::ComputeFromFrames(const std::vector<BoneHiracheryFrame>& frames)
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

void ClipInfo::ComputeFromBlocklizedMat(const Eigen::MatrixXf & mat)
{
	X = mat;
	ComputePcaQr();
}

void ClipInfo::BlocklizationAndComputeEnergy(const std::vector<BoneHiracheryFrame>& frames)
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
	MatrixXf Ex(frameCount, numBones * 3);
	vector<int> bParents(numBones);
	for (auto block : *pBlockArmature)
	{
		int bpid = 0;
		if (block->parent())
			bpid = block->parent()->Joints.back()->ID;

		for (auto joint : block->Joints)
		{
			bParents[joint->ID] = bpid;
		}
	}

	for (Eigen::DenseIndex i = 0; i < frameCount; i++)
	{
		for (Eigen::DenseIndex j = 0; j < numBones; j++)
		{
			Vector3 v = frames[i][j].GblTranslation - frames[i][bParents[j]].GblTranslation;
			Ex.block<1, 3>(i, j * 3) = Eigen::Vector3f::Map(&v.x);
		}
	}
	Ex.rowwise() -= Ex.colwise().mean();
	RowVectorXf E3j = Ex.cwiseAbs2().colwise().mean();
	// 3xJ, Translation Energy(Variance) among 3 axis
	Ej3 = Eigen::MatrixXf::Map(E3j.data(), 3, numBones);
	Ej = Ej3.colwise().sum();
	Ejrot = Ej.replicate(3, 1) - Ej3;

	const auto& blocks = *pBlockArmature;
	Eb.setZero(blocks.size());
	Eb3.setZero(3, blocks.size());
	Ebrot.setZero(3, blocks.size());
	Xbs.resize(blocks.size());
	DimX.resize(blocks.size());

	Eigen::Pca<Eigen::MatrixXf> pca;
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;

		Eigen::MatrixXf Yb(X.rows(), block->Joints.size() * CharacterFeature::Dimension);
		for (size_t j = 0; j < joints.size(); j++)
		{
			auto jid = joints[j]->ID;
			Yb.middleCols<DimPerBone>(j * DimPerBone) = X.middleCols<DimPerBone>(jid * DimPerBone);
			if (Eb(i) < Ej(jid))
			{
				Eb(i) = Ej(jid);
				Ebrot(i) = Ejrot(jid);
				Eb3.col(i) = Ej3.col(jid);
			}
		}

		Xbs[i] = Yb;

		pca.compute(Yb);
		DimX[i] = pca.reducedRank(0.01f);
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
	//		ActiveParts.push_back(i);
	//	}
	//}
}

void ClipInfo::ComputePcaQr()
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

		auto pvs = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>::Map(Pvs.col(i).data(), CLIP_FRAME_COUNT, 3);
		PvPcas[i].compute(pvs);
		PvQrs[i].compute(PvPcas[i].coordinates(3), true);

		//std::cout << "Pca finish" << endl;
	});
}

void ClipInfo::ComputeSpatialTraits(const std::vector<BoneHiracheryFrame> &frames)
{
	int gblRefId = (*pBlockArmature)[0]->Joints.back()->ID;
	auto frameCount = frames.size();
	const auto& blocks = *pBlockArmature;
	Sp.setZero(6, blocks.size());
	Pvs.setZero(frameCount * 3, blocks.size());
	for (auto& block : blocks)
	{
		auto i = block->Index;
		auto& joints = block->Joints;
		auto lastJid = joints.back()->ID;

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

	for (auto& block : boost::make_iterator_range(blocks.rbegin(), blocks.rend()))
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

const Eigen::RowVector3f & Causality::ClipInfo::GetActivePartsDifferenceAverage(int pi, int pj) const
{
	return PvDifMean(pi, pj);
}

const Eigen::Matrix3f & Causality::ClipInfo::GetActivePartsDifferenceCovarience(int pi, int pj) const
{
	return PvDifCov(pi, pj);
}

void CyclicStreamClipinfo::InitializePvFacade(ShrinkedArmature& parts)
{
	ClipFacade::SetFeature<PartsFeatureType>();
	ClipFacade::Prepare(parts, CLIP_FRAME_COUNT * 2, ComputePcaQr | ComputeNormalize | ComputePairDif);
}

CyclicStreamClipinfo::CyclicStreamClipinfo(ShrinkedArmature& parts, time_seconds minT, time_seconds maxT, double sampleRateHz, size_t interval_frames)
{
	InitializeStreamView(parts, minT, maxT,sampleRateHz, interval_frames);

	InitializePvFacade(parts);
}

void CyclicStreamClipinfo::InitializeStreamView(ShrinkedArmature& parts, time_seconds minT, time_seconds maxT, double sampleRateHz, size_t interval_frames)
{
	m_pParts = &parts;
	m_pFeature.reset(new PartsFeatureType());

	m_minHz = 1 / maxT.count();
	m_maxHz = 1 / minT.count();
	m_sampleRate = sampleRateHz;

	// find closest 2^k window size
	m_windowSize = 1 << static_cast<int>(ceil(log2(maxT.count() * sampleRateHz * 4)));

	m_analyzeInterval = interval_frames;
	// if automatic, we set 1/16 of period as analyze interval
	if (m_analyzeInterval == 0)
		m_analyzeInterval = std::max(m_windowSize / 16, 1);

	m_frameWidth = 0;
	for (int i = 0; i < parts.size(); i++)
		m_frameWidth += m_pFeature->GetDimension(*parts[i]);

	// make sure each row/column in buffer is aligned as __mm128 for SIMD
	const auto alignBoundry = alignof(__m128) / sizeof(float);
	m_bufferWidth = ((m_frameWidth + 1) / alignBoundry) * alignBoundry;

	// this 4 is just some majical constant that large enough to avaiod frequent data moving
	m_bufferCapacity = m_windowSize * 4;

	m_buffer.setZero(m_bufferWidth, m_bufferCapacity);
	m_Spectrum.setZero(m_bufferWidth, m_windowSize);
	m_SmoothedBuffer.resize(m_frameWidth, m_windowSize);

	m_cropMargin = m_sampleRate * 0.1; // 0.1s of frames as margin
	m_cyclicDtcThr = 0.75f;

	int n = m_windowSize;

#ifdef FFTW
	fftwf_plan plan = fftwf_plan_many_dft_r2c(1, &n, m_frameWidth,
		m_buffer.data(), nullptr, m_bufferWidth, 1,
		(fftwf_complex*)m_Spectrum.data(), nullptr, m_bufferWidth, 1,
		0);
#endif
}

void CyclicStreamClipinfo::StreamFrame(const FrameType & frame)
{
	using namespace Eigen;
	using namespace DirectX;

	if (m_bufferSize < m_windowSize)
		++m_bufferSize;
	else if (m_bufferSize > m_windowSize)
	{
		m_bufferHead += m_bufferSize - m_windowSize;
		m_bufferSize -= m_windowSize;

		if (m_bufferHead + m_bufferSize >= m_bufferCapacity)
		{
			// unique_lock
			std::lock_guard<std::mutex> guard(m_bfMutex);
			// move the buffer to leftmost
			m_buffer.leftCols(m_bufferSize) = m_buffer.middleCols(m_bufferHead, m_bufferSize);
			m_bufferHead = 0;
		}
	}

	// the last column
	auto fv = m_buffer.col(m_bufferHead + m_bufferSize - 1);

	auto& parts = *m_pParts;
	int stIdx = 0;
	for (int i = 0; i < parts.size(); i++)
	{
		auto& part = *parts[i];
		auto bv = m_pFeature->Get(part, frame);
		int dim = m_pFeature->GetDimension(*parts[i]);
		fv.segment(stIdx, dim) = bv.transpose();
		stIdx += dim;
	}

	++m_frameCounter;
	if (m_enableCyclicDtc && m_frameCounter >= m_analyzeInterval && m_bufferSize >= m_windowSize)
	{
		m_frameCounter = 0;
		AnaylzeRecentStream();
	}
}

struct scope_unlock
{
	std::mutex& _mutex;

	scope_unlock(std::mutex& mutex)
		: _mutex(mutex)
	{
	}

	~scope_unlock()
	{
		_mutex.unlock();
	}
};

void CyclicStreamClipinfo::AnaylzeRecentStream()
{
	// Anaylze starting
	size_t head = m_bufferHead;
	size_t windowSize = m_windowSize;

	CaculateSpecturum(head, windowSize);

	auto fr = CaculatePeekFrequency(m_Spectrum);

	if (fr.Support > m_cyclicDtcThr && m_facadeMutex.try_lock())
	{
		{
			scope_unlock guard(m_facadeMutex);

			auto& X = ClipFacade::SetFeatureMatrix();
			float Tseconds = 1 / fr.Frequency;
			CropResampleInput(X, head, fr.PeriodInFrame, CLIP_FRAME_COUNT, 0.8f);

			ClipFacade::SetClipTime(Tseconds);
			ClipFacade::CaculatePartsMetric();
		}
	}
}

void CyclicStreamClipinfo::CaculateSpecturum(size_t head, size_t windowSize)
{
	m_readerHead = head;
	m_readerSize = windowSize;

	std::lock_guard<std::mutex> guard(m_bfMutex);

	int n = windowSize;
#ifdef FFTW
	fftwf_plan plan = fftwf_plan_many_dft_r2c(1, &n, m_frameWidth,
		m_buffer.col(head).data(), nullptr, m_bufferWidth, 1,
		(fftwf_complex*)m_Spectrum.data(), nullptr, m_bufferWidth, 1,
		0);

	// keep the plan alive will help the plan speed
	if (m_fftplan)
		fftwf_destroy_plan((fftwf_plan)m_fftplan);

	m_fftplan = plan;

	fftwf_execute(plan);
#endif
}

void CyclicStreamClipinfo::CropResampleInput(_Out_ MatrixXf& X, size_t head, size_t inputPeriod, size_t resampledPeriod, float smoothStrength)
{
	const int smoothIteration = 4;
	auto T = inputPeriod;

	int inputLength = T + m_cropMargin * 2;
	assert(inputLength< m_bufferSize);
	// copy the buffer
	auto Xs = m_SmoothedBuffer.topRows(inputLength);

	{
		// Critial section, copy data from buffer and transpose in Column Major
		std::lock_guard<std::mutex> guard(m_bfMutex);
		Xs = m_buffer.block(head, 0, inputLength, m_frameWidth).transpose();
	}

	// Smooth the input 
	laplacianSmooth(Xs, smoothStrength, smoothIteration, Eigen::CloseLoop);

	//! To-do , use better method to crop out the "example" single period

	// Resample input into X
	cublicBezierResample(X,
		m_SmoothedBuffer.middleRows(m_cropMargin, T),
		resampledPeriod,
		Eigen::CloseLoop);
}

CyclicStreamClipinfo::FrequencyResolveResult CyclicStreamClipinfo::CaculatePeekFrequency(const Eigen::MatrixXcf & spectrum)
{
	FrequencyResolveResult fr;

	// Column major spectrum, 1 column = 1 frame in time
	auto& Xf = spectrum;
	auto windowSize = Xf.cols();

	int idx;
	auto& Ea = m_SpectrumEnergy;

	// Note Xf is (bufferWidth X windowSize)
	// thus we crop it top frameWidth rows and intersted band in cols to caculate energy
	Ea = Xf.block(0, m_minHz - 1, m_frameWidth, m_HzWidth + 2).cwiseAbs2().colwise().sum().transpose();

	DEBUGOUT(Ea.transpose());

	Ea.segment(1, Ea.size() - 2).maxCoeff(&idx); // Frequency 3 - 30
	++idx;

	// get the 2 adjicant freequency as well, to perform interpolation to get better estimation
	auto Ex = Ea.segment<3>(idx - 1);
	idx += m_minHz;

	DEBUGOUT(Ex.transpose());

	Vector3f Ix = { idx - 1.0f, (float)idx, idx + 1.0f };
	float peekFreq = Ex.dot(Ix) / Ex.sum();

	int T = (int)ceil(windowSize / peekFreq);

	float snr = Ex.sum() / Ea.segment(1, Ea.size() - 2).sum();

	fr.Frequency = windowSize / peekFreq / m_sampleRate;
	fr.PeriodInFrame = T;
	fr.Support = snr;

	return fr;
}

ClipFacade::ClipFacade()
{
	m_pParts = nullptr;
	m_flag = NotInitialize;
	m_pairInfoLevl = NonePair;
	m_dimP = -1;
	m_pdFix = false;
	m_inited = false;
	m_pcaCutoff = g_CharacterPcaCutoff;
	m_ActiveEnergyThreshold = g_CharacterActiveEnergy;
	m_SubactiveEnergyThreshold = g_CharacterSubactiveEnergy;
}

ClipFacade::~ClipFacade()
{

}

void Causality::ClipFacade::Prepare(const ShrinkedArmature & parts, int clipLength, int flag)
{
	assert(m_pFeature != nullptr && "Set Feature Before Call Prepare");

	m_pParts = &parts;
	m_flag = flag;

	m_Edim.resize(parts.size());
	m_Eb.resize(parts.size());
	m_partDim.resize(parts.size());
	m_partSt.resize(parts.size());

	m_ActiveParts.reserve(parts.size());
	m_SubactiveParts.reserve(parts.size());

	if (m_flag & ComputePca)
	{
		m_Pcas.resize(parts.size());
		m_PcaDims.resize(parts.size());

		if (m_flag & ComputePcaQr)
			m_thickQrs.resize(parts.size());
	}

	m_partSt[0] = 0;
	m_pdFix = true;
	for (int i = 0; i < parts.size(); i++)
	{
		m_partDim[i] = m_pFeature->GetDimension(*parts[i]);
		if (i > 0)
		{
			m_pdFix = m_pdFix && (m_partDim[i] == m_partDim[i - 1]);
			m_partSt[i] = m_partSt[i - 1] + m_partDim[i - 1];
		}

		m_Edim[i].resize(m_partDim[i]);
	}
	m_dimP = m_pdFix ? m_partDim[0] : -1;

	int fLength = m_partDim.back() + m_partSt.back();

	if (clipLength > 0)
	{
		m_X.resize(clipLength, fLength);
		m_uX.resize(fLength);
		m_cX.resizeLike(m_X);
		if (m_flag & ComputeNormalize)
			m_Xnor.resizeLike(m_X);
	}

	if (!m_pdFix)
		m_flag &= ~ComputePairDif;

	if (m_flag & ComputePairDif)
	{
		m_difMean.setZero(parts.size() * m_dimP, parts.size());
		m_difMean.setZero(parts.size() * m_dimP, parts.size() * m_dimP);
	}
}

void ClipFacade::AnalyzeSequence(gsl::array_view<BoneHiracheryFrame> frames, double sequenceTime)
{
	assert(m_pParts != nullptr);

	m_clipTime = sequenceTime;

	SetFeatureMatrix(frames);

	CaculatePartsMetric();
}

void Causality::ClipFacade::SetFeatureMatrix(gsl::array_view<BoneHiracheryFrame> frames)
{
	assert(m_pParts != nullptr && m_flag != NotInitialize);

	m_inited = false;

	auto& parts = *m_pParts;
	int fLength = m_partDim.back() + m_partSt.back();

	m_X.resize(frames.size(), fLength);
	for (int f = 0; f < frames.size(); f++)
	{
		auto& frame = frames[f];
		for (int i = 0; i < parts.size(); i++)
		{
			auto part = parts[i];

			auto fv = m_X.block(f, m_partSt[i], 1, m_partDim[i]);

			fv = m_pFeature->Get(*part, frame);
		}
	}
}

void ClipFacade::CaculatePartsMetric()
{
	auto& parts = *m_pParts;

	m_uX = m_X.colwise().mean().eval();
	m_cX = m_X - m_uX.replicate(m_X.rows(), 1).eval();
	if (m_flag & ComputeNormalize)
		m_Xnor = m_X;

	m_Edim.resize(parts.size());
	m_Eb.resize(parts.size());
	m_partDim.resize(parts.size());
	m_partSt.resize(parts.size());

	m_Pcas.resize(parts.size());
	m_thickQrs.resize(parts.size());

	for (int i = 0; i < parts.size(); i++)
	{

		m_Edim[i] = m_cX.middleCols(m_partSt[i], m_partDim[i]).cwiseAbs2().colwise().sum().transpose();
		m_Eb[i] = m_Edim[i].sum();

		if (m_flag & ComputeNormalize)
			m_Xnor.middleCols(m_partSt[i], m_partDim[i]).rowwise().normalize();
	}

	m_Eb /= m_Eb.maxCoeff();

	for (int i = 0; i < parts.size(); i++)
	{
		if (m_Eb[i] > m_ActiveEnergyThreshold)
		{
			m_ActiveParts.push_back(i);
		}
		else if (m_Eb[i] > m_SubactiveEnergyThreshold)
		{
			m_SubactiveParts.push_back(i);
		}

		// Compute Pca for all active and sub-active parts
		// inactive parts
		if ((m_flag & ComputePca) && m_Eb[i] > m_SubactiveEnergyThreshold)
		{
			auto& pca = m_Pcas[i];
			pca.computeCentered(m_cX.middleCols(m_partSt[i], m_partDim[i]), true);
			pca.setMean(m_uX.segment(m_partSt[i], m_partDim[i]));
			auto d = pca.reducedRank(m_pcaCutoff);
			m_PcaDims[i] = d;

			//! Potiential unnessary matrix copy here!!!
			if (m_flag & ComputePcaQr)
				m_thickQrs[i].compute(m_Pcas[i].coordinates(d), false, true);
		}
	}

	if (m_flag & ComputePairDif)
		CaculatePartsPairMetric();

	m_inited = true;
}

void ClipFacade::CaculatePartsPairMetric(PairDifLevelEnum level)
{
	m_pairInfoLevl = level;

	auto& parts = *m_pParts;

	m_difMean.resize(parts.size() * m_dimP, parts.size());
	m_difMean.resize(parts.size() * m_dimP, parts.size() * m_dimP);

	float thrh = 0;
	switch (level)
	{
	case Causality::ClipFacade::ActivePartPairs:
		thrh = m_ActiveEnergyThreshold;
		break;
	case Causality::ClipFacade::SubactivePartPairs:
		thrh = m_SubactiveEnergyThreshold;
		break;
	case Causality::ClipFacade::AllPartPairs:
		thrh = 0;
		break;
	case Causality::ClipFacade::NonePair:
	default:
		return;
	}

	MatrixXf Xij(ClipFrames(), m_dimP);
	RowVectorXf uXij(m_dimP);

	for (int i = 0; i < parts.size(); i++)
	{
		if (m_Eb[i] < thrh) continue;
		for (int j = i+1; j < parts.size(); j++)
		{
			if (m_Eb[j] < thrh) continue;

			Xij = GetPartsDifferenceSequence(i, j);
			uXij = Xij.colwise().mean();
			m_difMean.block(i*m_dimP, j, m_dimP, 1) = uXij.transpose();

			auto covij = m_difCov.block(i*m_dimP, j*m_dimP, m_dimP, m_dimP);

			Xij.rowwise() -= uXij;
			covij = Xij.transpose() * Xij;

			// mean is aniti-symetric, covarience is symetric
			m_difMean.block(j*m_dimP, i, m_dimP, 1) = -uXij.transpose();
			m_difCov.block(j*m_dimP, i*m_dimP, m_dimP, m_dimP) = covij;
		}
	}
}