#include "pch_bcl.h"
#include "AnimationAnalyzer.h"
#include <algorithm>
#include <ppl.h>
#include "CCA.h"
#include "Settings.h"
#include "EigenExtension.h"
#include <fftw3.h>
#include <unsupported\Eigen\fft>

using namespace Causality;
using namespace std;
using namespace Concurrency;
using namespace Eigen;

ClipInfo::ClipInfo(const ShrinkedArmature & bArm)
	:pBlockArmature(&bArm), PcaCutoff(0.01f), IsReady(false), ActiveEnergyThreshold(0.35f), PerceptiveVectorReconstructor(pBlockArmature->size())
{
}

ClipInfo::~ClipInfo()
{
}

void Causality::ClipInfo::ProcessFrames(gsl::array_view<BoneHiracheryFrame> frames)
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

		auto pvs = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Map(Pvs.col(i).data(), CLIP_FRAME_COUNT, 3);
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

CyclicStreamClipinfo::CyclicStreamClipinfo(ShrinkedArmature* pArmature)
	: InputClipInfo(*pArmature)
{

	Initalize();
}

void Causality::CyclicStreamClipinfo::Initalize()
{
	m_windowSize = CaculateWindowSize();

	m_frameWidth = pBlockArmature->size() * InputFeature::Dimension;
	m_bufferWidth = ((m_frameWidth + 1) / 4) * 4;
	m_bufferCapacity = m_windowSize * 4;
	m_buffer.setZero(m_bufferWidth, m_bufferCapacity);
	m_Spectrum.setZero(m_bufferWidth, m_windowSize);
	m_SmoothedBuffer.resize(m_frameWidth, m_windowSize);


	m_cropMargin = m_sampleRate * 0.3; // 0.3s margin

	int n = m_windowSize;
	fftwf_plan plan = fftwf_plan_many_dft_r2c(1, &n, m_frameWidth,
		m_buffer.data(), nullptr, m_bufferWidth, 1,
		(fftwf_complex*)m_Spectrum.data(), nullptr, m_bufferWidth, 1,
		0);
}

void Causality::CyclicStreamClipinfo::StreamFrame(const FrameType & frame)
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
			std::lock_guard<std::mutex> guard(m_mutex);
			// move the buffer to leftmost
			m_buffer.leftCols(m_bufferSize) = m_buffer.middleCols(m_bufferHead, m_bufferSize);
			m_bufferHead = 0;
		}
	}

	// the last column
	auto fv = m_buffer.col(m_bufferHead + m_bufferSize - 1);
	const auto dim = InputFeature::Dimension;

	auto& parts = *pBlockArmature;
	for (int i = 0; i < parts.size(); i++)
	{
		auto part = parts[i];
		auto bv = m_pFeature->Get(*part, frame);
		assert(bv.size() == dim);
		fv.segment(i * dim, dim) = bv;
	}

	++m_frameCounter;
	if (m_enableCyclicDtc && m_frameCounter >= m_analyzeInterval && m_bufferSize >= m_windowSize)
	{
		m_frameCounter = 0;
		AnaylzeRecentStream();
	}
}

void Causality::CyclicStreamClipinfo::AnaylzeRecentStream()
{
	// Anaylze starting
	size_t head = m_bufferHead;
	size_t windowSize = m_windowSize;

	CaculateSpecturum(head, windowSize);

	auto fr = CaculatePeekFrequency(m_Spectrum);

	if (fr.Support > m_cyclicDtcThr)
	{
		IsReady = false;
		CropResampleInput(head, windowSize, CLIP_FRAME_COUNT, 0.8f);
		CaculatePartsMatricFromX(CLIP_FRAME_COUNT);
	}
}

void Causality::CyclicStreamClipinfo::CaculateSpecturum(size_t head, size_t windowSize)
{
	m_readerHead = head;
	m_readerSize = windowSize;

	std::lock_guard<std::mutex> guard(m_mutex);

	int n = windowSize;
	fftwf_plan plan = fftwf_plan_many_dft_r2c(1, &n, m_frameWidth,
		m_buffer.col(head).data(), nullptr, m_bufferWidth, 1,
		(fftwf_complex*)m_Spectrum.data(), nullptr, m_bufferWidth, 1,
		0);

	// keep the plan alive will help the plan speed
	if (m_fftplan)
		fftwf_destroy_plan((fftwf_plan)m_fftplan);

	m_fftplan = plan;

	fftwf_execute(plan);
}

void Causality::CyclicStreamClipinfo::CropResampleInput(size_t head, size_t inputPeriod, size_t framePerCycle, float smoothStrength)
{
	const int SmoothIteration = 4;
	auto T = inputPeriod;

	assert((T + m_cropMargin) * 2 < m_bufferSize);
	// copy the buffer
	auto& Xs = m_SmoothedBuffer.topRows((T + m_cropMargin) * 2);

	{
		// Critial section, copy data from buffer and transpose in Column Major
		std::lock_guard<std::mutex> guard(m_mutex);
		Xs = m_buffer.block(head, 0, (T + m_cropMargin) * 2, m_frameWidth).transpose();
	}

	//! Smooth the input 
	laplacian_smooth(Xs, smoothStrength, SmoothIteration);

	cublic_bezier_resample(X,
		m_SmoothedBuffer.middleRows(m_cropMargin, T * 2),
		CLIP_FRAME_COUNT * 2,
		Eigen::CloseLoop);
}

CyclicStreamClipinfo::FrequencyResolveResult CyclicStreamClipinfo::CaculatePeekFrequency(const Eigen::MatrixXcf & spectrum)
{
	FrequencyResolveResult fr;

	auto& Xf = spectrum;
	m_Spmod = Xf;
	auto& Xs = m_Spmod;

	int idx;
	auto& Ea = m_SpectrumEnergy;

	// Note Xf is (bufferWidth X windowSize)
	// thus we crop it top frameWidth rows and intersted band in cols to caculate energy
	Ea = Xf.block(0, m_minHz - 1, m_frameWidth, m_HzWidth + 2).cwiseAbs2().colwise().sum().transpose();

	cout << Ea.transpose() << endl;

	Ea.segment(1, Ea.size() - 2).maxCoeff(&idx); // Frequency 3 - 30
	++idx;

	auto Ex = Ea.middleRows<3>(idx - 1);
	idx += m_minHz;

	cout << Ex.transpose() << endl;

	Vector3f Ix = { idx - 1.0f, (float)idx, idx + 1.0f };
	float peekFreq = Ex.dot(Ix) / Ex.sum();

	int T = (int)ceil(Xf.rows() / peekFreq);

	float snr = Ex.sum() / Ea.segment(1, Ea.size() - 2).sum();

	fr.Frequency = peekFreq;
	fr.PeriodInFrame = T;
	fr.Support = snr;
}

void Causality::InputClipInfo::CaculatePartsMatricFromX(size_t T)
{
	auto& parts = *pBlockArmature;

	const int fDim = 3;

	auto m_uX = X.colwise().mean().eval();
	m_cX = X - m_uX.replicate(X.rows(), 1).eval();

	Eb3.resize(fDim, parts.size());
	RowVectorXf::Map(Eb3.data(), Eb3.size()) = m_cX.cwiseAbs2().colwise().sum();
	Eb = Eb3.colwise().sum();

	Eb /= Eb.maxCoeff();

	Pcas.resize(parts.size());
	for (int i = 0; i < parts.size(); i++)
	{
		if (Eb[i] > ActiveEnergyThreshold)
		{
			ActiveParts.push_back(i);
		}
		else if (Eb[i] > SubactiveEnergyThreshold)
		{
			SubactiveParts.push_back(i);
		}

		// Compute Pca for all active and sub-active parts
		// inactive parts
		if (Eb[i] > SubactiveEnergyThreshold)
		{
			auto& pca = Pcas[i];
			pca.computeCentered(m_cX.middleCols(i*fDim, fDim), true);
			pca.setMean(m_uX.segment(i*fDim, fDim));
			auto d = pca.reducedRank(PcaCutoff);

			Qrs[i].compute(Pcas[i].coordinates(d),true);
		}
	}

	// assert(X is continuous stored)
	Pvs.resizeLike(X);
	MatrixXf::Map(Pvs.data(), fDim, Pvs.size() / fDim) = 
		MatrixXf::Map(X.data(), fDim, X.size() / fDim).colwise().normalized();

}
