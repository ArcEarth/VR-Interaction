#include "pch_bcl.h"
#include "GestrualTracker.h"
#include <numeric>
#include <random>


using namespace Causality;

std::random_device g_rd;

IGestureTracker::~IGestureTracker()
{
}

ParticalFilterBase::~ParticalFilterBase()
{
}

void ParticalFilterBase::Step(const InputVectorType & input)
{
	SetInputState(input);
	StepParticals();
}

const ParticalFilterBase::TrackingVectorType & ParticalFilterBase::CurrentState() const
{
	return m_CurrentSample;
}

void ParticalFilterBase::StepParticals()
{
	Resample(m_NewSample, m_CurrentSample);
	m_NewSample.swap(m_CurrentSample);

	auto& sample = m_CurrentSample;
	auto n = sample.rows();
	auto dim = sample.cols() - 1;

	for (int i = 0; i < n; i++)
	{
		auto partical = sample.block<1, -1>(i, 1, 1, dim);

		Progate(partical);
		sample(i, 0) = Likilihood(partical);
	}

	m_CurrentSampleMean = (sample.rightCols(dim).array() * sample.col(0).replicate(1, dim).array()).rowwise().sum();
}

// resample the weighted sample in O(n*log(n)) time
// generate n ordered point in range [0,1] is n log(n), thus we cannot get any better
void ParticalFilterBase::Resample(Eigen::MatrixXf & resampled, const Eigen::MatrixXf & sample)
{
	std::mt19937 mt(g_rd());
	assert((resampled.data() != sample.data()) && "resampled and sample cannot be the same");

	auto n = sample.rows();
	auto dim = sample.cols() - 1;
	resampled.resizeLike(sample);

	auto cdf = resampled.col(0);
	std::partial_sum(sample.col(0).data(), sample.col(0).data() + n, cdf.data());

	for (int i = 0; i < n; i++)
	{
		// get x from range [0,1] randomly
		float x = (float)(mt() - std::mt19937::min()) / (float)(std::mt19937::max() - std::mt19937::min());

		auto itr = std::lower_bound(cdf.data(), cdf.data() + n, x);
		auto idx = itr - cdf.data();

		resampled.block<1,-1>(i,1,1, dim) = sample.block<1, -1>(i, 1, 1, dim);
	}

	cdf.array() = 1 / (float)n;
}

void CharacterParticalFilter::Reset(const InputVectorType & input)
{
}

void CharacterParticalFilter::SetInputState(const InputVectorType & input)
{
	m_CurrentInput = input;
}

float CharacterParticalFilter::Likilihood(const TrackingVectorBlockType & x)
{
	float t = x[0];
	float s = x[1];
	float dt = x[2];
	float ds = x[3];
}

void CharacterParticalFilter::Progate(TrackingVectorBlockType & x)
{
	float t = x[0];
	float s = x[1];
	float dt = x[2];
	float ds = x[3];
}
