#pragma once
#include <Eigen\Core>
#include "Animations.h"
#include "ArmatureParts.h"

namespace Causality
{
	class IGestureTracker
	{
	public:
	public:
		typedef float ScalarType;
		typedef Eigen::Matrix<ScalarType, -1, -1> MatrixType;
		typedef Eigen::Matrix<ScalarType, 1, -1, Eigen::AutoAlign | Eigen::RowMajor> TrackingVectorType;
		typedef Eigen::Block<MatrixType, 1, -1> TrackingVectorBlockType;

		typedef Eigen::Matrix<ScalarType, 1, -1> InputVectorType;

		virtual ~IGestureTracker();

	public:
		// aka, Initialize, discard all history information, just initialize with input, reset all state
		virtual void Reset(const InputVectorType& input) = 0;
		// Step forward the tracking state from t to t+1
		virtual void Step(const InputVectorType& input) = 0;
		// Get the tracking state
		virtual const TrackingVectorType& CurrentState() const = 0;
	};

	// Provide base methods for Particle Filter
	// 
	// The sample matrix S is N x (Dim+1)
	// Where N is number of Particals
	// Dim is the state vector dimension
	// S.col(0), the first column of S stores the weights
	// S.row(i) is a particale
	// S(i, 0) is the particale weight
	// S(i, 1...Dim) is the state vector
	class ParticaleFilterBase : public IGestureTracker
	{
	public:
		~ParticaleFilterBase();

		void Step(const InputVectorType& input) override;

		const TrackingVectorType& CurrentState() const override;

		//virtual void Reset(const InputVectorType& input) = 0;

	
	protected: // Interfaces
		virtual void SetInputState(const InputVectorType& input) = 0;
		// Get the likilihood of partical state x in current time with pre-seted input state
		virtual float Likilihood(const TrackingVectorBlockType &x) = 0;

		virtual void Progate(TrackingVectorBlockType& x) = 0;

	protected:
		void StepParticals();

		void Resample(_Out_ Eigen::MatrixXf& resampled, _In_ const Eigen::MatrixXf& sample);

		MatrixType m_sample;
		MatrixType m_newSample;
		// Mean state 
		TrackingVectorType m_state;
	};
}