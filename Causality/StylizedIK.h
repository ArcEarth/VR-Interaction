#pragma once
#include "Armature.h"
#include "Animations.h"
#include "RegressionModel.h"
#include "GaussianProcess.h"
#include "PcaCcaMap.h"

namespace Causality
{
	class StylizedChainIK : public IRegression
	{
		// translation of local chain, 128 bit aligned
		std::vector<DirectX::Vector4, DirectX::XMAllocator>	m_chain;
		float												m_chainLength;
		gaussian_process_regression							m_gplvm;

		double												m_ikWeight;
		double												m_ikLimitWeight;
		double												m_markovWeight;
		double												m_styleWeight;

		double												m_currentError;

		double												m_meanLk;
		long												m_counter;
		bool												m_cValiad;
		Eigen::RowVectorXd									m_iy;// initial y, default y
		Eigen::RowVectorXd									m_cx;
		Eigen::RowVectorXd									m_cy;
		Eigen::RowVectorXd									m_wy;//weights of y
		Eigen::MatrixXd										m_limy;//weights of y

		Eigen::RowVectorXd									m_ey;
		double												m_segmaX;

		Eigen::Vector3d										m_goal;
		Quaternion											m_baseRot;

	public:
		StylizedChainIK();
		StylizedChainIK(const std::vector<const Joint*> &joints, const BoneHiracheryFrame& defaultframe);

		void SetChain(const std::vector<const Joint*> &joints, const BoneHiracheryFrame& defaultframe);
		void SetGoal(const Eigen::Vector3d& goal);
		void SetIKWeight(double weight);
		void SetMarkovWeight(double weight);
		void SetBaseRotation(const Quaternion& q);
		void SetYWeights(const Eigen::RowVectorXd &wy);
		void SetHint(const Eigen::RowVectorXd &y);
		template <class Derived>
		void SetGplvmWeight(const Eigen::DenseBase<Derived>& w) { m_wy = w; }
		template <class Derived>
		void SetYLimit(const Eigen::DenseBase<Derived>& limY) { m_limy = limY; }

		gaussian_process_regression& Gplvm() { return m_gplvm; }
		const gaussian_process_regression& Gplvm() const { return m_gplvm; }

		DirectX::XMVECTOR EndPosition(const DirectX::XMFLOAT4A* rotqs);
		Eigen::Matrix3Xf EndPositionJacobi(const DirectX::XMFLOAT4A* rotqs);

		void JacobbiFromR(DirectX::XMFLOAT4X4A &jac, _In_reads_(3) const float* r);

		double objective(const Eigen::RowVectorXd &x, const Eigen::RowVectorXd &y);

		Eigen::RowVectorXd objective_derv(const Eigen::RowVectorXd & x, const Eigen::RowVectorXd & y);

		// Filter alike interface
		// reset history data
		void Reset();
		void Reset(const Eigen::RowVectorXd &x, const Eigen::RowVectorXd &y);

		// return the joints rotation vector
		const Eigen::RowVectorXd& Apply(const Eigen::Vector3d& goal);
		const Eigen::RowVectorXd& Apply(const Eigen::Vector3d& goal, const Eigen::Vector3d& goal_velocity);
		const Eigen::RowVectorXd& Apply(const Eigen::Vector3d& goal, const Eigen::VectorXd& hint_y);

		double CurrentError() const { return m_currentError; }
		// Inherited via IRegression
		virtual float Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y) override;
		virtual float Predict(const Eigen::RowVectorXf & X, Eigen::RowVectorXf & Y) override;
	};

#define AUTO_PROPERTY(type,name) private : type m_##name;\
public:\
	type& name() { return m_##name; } \
	const type& name() { return m_##name; } \
	void set_##name(const type& val) { m_##name = val;}



	class PcaCcaIK : public IRegression
	{
	public:
		// Inherited via IRegression
		virtual float Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y) override;
		virtual float Predict(const Eigen::RowVectorXf & X, Eigen::RowVectorXf & Y) override;

		PcaCcaMap& Cca() { return m_cca; }
		const PcaCcaMap& Cca() const { return m_cca; }


	private:
		float		m_xCut, m_yCut;
		PcaCcaMap	m_cca;
	};

	class GaussianProcessIK : public IRegression
	{
	public:
		// Inherited via IRegression
		virtual float Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y) override;
		virtual float Predict(const Eigen::RowVectorXf & X, Eigen::RowVectorXf & Y) override;

		gaussian_process_regression& Gpr() { return m_gpr; }
		const gaussian_process_regression& Gpr() const { return m_gpr; }

		Eigen::MatrixXd& ObsrCov() { return m_obsrCov; }
		const Eigen::MatrixXd& ObsrCov() const { return m_obsrCov; }

	private:
		Eigen::MatrixXd				m_obsrCov;
		gaussian_process_regression m_gpr;
	};
}