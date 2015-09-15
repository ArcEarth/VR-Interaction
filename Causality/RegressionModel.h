#pragma once
#include <Eigen\Core>
#include <memory>

namespace Causality
{
	class IRegression abstract
	{
		// return the confidence of the model
		virtual float Fit(const Eigen::MatrixXf& X, const Eigen::MatrixXf& Y) = 0;

		// return the confidence of the prediction
		virtual float Predict(const Eigen::RowVectorXf& X, const Eigen::RowVectorXf& Y) const = 0;
	};

	class PcaCcaRegression : public IRegression
	{
	public:
		// Inherited via IRegression
		virtual float Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y) override;
		virtual float Predict(const Eigen::RowVectorXf & X, const Eigen::RowVectorXf & Y) const override;

	private:
		class Impl;
		std::unique_ptr<Impl> m_pImpl;
	};

	class GaussianProcessRegression : public IRegression
	{
	public:
		// Inherited via IRegression
		virtual float Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y) override;
		virtual float Predict(const Eigen::RowVectorXf & X, const Eigen::RowVectorXf & Y) const override;

	private:
		class Impl;
		std::unique_ptr<Impl> m_pImpl;
	};
}