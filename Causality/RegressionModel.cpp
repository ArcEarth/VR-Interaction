#include "pch_bcl.h"
#include "RegressionModel.h"

float Causality::PcaCcaRegression::Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y)
{
	return 0.0f;
}

float Causality::PcaCcaRegression::Predict(const Eigen::RowVectorXf & X, const Eigen::RowVectorXf & Y) const
{
	return 0.0f;
}


float Causality::GaussianProcessRegression::Fit(const Eigen::MatrixXf & X, const Eigen::MatrixXf & Y)
{
	return 0.0f;
}

float Causality::GaussianProcessRegression::Predict(const Eigen::RowVectorXf & X, const Eigen::RowVectorXf & Y) const
{
	return 0.0f;
}
