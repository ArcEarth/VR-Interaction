#pragma once
#include <Eigen\Dense>
#include <Eigen\fft>

namespace Eigen {

	template <class InputDerived>
	inline Matrix<InputDerived::Scalar, Dynamic, InputDerived::ColsAtCompileTime> resample(const MatrixBase<InputDerived> &X, DenseIndex p, DenseIndex q)
	{
		assert(!X.IsRowMajor);
		auto n = X.rows();
		auto m = n * q / p;
		auto c = std::max(m, n);
		
		Matrix<std::complex<InputDerived::Scalar>, X.RowsAtCompileTime, X.ColsAtCompileTime> Xf(c,X.cols());
		output.resize(m, X.cols());

		FFT<float> fft;
		for (size_t i = 0; i < X.cols(); i++)
		{
			fft.fwd(Xf.col(i), X.col(i));
			fft.inv(Xr.col(i), Xf.col(i).topRows(m));
		}
		return Xr;
	}

}