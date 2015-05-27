#pragma once
#include <Eigen\Dense>

namespace Eigen {
	//template<typename _MatrixType> class CCA
	//{
	//public:

	//	/** \brief Synonym for the template parameter \p _MatrixType. */
	//	typedef _MatrixType MatrixType;

	//	enum {
	//		RowsAtCompileTime = MatrixType::RowsAtCompileTime,
	//		ColsAtCompileTime = MatrixType::ColsAtCompileTime,
	//		Options = MatrixType::Options,
	//		MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
	//		MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
	//	};

	//	/** \brief Scalar type for matrices of type #MatrixType. */
	//	typedef typename MatrixType::Scalar Scalar;
	//	typedef typename NumTraits<Scalar>::Real RealScalar;
	//	typedef typename MatrixType::Index Index;

	//	/** \brief Complex scalar type for #MatrixType.
	//	*
	//	* This is \c std::complex<Scalar> if #Scalar is real (e.g.,
	//	* \c float or \c double) and just \c Scalar if #Scalar is
	//	* complex.
	//	*/
	//	typedef std::complex<RealScalar> ComplexScalar;

	//	/** \brief Type for vector of eigenvalues as returned by eigenvalues().
	//	*
	//	* This is a column vector with entries of type #ComplexScalar.
	//	* The length of the vector is the size of #MatrixType.
	//	*/
	//	typedef Matrix<ComplexScalar, ColsAtCompileTime, 1, Options & ~RowMajor, MaxColsAtCompileTime, 1> EigenvalueType;

	//	/** \brief Type for matrix of eigenvectors as returned by eigenvectors().
	//	*
	//	* This is a square matrix with entries of type #ComplexScalar.
	//	* The size is the same as the size of #MatrixType.
	//	*/
	//	typedef Matrix<ComplexScalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime> EigenvectorsType;

	//	CCA()
	//	{

	//	}
	//};

	// Canonical correlation analysis
	// X : n x dX input vectors
	// Y : n x dY input vectors
	// n : number of observations
	// dX : dimension of feature X
	// dY : dimension of feature Y
	// return : correlation coefficients R, transform matrix A,B that maps X and Y to Latent space
	// thus argmax(A,B) corr(XA,YB)
	// it's best to make sure X Y are full-rank in the sense of colume rank
	class Cca
	{
	private:
		bool m_initialized;
		MatrixXf	A, B;	 // transform matrix to latent space
		VectorXf	R;		 // Correlations
		RowVectorXf uX, uY;	 // mean vector of X and Y
		DenseIndex	rankX, rankY;	// rank of input X and Y
		DenseIndex  d, dX, dY;		// Dimension of latent space, X feature, Y feature

		ColPivHouseholderQR<MatrixXf> qrX, qrY;
		// for transform
		JacobiSVD<MatrixXf,2> svdBt;	
		MatrixXf	invB;
	public:
		Cca() : m_initialized(false) {}

		Cca(const MatrixXf &X, const MatrixXf &Y) {
			compute(X, Y);
		}

		Cca& compute(const MatrixXf &X, const MatrixXf &Y, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB = false);

		const MatrixXf& matrixA() const { return A; }
		const MatrixXf& matrixB() const { return B; }
		const VectorXf& correlaltions() const { return R; }

		// transform X into correlated Y
		//? this is not finished yet!!!
		MatrixXf transform(const MatrixXf& X)
		{
			assert(X.cols() == dX); // dimension must agrees

			auto U = (X.rowwise() - uX) * A;
			if (rankY > d) // we need least square solution here
				return svdBt.solve(U.transpose()).transpose();
			else
				return U * invB;
		}
		// transform Y into correlated X
		MatrixXf inv_transform(const MatrixXf& Y);

	};

	template<typename _MatrixTypeX, typename _MatrixTypeY>
	inline Cca& Cca::computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB)
	{
		using namespace std;
		assert(qrX.rows() == qrX.rows());
		auto n = qrX.rows();
		// get ranks and latent space dimension
		rankX = qrX.rank();
		rankY = qrY.rank();
		d = min(rankX, rankY);
		// Get matrix Q,R and covQXY
		MatrixXf qX, qY;
		qrX.matrixQ().setLength(rankX).evalTo(qX);
		qrY.matrixQ().setLength(rankY).evalTo(qY);

		auto covQXY = qX.transpose() * qY;
		// SVD
		auto svd = covQXY.jacobiSvd();

		R = svd.singularValues().topRows(d);

		if (computeAB)
		{
			auto rX = qrX.matrixR().topLeftCorner(rankX, rankX).triangularView<Upper>();
			auto rY = qrY.matrixR().topLeftCorner(rankY, rankY).triangularView<Upper>();
			// A = rX \ U * sqrt(n-1)
			// B = rY \ V * sqrt(n-1)
			A = rX.solve(svd.matrixU().leftCols(d));
			B = rY.solve(svd.matrixV().leftCols(d));

			// normalize A,B and reverse the permutation , thus U,V will have unit varience
			A = A * sqrtf(n - 1) * qrX.colsPermutation().inverse();
			B = B * sqrtf(n - 1) * qrY.colsPermutation().inverse();

			// Compute Transform X -> Y
			if (d == rankY)
				invB = B.inverse();
			else
				svdBt = JacobiSVD<MatrixXf>(B.transpose());
		}

		m_initialized = true;
		return *this;
	}

	inline Cca& Cca::compute(const MatrixXf &X, const MatrixXf &Y, bool computeAB = false)
	{
		// Algorithm is explianed here : ( Qr + SVD version)
		// http://www.nr.com/whp/notes/CanonCorrBySVD.pdf
		assert(X.rows() == Y.rows());
		auto n = X.rows();
		dX = X.cols();
		dY = Y.cols();

		// zero mean X and Y
		uX = X.colwise().mean().eval();
		uY = Y.colwise().mean().eval();
		MatrixXf mX = X.rowwise() - uX; // dX x n
		MatrixXf mY = Y.rowwise() - uY; // dY x n

		// QR-decomposition
		auto qrX = mX.colPivHouseholderQr();
		auto qrY = mY.colPivHouseholderQr();
		
		return computeFromQr(qrX, qrY, computeAB);
	}

}