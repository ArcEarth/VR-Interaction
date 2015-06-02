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

	// zero mean and Decompose X, thus (X + repmat(uX,n,1)) * E = [Q 0] * [R ; 0]
	template< class _MatrixType>
	struct MeanThinQr
	{
		typedef _MatrixType MatrixType;
		enum {
			RowsAtCompileTime = MatrixType::RowsAtCompileTime,
			ColsAtCompileTime = MatrixType::ColsAtCompileTime,
			Options = MatrixType::Options,
			MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
			MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
		};

		typedef typename MatrixType::Scalar Scalar;
		typedef typename MatrixType::RealScalar RealScalar;
		typedef typename MatrixType::Index Index;
		typedef Matrix<Scalar, RowsAtCompileTime, Dynamic, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime> MatrixQType;
		typedef Matrix<Scalar, Dynamic, Dynamic, Options, MaxColsAtCompileTime, MaxColsAtCompileTime> MatrixRType;
		typedef PermutationMatrix<ColsAtCompileTime, MaxColsAtCompileTime> PermutationType;
		typedef TriangularView<const MatrixRType, Upper> MatrixRTriangularViewType;
		typedef Matrix<Scalar, 1, ColsAtCompileTime,RowMajor,1,MaxColsAtCompileTime> MeanVectorType;

		DenseIndex m_rows,m_cols,m_rank;	 // cols, rows, and rank
		MeanVectorType m_mean;			 // Rowwise Mean Vector of X, 1 x cols
		MatrixQType m_Q;		 // Self-Adjoint matrix Q , rows x rank
		MatrixRType m_R;		 // Upper triangluar matrix R , rank x rank
		PermutationType m_E; // column permutation matrix E , cols x cols

		inline DenseIndex rows() const { return m_rows; }
		inline DenseIndex cols() const { return m_cols; }
		inline DenseIndex rank() const { return m_rank; }
		inline const MeanVectorType& mean() const { return m_mean; }
		inline const MatrixQType& matrixQ() const{ return m_Q; }
		inline MatrixRTriangularViewType matrixR() const{ return m_R.triangularView<Upper>(); }
		inline const PermutationType& colsPermutation() const { return m_E; }

		MeanThinQr()
		{
		}

		MeanThinQr(const MatrixType& X)
		{
			compute(X);
		}

		void compute(const MatrixType& X, bool zeroMean = true)
		{
			m_cols = X.cols();
			m_rows = X.rows();

			MatrixXf mX = X;
			if (zeroMean)
			{
				m_mean = X.colwise().mean().eval();
				mX.rowwise() -= m_mean; // dX x n
			}
			else
			{
				m_mean.setZero(1,X.cols());
			}

			// QR-decomposition
			auto qrX = mX.colPivHouseholderQr();

			m_rank = qrX.rank();
			m_E = qrX.colsPermutation();
			m_R = qrX.matrixR().topLeftCorner(m_rank, m_rank).triangularView<Upper>();

			MatrixXf qX;
			qrX.matrixQ().evalTo(qX);
			m_Q = qX.leftCols(m_rank);
		}

	};

	// Canonical correlation analysis
	// X : n x dX input vectors
	// Y : n x dY input vectors
	// n : number of observations
	// dX : dimension of feature X
	// dY : dimension of feature Y
	// return : correlation coefficients R, transform matrix A,B that maps X and Y to Latent space
	// thus argmax(A,B) corr(XA,YB)
	// it's best to make sure X Y are full-rank in the sense of colume rank
	// thus X' * A ~ Y' * B (X',Y' is zero meaned X,Y)
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
		JacobiSVD<MatrixXf, 2> svdCovQXY;

		// for transform
		mutable JacobiSVD<MatrixXf,2> svdBt;	
		mutable MatrixXf	invB;
	public:
		Cca() : m_initialized(false) {}

		Cca(const MatrixXf &X, const MatrixXf &Y) {
			compute(X, Y);
		}

		Cca& compute(const MatrixXf &X, const MatrixXf &Y, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromQr(const MeanThinQr<_MatrixTypeX>& qrX, const MeanThinQr<_MatrixTypeY>& qrY, bool computeAB = false);

		DenseIndex rank() const { return d; }
		const MatrixXf& matrixA() const { return A; }
		const MatrixXf& matrixB() const { return B; }
		const VectorXf& correlaltions() const { return R; }

		// transform X into correlated Y
		//? this is not finished yet!!!
		MatrixXf transform(const MatrixXf& X) const
		{
			assert(X.cols() == dX); // dimension must agrees

			auto U = ((X.rowwise() - uX) * A).eval();

			MatrixXf Y;

			if (dY > d) // we need least square solution here
				Y = svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
			else
				Y = U * invB; // Y = U / B

			Y.rowwise() += uY;
			return Y;
		}

		// transform Y into correlated X
		MatrixXf inv_transform(const MatrixXf& Y);

	};

#define DebugLog(mat) #mat << " = " << endl << mat << endl 

	template<typename _MatrixTypeX, typename _MatrixTypeY>
	inline Cca& Cca::computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB)
	{
		using namespace std;
		assert(qrX.rows() == qrX.rows());
		auto n = qrX.rows();
		// get ranks and latent space dimension
		rankX = qrX.rank();
		rankY = qrY.rank();
		dX = qrX.cols();
		dY = qrY.cols();


		d = min(rankX, rankY);

		// Get matrix Q,R and covQXY
		MatrixXf qX, qY;
		qrX.matrixQ().evalTo(qX);
		qrY.matrixQ().evalTo(qY);
		//? This impl is not efficient!

		auto covQXY = (qX.leftCols(rankX).transpose() * qY.leftCols(rankY)).eval();

		//cout << "cov(Qx,Qy) = " << endl;
		//cout << covQXY << endl;

		// SVD
		unsigned int option = computeAB ? ComputeThinU | ComputeThinV : 0;
		auto svd = covQXY.jacobiSvd(option); // svdCovQXY

		R = svd.singularValues().topRows(d);

		//cout << DebugLog(R);

		if (computeAB)
		{
			auto rX = qrX.matrixR().topLeftCorner(rankX, rankX).triangularView<Upper>();
			auto rY = qrY.matrixR().topLeftCorner(rankY, rankY).triangularView<Upper>();

			A.setZero(dX, d);
			B.setZero(dY, d);

			// A = rX \ U * sqrt(n-1)
			// B = rY \ V * sqrt(n-1)
			A.topRows(rankX).noalias() = rX.solve(svd.matrixU().leftCols(d)) * sqrtf(n - 1.0f); // A : rankX x d
			B.topRows(rankY).noalias() = rY.solve(svd.matrixV().leftCols(d)) * sqrtf(n - 1.0f); // B : rankY x d
			// normalize A,B and reverse the permutation , thus U,V will have unit varience

			// Put coefficients back to their full size and their correct order
			A = qrX.colsPermutation().inverse() * A; // A : dX x d
			B = qrY.colsPermutation().inverse() * B; // B : dY x d

			//cout << DebugLog(A);
			//cout << DebugLog(B);

			// Compute Transform X -> Y
			if (d == dY)
				invB = B.inverse();
			else
				svdBt = JacobiSVD<MatrixXf>(B.transpose());
		}

		m_initialized = true;
		return *this;
	}

	template<typename _MatrixTypeX, typename _MatrixTypeY>
	inline Cca& Cca::computeFromQr(const MeanThinQr<_MatrixTypeX>& qrX, const MeanThinQr<_MatrixTypeY>& qrY, bool computeAB)
	{
		using namespace std;
		assert(qrX.rows() == qrX.rows());
		auto n = qrX.rows();
		// get ranks and latent space dimension
		rankX = qrX.rank();
		rankY = qrY.rank();
		dX = qrX.cols();
		dY = qrY.cols();

		uX = qrX.mean();
		uY = qrY.mean();

		d = min(rankX, rankY);

		// Get matrix Q,R and covQXY
		auto covQXY = (qrX.matrixQ().transpose() * qrY.matrixQ()).eval();

		//cout << DebugLog(covQXY);

		// SVD
		unsigned int option = computeAB ? ComputeThinU | ComputeThinV : 0;
		auto svd = covQXY.jacobiSvd(option); // svdCovQXY

		R = svd.singularValues().topRows(d);

		//cout << DebugLog(R);

		if (computeAB)
		{
			auto rX = qrX.matrixR();
			auto rY = qrY.matrixR();

			A.setZero(dX, d);
			B.setZero(dY, d);

			// A = rX \ U * sqrt(n-1)
			// B = rY \ V * sqrt(n-1)
			A.topRows(rankX).noalias() = rX.solve(svd.matrixU().leftCols(d)) * sqrtf(n - 1.0f); // A : rankX x d
			B.topRows(rankY).noalias() = rY.solve(svd.matrixV().leftCols(d)) * sqrtf(n - 1.0f); // B : rankY x d
																								// Put coefficients back to their full size and their correct order
			A = qrX.colsPermutation().inverse() * A; // A : dX x d
			B = qrY.colsPermutation().inverse() * B; // B : dY x d

			//cout << DebugLog(A);
			//cout << DebugLog(B);

			// Compute Transform X -> Y
			if (d == dY)
				invB = B.inverse();
			else
				svdBt = JacobiSVD<MatrixXf>(B.transpose());
		}

		m_initialized = true;
		return *this;
	}

	inline Cca& Cca::compute(const MatrixXf &X, const MatrixXf &Y, bool computeAB)
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