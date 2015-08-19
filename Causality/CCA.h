#pragma once
#include <Eigen\Dense>

namespace Eigen {

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
			: m_rank(0)
		{
		}

		MeanThinQr(const MatrixType& X)
		{
			compute(X);
		}

		void compute(const MatrixType& X, bool zeroMean = true)
		{
			if (X.size() == 0)
			{
				m_rank = 0;
				return;
			}

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

	template<class _MatrixType>
	struct Pca
	{
	public:
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

		typedef Matrix<Scalar, _MatrixType::ColsAtCompileTime, _MatrixType::ColsAtCompileTime> PrincipleComponentsType;
		typedef Matrix<Scalar, 1, _MatrixType::ColsAtCompileTime> RowVectorType;
	private:
		PrincipleComponentsType m_Comps;
		RowVectorType			m_Variences;
		RowVectorType			m_Mean;
		MatrixXf				m_Coords;
	public:
		Pca()
		{
		}

		Pca(const MatrixType& X)
		{
			compute(X);
		}

		template <typename Derived>
		void compute(const DenseBase<Derived>& X, bool computeCoords = true)
		{
			m_Mean = X.colwise().mean();
			MatrixType Xz = (X.rowwise() - m_Mean).eval();

			auto svd = Xz.jacobiSvd(ComputeThinV);
			m_Comps = svd.matrixV();
			m_Variences = svd.singularValues();
			m_Variences = m_Variences.cwiseAbs2();

			if (computeCoords)
				m_Coords = Xz * m_Comps;
		}

		// the demension after projection
		DenseIndex reducedRank(float cut_threshold_percentage) const 
		{ 
			int rank;
			float cut_threshold = m_Variences[0] * cut_threshold_percentage;
			auto cols = m_Variences.size();
			for (rank = 0; rank < cols && m_Variences[rank] > cut_threshold; rank++);
			return rank;
		}
		// column-wise principle components matrix
		auto components(DenseIndex nCols = -1) const {
			if (nCols < 0)
				nCols = m_Coords.cols();
			return m_Comps.leftCols(nCols);
		}
		// projected coordinates
		auto coordinates(DenseIndex nCols = -1) const
		{
			// you must spificy computeCoords = true in construction
			assert(m_Coords.size() > 0);

			if (nCols < 0)
				nCols = m_Coords.cols();
			return m_Coords.leftCols(nCols);
		}
		const auto& variences() const
		{
			return m_Variences;
		}
		const auto& mean() const { return m_Mean; }

	};

	// Canonical correlation analysis
	// template praram <_TScalar> : scalar used internal for this CCA
	// X : n x dX input vectors
	// Y : n x dY input vectors
	// n : number of observations
	// dX : dimension of feature X
	// dY : dimension of feature Y
	// return : correlation coefficients R, transform matrix A,B that maps X and Y to Latent space
	// thus argmax(A,B) corr(XA,YB)
	// it's best to make sure X Y are full-rank in the sense of colume rank
	// thus X' * A ~ Y' * B (X',Y' is zero meaned X,Y)
	template<class _TScalar>
	class Cca
	{
		typedef typename _TScalar Scalar;

		typedef Matrix<Scalar, -1, -1> MatrixType;
		typedef Matrix<Scalar,  1, -1> RowVectorType;
		typedef Matrix<Scalar, -1,  1> VectorType;

	private:
		bool m_initialized;
		MatrixType		A, B;	 // transform matrix to latent space
		VectorType		R;		 // Correlations
		RowVectorType	uX, uY;	 // mean vector of X and Y
		DenseIndex		rankX, rankY;	// rank of input X and Y
		DenseIndex		d, dX, dY;		// Dimension of latent space, X feature, Y feature

		ColPivHouseholderQR<MatrixType> qrX, qrY;
		JacobiSVD<MatrixType, 2> svdCovQXY;

		// for transform
		mutable JacobiSVD<MatrixType,2> svdBt;
		mutable MatrixType				invB;
	public:
		Cca() : m_initialized(false) {}


		template <typename DerivedX, typename DerivedY>
		Cca(const DenseBase<DerivedX> &X, const DenseBase<DerivedY> &Y, bool computeAB = false) 
		{
			compute(X, Y, computeAB);
		}

		template <typename DerivedX, typename DerivedY>
		Cca& compute(const DenseBase<DerivedX> &X, const DenseBase<DerivedY> &Y, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromQr(const MeanThinQr<_MatrixTypeX>& qrX, const MeanThinQr<_MatrixTypeY>& qrY, bool computeAB = false);

		template<typename _MatrixTypeX, typename _MatrixTypeY>
		Cca& computeFromPca(const Pca<_MatrixTypeX>& pcaX, const Pca<_MatrixTypeY>& pcaY, bool computeAB = false);

		DenseIndex rank() const { return d; }
		const MatrixType& matrixA() const { return A; }
		const MatrixType& matrixB() const { return B; }
		const VectorType& correlaltions() const { return R; }

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

	template<typename _TScalar>
	template<typename _MatrixTypeX, typename _MatrixTypeY>
	inline Cca<_TScalar>& Cca<_TScalar>::computeFromQr(const ColPivHouseholderQR<_MatrixTypeX>& qrX, const ColPivHouseholderQR<_MatrixTypeY>& qrY, bool computeAB)
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
		MatrixType qX, qY;
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
				svdBt = JacobiSVD<MatrixType>(B.transpose());
		}

		m_initialized = true;
		return *this;
	}

	template<typename _TScalar>
	template<typename _MatrixTypeX, typename _MatrixTypeY>
	inline Cca<_TScalar>& Cca<_TScalar>::computeFromQr(const MeanThinQr<_MatrixTypeX>& qrX, const MeanThinQr<_MatrixTypeY>& qrY, bool computeAB)
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
		if (d == 0)
			return *this;

		// Get matrix Q,R and covQXY
		auto covQXY = (qrX.matrixQ().transpose() * qrY.matrixQ()).eval();

		//std::cout << DebugLog(covQXY);

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
			//std::cout << DebugLog(svd.matrixU());
			//std::cout << DebugLog(svd.matrixV());

			//std::cout << DebugLog(A);
			//std::cout << DebugLog(B);

			// Put coefficients back to their full size and their correct order
			A = qrX.colsPermutation() * A; // A : dX x d
			B = qrY.colsPermutation() * B; // B : dY x d

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

	template<class _TScalar>
	template <typename DerivedX, typename DerivedY>
	inline Cca<_TScalar>& Cca<_TScalar>::compute(const DenseBase<DerivedX> &X, const DenseBase<DerivedY> &Y, bool computeAB)
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
		MatrixType mX = X.rowwise() - uX; // dX x n
		MatrixType mY = Y.rowwise() - uY; // dY x n

		// QR-decomposition
		auto qrX = mX.colPivHouseholderQr();
		auto qrY = mY.colPivHouseholderQr();
		
		return computeFromQr(qrX, qrY, computeAB);
	}

	struct CcaMap
	{
		DenseIndex Jx, Jy;
		MatrixXf A, B;
		RowVectorXf uX, uY;
		JacobiSVD<MatrixXf> svdBt;
		MatrixXf invB;
		bool useInvB;

		template <class DerivedX, class DerivedY>
		void Apply(_In_ const DenseBase<DerivedX> &Xp, _Out_ DenseBase<DerivedY> &Yp) const;

		float CreateFrom(_In_ const MatrixXf &X, const _In_ MatrixXf &Y);
	};

	struct PcaCcaMap : public CcaMap
	{
		MatrixXf	pcX, pcY; // Principle components of X or Y
		RowVectorXf uXpca, uYpca; // Mean of X or Y

		template <class DerivedX, class DerivedY>
		void Apply(_In_ const DenseBase<DerivedX> &Xp, _Out_ MatrixBase<DerivedY> &Yp) const;

		float CreateFrom(_In_ const MatrixXf &X, const _In_ MatrixXf &Y, float Xcutoff = 0.04f, float Ycutoff = 0.04f);
	};

	template <class DerivedX, class DerivedY>
	inline void ApplyCcaMap(_In_ const CcaMap& map, _In_ const DenseBase<DerivedX> &Xp, _Out_ DenseBase<DerivedY> &Yp)
	{
		auto U = ((Xp.rowwise() - map.uX) * map.A).eval();
		if (map.useInvB)
			Yp = U * map.invB;
		else
			Yp = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
		Yp.rowwise() += map.uY;
	}

	template <class DerivedX, class DerivedY>
	inline void ApplyPcaCcaMap(_In_ const PcaCcaMap& map, _In_ const DenseBase<DerivedX> &Xp, _Out_ MatrixBase<DerivedY> &Yp)
	{
		// Project X with PCA
		auto Xpca = ((Xp.rowwise() - map.uXpca) * map.pcX).eval();
		// Project Xpca with CCA to latent space
		auto U = ((Xpca.rowwise() - map.uX) * map.A).eval();
		// Recover Y from latent space
		Matrix<DenseBase<DerivedX>::Scalar,-1,-1> Y;
		if (map.useInvB)
			Y = U * map.invB;
		else
			Y = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
															 // Add the mean
		Y.rowwise() += map.uY;
		// Reconstruct by principle components
		Yp = Y * map.pcY.transpose();
		Yp.rowwise() += map.uYpca;
	}

	template <class DerivedX, class DerivedY>
	inline void CcaMap::Apply(_In_ const DenseBase<DerivedX> &Xp, _Out_ DenseBase<DerivedY> &Yp) const
	{
		ApplyCcaMap<DerivedX, DerivedY>(*this, Xp, Yp);
	}

	template <class DerivedX, class DerivedY>
	inline void PcaCcaMap::Apply(_In_ const DenseBase<DerivedX> &Xp, _Out_ MatrixBase<DerivedY> &Yp) const
	{
		ApplyPcaCcaMap<DerivedX,DerivedY>(*this, Xp, Yp);
	}

	inline float CreateCcaMap(CcaMap& map, _In_ const MatrixXf &X, const _In_ MatrixXf &Y)
	{
		MeanThinQr<MatrixXf> qrX(X), qrY(Y);
		Cca<float> cca;
		cca.computeFromQr(qrX, qrY, true);

		if (cca.rank() == 0) return .0f;

		map.Jx = 0; map.Jy = 0;
		map.A = cca.matrixA();
		map.B = cca.matrixB();
		map.uX = qrX.mean();
		map.uY = qrY.mean();

		if (cca.rank() == qrY.cols()) // d == dY
		{
			map.useInvB = true;
			map.invB = map.B.inverse();
		}
		else
		{
			map.useInvB = false;
			map.svdBt = JacobiSVD<MatrixXf>(map.B.transpose(), ComputeThinU | ComputeThinV);;
		}
		return cca.correlaltions().minCoeff();
	}

	inline float CreatePcaCcaMap(PcaCcaMap& map, _In_ const MatrixXf &X, _In_ const MatrixXf &Y, float Xcutoff = 0.04f, float Ycutoff = 0.04f)
	{
		Pca<MatrixXf> pcaX(X), pcaY(Y);
		map.uXpca = pcaX.mean();
		map.uYpca = pcaY.mean();

		auto dX = pcaX.reducedRank(Xcutoff);
		auto dY = pcaY.reducedRank(Ycutoff);
		map.pcX = pcaX.components(dX);
		map.pcY = pcaY.components(dY);

		return CreateCcaMap(map, pcaX.coordinates(dX), pcaY.coordinates(dY));
	}

	inline float CcaMap::CreateFrom(_In_ const MatrixXf &X, const _In_ MatrixXf &Y)
	{
		return CreateCcaMap(*this, X, Y);
	}

	inline float PcaCcaMap::CreateFrom(_In_ const MatrixXf &X, const _In_ MatrixXf &Y, float Xcutoff , float Ycutoff )
	{
		return CreatePcaCcaMap(*this, X, Y, Xcutoff, Ycutoff);
	}

}