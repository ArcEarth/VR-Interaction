#pragma once
#include "CCA.h"

namespace Causality
{
	struct CcaMap
	{
		Eigen::DenseIndex Jx, Jy;
		Eigen::MatrixXf A, B;
		Eigen::RowVectorXf uX, uY;
		Eigen::JacobiSVD<Eigen::MatrixXf> svdBt;
		Eigen::MatrixXf invB;
		bool useInvB;

		template <class DerivedX, class DerivedY>
		void Apply(_In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::DenseBase<DerivedY> &Yp) const;

		float CreateFrom(_In_ const Eigen::MatrixXf &X, const _In_ Eigen::MatrixXf &Y);
	};

	struct PcaCcaMap : public CcaMap
	{
		Eigen::MatrixXf	pcX, pcY; // Principle components of X or Y
		Eigen::RowVectorXf uXpca, uYpca; // Mean of X or Y

		template <class DerivedX, class DerivedY>
		void Apply(_In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::MatrixBase<DerivedY> &Yp) const;

		float CreateFrom(_In_ const Eigen::MatrixXf &X, const _In_ Eigen::MatrixXf &Y, float Xcutoff = 0.04f, float Ycutoff = 0.04f);
	};

	template <class DerivedX, class DerivedY>
	inline void ApplyCcaMap(_In_ const CcaMap& map, _In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::DenseBase<DerivedY> &Yp)
	{
		auto U = ((Xp.rowwise() - map.uX) * map.A).eval();
		if (map.useInvB)
			Yp = U * map.invB;
		else
			Yp = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
		Yp.rowwise() += map.uY;
	}

	template <class DerivedX, class DerivedY>
	inline void ApplyPcaCcaMap(_In_ const PcaCcaMap& map, _In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::MatrixBase<DerivedY> &Yp)
	{
		using namespace Eigen;
		// Project X with PCA
		auto Xpca = ((Xp.rowwise() - map.uXpca) * map.pcX).eval();
		// Project Xpca with CCA to latent space
		auto U = ((Xpca.rowwise() - map.uX) * map.A).eval();
		// Recover Y from latent space
		Matrix<DenseBase<DerivedX>::Scalar, -1, -1> Y;
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
	inline void CcaMap::Apply(_In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::DenseBase<DerivedY> &Yp) const
	{
		ApplyCcaMap<DerivedX, DerivedY>(*this, Xp, Yp);
	}

	template <class DerivedX, class DerivedY>
	inline void PcaCcaMap::Apply(_In_ const Eigen::DenseBase<DerivedX> &Xp, _Out_ Eigen::MatrixBase<DerivedY> &Yp) const
	{
		ApplyPcaCcaMap<DerivedX, DerivedY>(*this, Xp, Yp);
	}

	inline float CreateCcaMap(CcaMap& map, _In_ const Eigen::MatrixXf &X, const _In_ Eigen::MatrixXf &Y)
	{
		using namespace Eigen;

		Eigen::MeanThinQr<Eigen::MatrixXf> qrX(X), qrY(Y);
		Eigen::Cca<float> cca;
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
			map.svdBt = Eigen::JacobiSVD<Eigen::MatrixXf>(map.B.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);;
		}
		return cca.correlaltions().minCoeff();
	}

	inline float CreatePcaCcaMap(PcaCcaMap& map, _In_ const Eigen::MatrixXf &X, _In_ const Eigen::MatrixXf &Y, float Xcutoff = 0.04f, float Ycutoff = 0.04f)
	{
		using namespace Eigen;
		Pca<MatrixXf> pcaX(X), pcaY(Y);
		map.uXpca = pcaX.mean();
		map.uYpca = pcaY.mean();

		auto dX = pcaX.reducedRank(Xcutoff);
		auto dY = pcaY.reducedRank(Ycutoff);
		map.pcX = pcaX.components(dX);
		map.pcY = pcaY.components(dY);

		return CreateCcaMap(map, pcaX.coordinates(dX), pcaY.coordinates(dY));
	}

	inline float CcaMap::CreateFrom(_In_ const Eigen::MatrixXf &X, const _In_ Eigen::MatrixXf &Y)
	{
		return CreateCcaMap(*this, X, Y);
	}

	inline float PcaCcaMap::CreateFrom(_In_ const Eigen::MatrixXf &X, const _In_ Eigen::MatrixXf &Y, float Xcutoff, float Ycutoff)
	{
		return CreatePcaCcaMap(*this, X, Y, Xcutoff, Ycutoff);
	}
}