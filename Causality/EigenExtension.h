#pragma once
#include <Eigen\Dense>
#include <Eigen\fft>
#include <algorithm>

#include <cstdlib>
#include <cstdint>


namespace Eigen {

	template <class InputDerived> inline 
	Matrix< typename internal::traits<InputDerived>::Scalar,
			Dynamic,
			internal::traits<InputDerived>::ColsAtCompileTime>
	resample(const MatrixBase<InputDerived> &X, DenseIndex p, DenseIndex q)
	{
		using traits = internal::traits<InputDerived>;
		typedef typename traits::Scalar Scalar;
		using namespace std;

		assert(!X.IsRowMajor);
		auto n = X.rows();
		auto m = n * q / p;
		auto c = max(m, n);
		
		Matrix<std::complex<Scalar>, Dynamic, 1> Xf(c);
		Matrix< traits::Scalar, Dynamic, traits::ColsAtCompileTime> Xr(m, X.cols());
		//Xr.resize(m, X.cols());
		Xf.setZero();

		FFT<Scalar> fft;
		for (size_t i = 0; i < X.cols(); i++)
		{
			fft.fwd(Xf.data(), X.col(i).data(),n);
			fft.inv(Xr.col(i).data(), Xf.data(),m);
		}
		return Xr;
	}

	namespace impl
	{
		template <class Derived> inline
		inline void compute_slack(
			const DenseIndex x,
			std::vector<typename Derived::Scalar>& slack,
			std::vector<DenseIndex>& slackx,
			const DenseBase<Derived> & cost,
			const std::vector<typename Derived::Scalar>& lx,
			const std::vector<typename Derived::Scalar>& ly
			)
		{
			auto n = ly.size();
			for (size_t y = 0; y < n; ++y)
			{
				if (lx[x] + ly[y] - cost(x, y) < slack[y])
				{
					slack[y] = lx[x] + ly[y] - cost(x, y);
					slackx[y] = x;
				}
			}
		}
	}

	template <class Derived> inline
		typename internal::traits<Derived>::Scalar 
		matching_cost(const DenseBase<Derived> &cost, const std::vector<DenseIndex>& matching)
	{
		typedef typename internal::traits<Derived>::Scalar Scalar;
		Scalar sum(0);
		for (size_t i = 0; i < matching.size(); i++)
		{
			auto j = matching[i];
			if (j != -1)
				sum += cost(i, j);
		}
		return sum;
	}

	template <class Derived> inline 
		std::vector<DenseIndex>
		max_weight_bipartite_matching(const DenseBase<Derived> &cost_)
	{
		typedef DenseBase<Derived> MatrixType;
		typedef typename internal::traits<Derived>::Scalar Scalar;
		typedef Scalar scalar_type;
		typedef DenseIndex index_type;
		using namespace impl;
		// Kuhn-Munkres Algorithm

		if (cost_.size() == 0)
			return std::vector<index_type>();

		size_t n = std::max(cost_.rows(),cost_.cols());
		Matrix<scalar_type, -1, -1> cost(n,n);
		cost.setZero();
		cost.block(0, 0, cost_.rows(), cost_.cols()) = cost_;

		std::vector<scalar_type> lx(n), ly(n);
		std::vector<index_type> xy;
		std::vector<index_type> yx;
		std::vector<char> S, T;
		std::vector<scalar_type> slack;
		std::vector<index_type> slackx;
		std::vector<index_type> aug_path;

		// Initially, nothing is matched. 
		xy.assign(n, -1);
		yx.assign(n, -1);
		/*
		We maintain the following invariant:
		Vertex x is matched to vertex xy[x] and
		vertex y is matched to vertex yx[y].

		A value of -1 means a vertex isn't matched to anything.  Moreover,
		x corresponds to rows of the cost matrix and y corresponds to the
		columns of the cost matrix.  So we are matching X to Y.
		*/

		// Create an initial feasible labeling.  Moreover, in the following
		// code we will always have: 
		//     for all valid x and y:  lx[x] + ly[y] >= cost(x,y)
		// Intialize flexable labels
		auto Lx = VectorXf::Map(lx.data(), n) ;
		Lx = cost.rowwise().maxCoeff();
		ly.resize(n);
		ly.assign(n, 0);

		// Now grow the match set by picking edges from the equality subgraph until
		// we have a complete matching.
		for (long match_size = 0; match_size < n; ++match_size)
		{
			std::deque<long> q;

			// Empty out the S and T sets
			S.assign(n, false);
			T.assign(n, false);

			// clear out old slack values
			slack.assign(n, std::numeric_limits<scalar_type>::max());
			slackx.resize(n);
			/*
			slack and slackx are maintained such that we always
			have the following (once they get initialized by compute_slack() below):
			- for all y:
			- let x == slackx[y]
			- slack[y] == lx[x] + ly[y] - cost(x,y)
			*/

			aug_path.assign(n, -1);

			for (long x = 0; x < n; ++x)
			{
				// If x is not matched to anything
				if (xy[x] == -1)
				{
					q.push_back(x);
					S[x] = true;

					compute_slack(x, slack, slackx, cost, lx, ly);
					break;
				}
			}


			long x_start = 0;
			long y_start = 0;

			// Find an augmenting path.  
			bool found_augmenting_path = false;
			while (!found_augmenting_path)
			{
				while (q.size() > 0 && !found_augmenting_path)
				{
					const long x = q.front();
					q.pop_front();
					for (long y = 0; y < n; ++y)
					{
						if (cost(x, y) == lx[x] + ly[y] && !T[y])
						{
							// if vertex y isn't matched with anything
							if (yx[y] == -1)
							{
								y_start = y;
								x_start = x;
								found_augmenting_path = true;
								break;
							}

							T[y] = true;
							q.push_back(yx[y]);

							aug_path[yx[y]] = x;
							S[yx[y]] = true;
							compute_slack(yx[y], slack, slackx, cost, lx, ly);
						}
					}
				}

				if (found_augmenting_path)
					break;


				// Since we didn't find an augmenting path we need to improve the 
				// feasible labeling stored in lx and ly.  We also need to keep the
				// slack updated accordingly.
				scalar_type delta = std::numeric_limits<scalar_type>::max();
				for (unsigned long i = 0; i < T.size(); ++i)
				{
					if (!T[i])
						delta = std::min(delta, slack[i]);
				}
				for (unsigned long i = 0; i < T.size(); ++i)
				{
					if (S[i])
						lx[i] -= delta;

					if (T[i])
						ly[i] += delta;
					else
						slack[i] -= delta;
				}



				q.clear();
				for (long y = 0; y < n; ++y)
				{
					if (!T[y] && slack[y] == 0)
					{
						// if vertex y isn't matched with anything
						if (yx[y] == -1)
						{
							x_start = slackx[y];
							y_start = y;
							found_augmenting_path = true;
							break;
						}
						else
						{
							T[y] = true;
							if (!S[yx[y]])
							{
								q.push_back(yx[y]);

								aug_path[yx[y]] = slackx[y];
								S[yx[y]] = true;
								compute_slack(yx[y], slack, slackx, cost, lx, ly);
							}
						}
					}
				}
			} // end while (!found_augmenting_path)

			  // Flip the edges aDenseIndex the augmenting path.  This means we will add one more
			  // item to our matching.
			for (long cx = x_start, cy = y_start, ty;
			cx != -1;
				cx = aug_path[cx], cy = ty)
			{
				ty = xy[cx];
				yx[cy] = cx;
				xy[cx] = cy;
			}

		}

		if (cost_.rows() < n)
			xy.resize(cost_.rows());
		else if (cost_.cols() < n)
			for (auto& xyi : xy)
				if (xyi >= cost_.cols())
					xyi = -1;

		return xy;

	}

}