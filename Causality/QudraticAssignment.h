#pragma once
#include <vector>
#include <Eigen\Core>

// spamming std namespace
namespace std
{
	template <typename Iterator>
	inline bool next_combination(Iterator first,
		Iterator k,
		Iterator last);

	template <typename Iterator>
	inline bool next_combination(const Iterator first, Iterator k, const Iterator last)
	{
		/* Credits: Thomas Draper */
		// http://stackoverflow.com/a/5097100/8747
		if ((first == last) || (first == k) || (last == k))
			return false;
		Iterator itr1 = first;
		Iterator itr2 = last;
		++itr1;
		if (last == itr1)
			return false;
		itr1 = last;
		--itr1;
		itr1 = k;
		--itr2;
		while (first != itr1)
		{
			if (*--itr1 < *itr2)
			{
				Iterator j = k;
				while (!(*itr1 < *j)) ++j;
				std::iter_swap(itr1, j);
				++itr1;
				++j;
				itr2 = k;
				std::rotate(itr1, j, last);
				while (last != j)
				{
					++j;
					++itr2;
				}
				std::rotate(k, itr2, last);
				return true;
			}
		}
		std::rotate(first, k, last);
		return false;
	}
}

namespace Eigen
{
	// C(i,j,ass(i),ass(j)) must exist
	template <class QuadraticFuncType>
	float quadratic_assignment_cost(const Eigen::MatrixXf& A, const QuadraticFuncType &C, _In_reads_(A.rows()) Eigen::DenseIndex* ass)
	{
		using namespace std;
		using namespace Eigen;

		int n = A.rows();
		float score = 0;
		for (int i = 0; i < n; i++)
		{
			score += A(i, ass[i]);
			for (int j = i + 1; j < n; j++)
			{
				score += C(i, j, ass[i], ass[j]);
			}
		}
		return score;
	}

	// C(i,j,ass(i),ass(j)) must exist
	// Brute-force solve QAP
	template <class QuadraticFuncType>
	float max_quadratic_assignment(const Eigen::MatrixXf& A, const QuadraticFuncType &C, _Out_ std::vector<Eigen::DenseIndex>& assignment)
	{
		using namespace std;
		using namespace Eigen;
		auto nx = A.rows(), ny = A.cols();
		assert(ny >= nx);
		vector<DenseIndex> s(std::max(nx, ny));
		for (int i = 0; i < s.size(); i++)
		{
			s[i] = i;
		}

		vector<DenseIndex>  optAss(nx);
		float optScore = std::numeric_limits<float>::min();

		do {
			do {
				float score = quadratic_assignment_cost(A, C, s.data());
				if (score > optScore)
				{
					optAss.assign(s.begin(), s.begin() + nx);
					optScore = score;
				}
				for (auto& i : s)
				{
					cout << i << ' ';
				}
				//cout << ':' << score << endl;
			} while (std::next_permutation(s.begin(), s.begin() + nx));
		} while (next_combination(s.begin(), s.begin() + nx, s.end()));

		assignment = optAss;
		return optScore;
	}

}