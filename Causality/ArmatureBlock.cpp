#include "pch_bcl.h"
#include "ArmatureBlock.h"
#include "Animations.h"

using namespace Causality;
using namespace DirectX;
using namespace Eigen;
using namespace std;

void Causality::BlockArmature::SetArmature(const IArmature & armature)
{
	m_pArmature = &armature;
	m_pRoot.reset(ShrinkChainToBlock(armature.root()));
	int idx = 0, accumIdx = 0;

	for (auto& block : m_pRoot->nodes())
	{
		m_BlocksCache.push_back(&block);
		block.Index = idx++;
		block.AccumulatedJointCount = accumIdx;
		accumIdx += block.Joints.size();
		//block.Wx = block.Wx.array().exp();
	}
}

void Causality::BlockArmature::ComputeWeights()
{
	const auto& frame = m_pArmature->default_frame();

	for (auto& pblcok : m_BlocksCache)
	{
		auto& block = *pblcok;
		float length = 0;
		block.Wxj.resize(block.Joints.size());
		block.Wxj.setZero();
		for (int i = block.Joints.size() - 1; i >= 0; --i)
		{
			length += frame[block.Joints[i]->ID()].LclTranslation.Length();
			block.Wxj[i] = length;
		}

		//block.Wxj /= block.Wxj[block.Wxj.size()-1];

		Matrix<float, -1, -1, RowMajor> wx = block.Wxj.replicate(1, 3).eval();
		block.Wx = VectorXf::Map(wx.data(), wx.size());
	}

}

Causality::BlockArmature::BlockArmature(const IArmature & armature)
{
	SetArmature(armature);
}

KinematicBlock* Causality::ShrinkChainToBlock(const Joint* pJoint)
{
	if (pJoint == nullptr || pJoint->is_null()) return nullptr;
	KinematicBlock* pBlock = new KinematicBlock;

	auto pChild = pJoint;
	do
	{
		pBlock->Joints.push_back(pChild);
		pJoint = pChild;
		pChild = pChild->first_child();
	} while (pChild && !pChild->next_sibling());

	for (auto& child : pJoint->children())
	{
		auto childBlcok = ShrinkChainToBlock(&child);
		pBlock->append_children_back(childBlcok);
	}
	return pBlock;
}

Eigen::PermutationMatrix<Eigen::Dynamic> Causality::BlockArmature::GetJointPermutationMatrix(size_t feature_dim) const
{
	using namespace Eigen;

	// Perform a Column permutation so that feature for block wise is consist
	// And remove the position data!!!

	auto N = m_pArmature->size();
	vector<int> indices(N);
	int idx = 0;

	for (auto pBlock : m_BlocksCache)
	{
		for (auto& pj : pBlock->Joints)
		{
			//indices[pj->ID()] = idx++;
			indices[idx++] = pj->ID();
		}
	}

	MatrixXi a_indvec = RowVectorXi::Map(indices.data(), indices.size()).replicate(feature_dim, 1);
	a_indvec.array() *= feature_dim;
	a_indvec.colwise() += Matrix<int, -1, 1>::LinSpaced(feature_dim, 0, feature_dim - 1);

#ifdef _DEBUG
	cout << a_indvec << endl;
#endif

	Eigen::PermutationMatrix<Eigen::Dynamic> perm(VectorXi::Map(a_indvec.data(), N*feature_dim));
	return perm;
}
