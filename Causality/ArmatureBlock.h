#pragma once
#include "Armature.h"

namespace Causality
{

	// A Kinematic Block is a one-chain in the kinmatic tree, with additional anyalaze information 
	// usually constructed from shrinking the kinmatic tree
	// Each Block holds the children joints and other structural information
	// It is also common to build Multi-Level-of-Detail Block Layers
	class KinematicBlock : public tree_node<KinematicBlock>
	{
	public:
		int					Index;				// Index for this block, valid only through this layer
		int					LoD;				// Level of detail
		vector<const Joint*>Joints;				// Contained Joints

												// Structural feature
		int					LoG;				// Level of Grounding
		SymetricTypeEnum	SymetricType;		// symmetry type
		float				ExpandThreshold;	// The threshold to expand this part
		int					GroundIdx;

		KinematicBlock*		GroundParent;		// Path to grounded bone
		KinematicBlock*		SymetricPair;		// Path to grounded bone

		KinematicBlock*			LoDParent;		// Parent in LoD term
		vector<KinematicBlock*> LoDChildren;	// Children in LoD term

		int					AccumulatedJointCount; // The count of joints that owned by blocks who have minor index

		// Local Animation Domination properties
		KinematicBlock*			Dominator;
		list<KinematicBlock*>	Slaves;

		// Saliansy properties
		int					ActiveActionCount;
		list<string>		ActiveActions;

		KinematicBlock()
		{
			Index = -1;
			LoD = 0;
			LoG = 0;
			SymetricType = SymetricTypeEnum::Symetric_None;
			ExpandThreshold = 0;
			GroundIdx = 0;
			GroundParent = 0;
			SymetricPair = 0;
			LoDParent = 0;
			Dominator = 0;
			ActiveActionCount = 0;
		}
		// Motion and geometry feature
		//bool				IsActive;			// Is this feature "Active" in energy?
		//bool				IsStable;			// Is Current state a stable state
		//float				MotionEnergy;		// Motion Energy Level
		//float				PotientialEnergy;	// Potenial Energy Level

		BoundingOrientedBox GetBoundingBox(const AffineFrame& frame) const;
		template <class FeatureType>
		RowVectorX			GetFeatureVector(const AffineFrame& frame, bool blockwiseLocalize = false) const;
		template <class FeatureType>
		void				SetFeatureVector(_Out_ AffineFrame& frame, _In_ const RowVectorX& feature) const;
		template <class FeatureType>
		size_t				GetFeatureDim() const {
			return FeatureType::Dimension * Joints.size();
		}

		bool				IsEndEffector() const;		// Is this feature a end effector?
		bool				IsGrounded() const;			// Is this feature grounded? == foot semantic
		bool				IsSymetric() const;			// Is this feature a symetric pair?
		bool				IsLeft() const;				// Is this feature left part of a symtric feature
		bool				IsDominated() const;
		bool				HasSlaves() const;
	};

	// Shrink a Kinmatic Chain structure to a KinematicBlock
	KinematicBlock* ShrinkChainToBlock(const Joint* pJoint);


	class BlockArmature
	{
	private:
		std::unique_ptr<KinematicBlock> m_pRoot;
		const IArmature*				m_pArmature;
		std::vector<KinematicBlock*>	m_BlocksCache;
	public:
		typedef std::vector<KinematicBlock*> cache_type;

		void SetArmature(const IArmature& armature);

		explicit BlockArmature(const IArmature& armature);

		BlockArmature() = default;

		bool   empty() const { return m_pArmature == nullptr; }
		size_t size() const { return m_BlocksCache.size(); }

		auto begin() { return m_BlocksCache.begin(); }
		auto end() { return m_BlocksCache.end(); }
		auto begin() const { return m_BlocksCache.begin(); }
		auto end() const { return m_BlocksCache.end(); }
		auto rbegin() { return m_BlocksCache.rbegin(); }
		auto rend() { return m_BlocksCache.rend(); }
		auto rbegin() const { return m_BlocksCache.rbegin(); }
		auto rend() const { return m_BlocksCache.rend(); }

		const IArmature& Armature() const { return *m_pArmature; }
		KinematicBlock* Root() { return m_pRoot.get(); }
		const KinematicBlock* Root() const { return m_pRoot.get(); }
		KinematicBlock* operator[](int id) { return m_BlocksCache[id]; }
		const KinematicBlock* operator[](int id) const { return m_BlocksCache[id]; }

		// the matrix which alters joints into 
		Eigen::PermutationMatrix<Eigen::Dynamic> GetJointPermutationMatrix(size_t feature_dim) const;
	};

	template <class FeatureType>
	inline RowVectorX KinematicBlock::GetFeatureVector(const AffineFrame & frame, bool blockwiseLocalize) const
	{
		RowVectorX Y(GetFeatureDim<FeatureType>());
		for (size_t j = 0; j < Joints.size(); j++)
		{
			auto jid = Joints[j]->ID();
			auto Yj = Y.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension).transpose();
			FeatureType::Get(Yj, frame[Joints[j]->ID()]);
		}

		if (parent() != nullptr && blockwiseLocalize && FeatureType::EnableBlcokwiseLocalization)
		{
			RowVectorX reference(FeatureType::Dimension);
			FeatureType::Get(reference, frame[parent()->Joints.back()->ID()]);
			Y -= reference.replicate(1, Joints.size());
		}
		return Y;
	}

	template <class FeatureType>
	inline void KinematicBlock::SetFeatureVector(AffineFrame & frame, const RowVectorX & X) const
	{
		for (size_t j = 0; j < Joints.size(); j++)
		{
			auto jid = Joints[j]->ID();
			auto Xj = X.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension);
			FeatureType::Set(frame[jid], Xj);
		}
	}

}