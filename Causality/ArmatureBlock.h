#pragma once
#include "Armature.h"

namespace Causality
{

	template <class _TBoneFeature>
	struct BlockEndEffectorOnly;
	template <class _TBoneFeature>
	struct BlockLocalized;


	template <class _TBoneFeature>
	struct BlockEndEffectorOnly : public _TBoneFeature
	{
		typedef _TBoneFeature BoneFeature;
		static const bool EndEffectorOnly = true;
	};

	template <class _TBoneFeature>
	struct BlockLocalized : public _TBoneFeature
	{
		typedef _TBoneFeature BoneFeature;
		static const bool BlockwiseLocalize = true;
	};

	class KinematicBlock;

	class BlockFeatureExtractor abstract
	{
	public:
		virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) = 0;
		virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame, _In_ const AffineFrame& last_frame, float frame_time)
		{
			return Get(block, frame);
		}

		virtual void Set(_In_ const KinematicBlock& block, _Out_ AffineFrame& frame, _In_ const RowVectorX& feature) = 0;
	};

	template <class BoneFeatureType>
	class BlockAllJointsFeatureExtractor : public BlockFeatureExtractor
	{
	public :
		bool EnableLocalization;

		BlockAllJointsFeatureExtractor(bool enableLocalzation = false)
			: EnableLocalization(enableLocalzation)
		{}

		BlockAllJointsFeatureExtractor(const BlockAllJointsFeatureExtractor&) = default;

		virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override
		{
			RowVectorX Y(block.Joints.size() * BoneFeatureType::Dimension);
			for (size_t j = 0; j < block.Joints.size(); j++)
			{
				auto jid = block.Joints[j]->ID();
				auto Yj = Y.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
				BoneFeatureType::Get(Yj, frame[jid]);
			}

			if (block.parent() != nullptr && EnableLocalization)
			{
				RowVectorX reference(BoneFeatureType::Dimension);
				BoneFeatureType::Get(reference, frame[block.parent()->Joints.back()->ID()]);
				Y -= reference.replicate(1, block.Joints.size());
			}
			return Y;
		}

		virtual void Set(_In_ const KinematicBlock& block, _Out_ AffineFrame& frame, _In_ const RowVectorX& feature) override
		{
			for (size_t j = 0; j < block.Joints.size(); j++)
			{
				auto jid = block.Joints[j]->ID();
				auto Xj = feature.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
				BoneFeatureType::Set(frame[jid], Xj);
			}
		}
	};

	template <class BoneFeatureType>
	class BlockEndEffectorFeatureExtractor : public BlockFeatureExtractor
	{
	public:
		bool EnableLocalization;
		float FrameTime;

		BlockEndEffectorFeatureExtractor(bool enableLocalzation = false, float frameTime = 1.0f)
			: EnableLocalization(enableLocalzation), FrameTime(frameTime)
		{}

		virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override
		{
			RowVectorX Y(BoneFeatureType::Dimension);

			BoneFeatureType::Get(Y, frame[block.Joints.back()->ID()]);

			if (block.parent() != nullptr && EnableLocalization)
			{
				RowVectorX reference(BoneFeatureType::Dimension);
				BoneFeatureType::Get(reference, frame[block.parent()->Joints.back()->ID()]);
				Y -= reference;
			}
			return Y;
		}

		virtual RowVectorX Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame, _In_ const AffineFrame& last_frame, float frame_time) override
		{
			static const auto Dim = BoneFeatureType::Dimension;
			RowVectorX Y(Dim * 2);

			Y.segment<Dim>(0) = Get(block, frame);
			Y.segment<Dim>(Dim) = Y.segment<Dim>(0);
			Y.segment<Dim>(Dim) -= Get(block, last_frame);
			Y.segment<Dim>(Dim) /= frame_time;

			return Y;
		}

		virtual void Set(_In_ const KinematicBlock& block, _Out_ AffineFrame& frame, _In_ const RowVectorX& feature) override
		{
			assert(false, "End Effector only block feature could not be use to set frame");
		}
	};


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
		vector<string>		ActiveActions;

		// Local motion and Principle displacement datas
		Eigen::MatrixXf						X;		// (Ac*F) x d, Muilti-clip local-motion feature matrix, Ac = Active Action Count, d = block feature dimension
		Eigen::MatrixXf						Pd;		// Muilti-clip Principle Displacement

		Eigen::PcaCcaMap					PdDriver; // Muilti-clip deducted driver from Principle displacement to local motion
		Eigen::MatrixXf						PvCorr; // ActiveActionCount x ActiveActionCount, record the pv : pv correlation 

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

		//template <class FeatureType>
		//RowVectorX			GetFeatureVector(const AffineFrame& frame, bool blockwiseLocalize = false) const;

		//template <class FeatureType>
		//void				SetFeatureVector(_Out_ AffineFrame& frame, _In_ const RowVectorX& feature) const;
		//template <class FeatureType>
		//size_t				GetFeatureDim() const {
		//	return FeatureType::Dimension * Joints.size();
		//}

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

	//template <class FeatureType>
	//inline RowVectorX KinematicBlock::GetFeatureVector(const AffineFrame & frame, bool blockwiseLocalize) const
	//{
	//	RowVectorX Y(GetFeatureDim<FeatureType>());
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID();
	//		auto Yj = Y.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension).transpose();
	//		FeatureType::Get(Yj, frame[Joints[j]->ID()]);
	//	}

	//	if (parent() != nullptr && blockwiseLocalize && FeatureType::BlockwiseLocalize)
	//	{
	//		RowVectorX reference(FeatureType::Dimension);
	//		FeatureType::Get(reference, frame[parent()->Joints.back()->ID()]);
	//		Y -= reference.replicate(1, Joints.size());
	//	}
	//	return Y;
	//}

	//template <class FeatureType>
	//inline void KinematicBlock::SetFeatureVector(AffineFrame & frame, const RowVectorX & X) const
	//{
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID();
	//		auto Xj = X.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension);
	//		FeatureType::Set(frame[jid], Xj);
	//	}
	//}
}