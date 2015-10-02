#pragma once
#include "Armature.h"
#include "GaussianProcess.h"
#include "PcaCcaMap.h"
#include "RegressionModel.h"
#include "StylizedIK.h"


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

	class ArmaturePart;

	class IArmaturePartFeature abstract
	{
	public:
		using RowVectorX = Eigen::RowVectorXf;

		//virtual int GetDimension(_In_ const ArmaturePart& block) = 0;

		virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) = 0;

		virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time)
		{
			return Get(block, frame);
		}

		virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) = 0;
	};

	namespace ArmaturePartFeatures
	{
		template <class BoneFeatureType>
		class AllJoints : public IArmaturePartFeature
		{
		public:
			bool EnableLocalization;

			AllJoints(bool enableLocalzation = false)
				: EnableLocalization(enableLocalzation)
			{}

			AllJoints(const AllJoints&) = default;

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				RowVectorX Y(block.Joints.size() * BoneFeatureType::Dimension);
				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID;
					auto Yj = Y.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
					BoneFeatureType::Get(Yj, frame[jid]);
				}

				if (block.parent() != nullptr && EnableLocalization)
				{
					RowVectorX reference(BoneFeatureType::Dimension);
					BoneFeatureType::Get(reference, frame[block.parent()->Joints.back()->ID]);
					Y -= reference.replicate(1, block.Joints.size());
				}
				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) override
			{
				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID;
					auto Xj = feature.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
					BoneFeatureType::Set(frame[jid], Xj);
				}
			}
		};

		// Joints feature with pca
		template <class BoneFeatureType>
		class AllJointsPca : public AllJoints<BoneFeatureType>
		{
			float	PcaCutoff;

			int GetDimension(_In_ const ArmaturePart& block)
			{
				return block.ChainPcaMatrix.cols();
			}

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				RowVectorX Y(block.Joints.size() * BoneFeatureType::Dimension);
				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID;
					auto Yj = Y.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
					BoneFeatureType::Get(Yj, frame[jid]);
				}

				if (block.parent() != nullptr && EnableLocalization)
				{
					RowVectorX reference(BoneFeatureType::Dimension);
					BoneFeatureType::Get(reference, frame[block.parent()->Joints.back()->ID]);
					Y -= reference.replicate(1, block.Joints.size());
				}

				Y = (Y - block.ChainPcaMean) * block.ChainPcaMatrix;

				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) override
			{
				RowVectorX Y = feature * block.ChainPcaMatrix.transpose() + block.ChainPcaMean;

				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID;
					auto Xj = Y.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
					BoneFeatureType::Set(frame[jid], Xj);
				}
			}
		};

		template <class BoneFeatureType>
		class EndEffector : public IArmaturePartFeature
		{
		public:
			bool EnableLocalization;
			float FrameTime;

			EndEffector(bool enableLocalzation = false, float frameTime = 1.0f)
				: EnableLocalization(enableLocalzation), FrameTime(frameTime)
			{}

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				RowVectorX Y(BoneFeatureType::Dimension);

				BoneFeatureType::Get(Y, frame[block.Joints.back()->ID]);

				if (block.parent() != nullptr && EnableLocalization)
				{
					RowVectorX reference(BoneFeatureType::Dimension);
					BoneFeatureType::Get(reference, frame[block.parent()->Joints.back()->ID]);
					Y -= reference;
				}
				return Y;
			}

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) override
			{
				static const auto Dim = BoneFeatureType::Dimension;
				RowVectorX Y(Dim * 2);

				Y.segment<Dim>(0) = Get(block, frame);
				Y.segment<Dim>(Dim) = Y.segment<Dim>(0);
				Y.segment<Dim>(Dim) -= Get(block, last_frame);
				Y.segment<Dim>(Dim) /= frame_time;

				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) override
			{
				assert(!"End Effector only block feature could not be use to set frame");
			}
		};

		template <class BoneFeatureType>
		class ExtentedEndEffector : public IArmaturePartFeature
		{
		};

		template <class PartFeatureType>
		class DeformationFromDefault : protected PartFeatureType
		{
		public:
			using PartFeatureType::PartFeatureType;
			void SetDefaultFrame(const BoneHiracheryFrame& frame)
			{
				m_rframe = m_rlframe = m_dframe = frame;
			}

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				BoneHiracheryFrame::Difference(m_rframe, m_dframe, frame);
				return Get(block, m_rframe);
			}

			virtual RowVectorX Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) override
			{
				BoneHiracheryFrame::Difference(m_rframe, m_dframe, frame);
				return Get(block, m_rframe);
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) override
			{
				//BoneHiracheryFrame::Difference(m_rframe, m_dframe, frame);
				//return Set(block, m_rframe);
			}
		private:
			BoneHiracheryFrame m_dframe, m_rframe, m_rlframe;
		};
	}

	// A Kinematic Block is a one-chain in the kinmatic tree, with additional anyalaze information 
	// usually constructed from shrinking the kinmatic tree
	// Each Block holds the children joints and other structural information
	// It is also common to build Multi-Level-of-Detail Block Layers
	class ArmaturePart : public tree_node<ArmaturePart>
	{
	public:
		int					Index;				// Index for this block, valid only through this layer
		int					LoD;				// Level of detail
		vector<const Joint*>Joints;				// Contained Joints
		//BoneHiracheryFrame			ChainFrame;			// Stores the length data and etc

												// Structural feature
		int					LoG;				// Level of Grounding
		SymetricTypeEnum	SymetricType;		// symmetry type
		float				ExpandThreshold;	// The threshold to expand this part
		int					GroundIdx;

		ArmaturePart*		GroundParent;		// Path to grounded bone
		ArmaturePart*		SymetricPair;		// Path to grounded bone

		ArmaturePart*		LoDParent;		// Parent in LoD term
		vector<ArmaturePart*> LoDChildren;	// Children in LoD term

		int					AccumulatedJointCount; // The count of joints that owned by blocks who have minor index

		// Local Animation Domination properties
		ArmaturePart*		Dominator;
		list<ArmaturePart*>	Slaves;

		// Saliansy properties
		int					ActiveActionCount;
		vector<string>		ActiveActions;
		int					SubActiveActionCount;
		vector<string>		SubActiveActions;

		// Local motion and Principle displacement datas
		Eigen::MatrixXf						X;		// (Ac*F) x d, Muilti-clip local-motion feature matrix, Ac = Active Action Count, d = block feature dimension
		Eigen::MatrixXf						LimitX;
		Eigen::VectorXf						Wxj;	// Jx1 Hirechical weights
		Eigen::VectorXf						Wx;		// dJx1 Weights replicated along dim
		Eigen::MatrixXf						Pd;		// Muilti-clip Principle Displacement

		// feature Pca
		Eigen::Pca<Eigen::MatrixXf>			ChainPca;
		Eigen::RowVectorXf					ChainPcaMean;
		Eigen::MatrixXf						ChainPcaMatrix;

		PcaCcaMap							PdCca; // Muilti-clip deducted driver from Principle displacement to local motion
		Eigen::MatrixXf						PvCorr; // ActiveActionCount x ActiveActionCount, record the pv : pv correlation 
		float								PvDriveScore;

		gaussian_process_regression			PdGpr;
		double								ObrsvVar; // The cannonical value's varience give observation

		StylizedChainIK						PdStyleIk;

		ArmaturePart()
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
			SubActiveActionCount = 0;
		}
		// Motion and geometry feature
		//bool				IsActive;			// Is this feature "Active" in energy?
		//bool				IsStable;			// Is Current state a stable state
		//float				MotionEnergy;		// Motion Energy Level
		//float				PotientialEnergy;	// Potenial Energy Level

		BoundingOrientedBox GetBoundingBox(const BoneHiracheryFrame& frame) const;

		//template <class FeatureType>
		//RowVectorX			GetFeatureVector(const BoneHiracheryFrame& frame, bool blockwiseLocalize = false) const;

		//template <class FeatureType>
		//void				SetFeatureVector(_Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) const;
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

	// Shrink a Kinmatic Chain structure to a ArmaturePart
	ArmaturePart* ShrinkChainToBlock(const Joint* pJoint);


	class ShrinkedArmature
	{
	private:
		std::unique_ptr<ArmaturePart> m_pRoot;
		const IArmature*			m_pArmature;
		std::vector<ArmaturePart*>	m_Parts;
	public:
		typedef std::vector<ArmaturePart*> cache_type;

		void SetArmature(const IArmature& armature);

		void ComputeWeights();

		explicit ShrinkedArmature(const IArmature& armature);

		ShrinkedArmature() = default;

		bool   empty() const { return m_pArmature == nullptr; }
		size_t size() const { return m_Parts.size(); }

		auto begin() { return m_Parts.begin(); }
		auto end() { return m_Parts.end(); }
		auto begin() const { return m_Parts.begin(); }
		auto end() const { return m_Parts.end(); }
		auto rbegin() { return m_Parts.rbegin(); }
		auto rend() { return m_Parts.rend(); }
		auto rbegin() const { return m_Parts.rbegin(); }
		auto rend() const { return m_Parts.rend(); }

		const IArmature& Armature() const { return *m_pArmature; }
		ArmaturePart* Root() { return m_pRoot.get(); }
		const ArmaturePart* Root() const { return m_pRoot.get(); }
		ArmaturePart* operator[](int id) { return m_Parts[id]; }
		const ArmaturePart* operator[](int id) const { return m_Parts[id]; }

		// the matrix which alters joints into 
		Eigen::PermutationMatrix<Eigen::Dynamic> GetJointPermutationMatrix(size_t feature_dim) const;
	};

	//template <class FeatureType>
	//inline RowVectorX ArmaturePart::GetFeatureVector(const BoneHiracheryFrame & frame, bool blockwiseLocalize) const
	//{
	//	RowVectorX Y(GetFeatureDim<FeatureType>());
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID;
	//		auto Yj = Y.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension).transpose();
	//		FeatureType::Get(Yj, frame[Joints[j]->ID]);
	//	}

	//	if (parent() != nullptr && blockwiseLocalize && FeatureType::BlockwiseLocalize)
	//	{
	//		RowVectorX reference(FeatureType::Dimension);
	//		FeatureType::Get(reference, frame[parent()->Joints.back()->ID]);
	//		Y -= reference.replicate(1, Joints.size());
	//	}
	//	return Y;
	//}

	//template <class FeatureType>
	//inline void ArmaturePart::SetFeatureVector(BoneHiracheryFrame & frame, const RowVectorX & X) const
	//{
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID;
	//		auto Xj = X.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension);
	//		FeatureType::Set(frame[jid], Xj);
	//	}
	//}
}