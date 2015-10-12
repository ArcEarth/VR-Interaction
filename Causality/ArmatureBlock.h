#pragma once
#include "Armature.h"
#include "GaussianProcess.h"
#include "PcaCcaMap.h"
#include "RegressionModel.h"
#include "StylizedIK.h"
#include "BoneFeatures.h"

namespace Causality
{
	class ArmaturePart;
	class ShrinkedArmature;

	class IArmaturePartFeature abstract
	{
	public:
		virtual ~IArmaturePartFeature();

		virtual int GetDimension(_In_ const ArmaturePart& block) = 0;

		//void Get(_In_ const ShrinkedArmature& parts, _Out_ Eigen::RowVectorXf& feature, _In_ const BoneHiracheryFrame& frame);

		//virtual void Get(_In_ const ArmaturePart& block, _Out_ Eigen::RowVectorXf& feature, _In_ const BoneHiracheryFrame& frame);

		virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) = 0;

		virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) = 0;

		virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time);
	};

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
		Eigen::DenseIndex					ChainPcadDim;
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
		//Eigen::RowVectorXf			GetFeatureVector(const BoneHiracheryFrame& frame, bool blockwiseLocalize = false) const;

		//template <class FeatureType>
		//void				SetFeatureVector(_Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) const;
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
	//inline Eigen::RowVectorXf ArmaturePart::GetFeatureVector(const BoneHiracheryFrame & frame, bool blockwiseLocalize) const
	//{
	//	Eigen::RowVectorXf Y(GetFeatureDim<FeatureType>());
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID;
	//		auto Yj = Y.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension).transpose();
	//		FeatureType::Get(Yj, frame[Joints[j]->ID]);
	//	}

	//	if (parent() != nullptr && blockwiseLocalize && FeatureType::BlockwiseLocalize)
	//	{
	//		Eigen::RowVectorXf reference(FeatureType::Dimension);
	//		FeatureType::Get(reference, frame[parent()->Joints.back()->ID]);
	//		Y -= reference.replicate(1, Joints.size());
	//	}
	//	return Y;
	//}

	//template <class FeatureType>
	//inline void ArmaturePart::SetFeatureVector(BoneHiracheryFrame & frame, const Eigen::RowVectorXf & X) const
	//{
	//	for (size_t j = 0; j < Joints.size(); j++)
	//	{
	//		auto jid = Joints[j]->ID;
	//		auto Xj = X.middleCols<FeatureType::Dimension>(j * FeatureType::Dimension);
	//		FeatureType::Set(frame[jid], Xj);
	//	}
	//}

	namespace ArmaturePartFeatures
	{
		template <class BoneFeatureType>
		class AllJoints : public IArmaturePartFeature
		{
		public:

			AllJoints() = default;

			AllJoints(const AllJoints&) = default;

			int GetDimension(_In_ const ArmaturePart& block)
			{
				return block.Joints.size() * BoneFeatureType::Dimension;
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				Eigen::RowVectorXf Y(block.Joints.size() * BoneFeatureType::Dimension);
				for (size_t j = 0; j < block.Joints.size(); j++)
				{
					auto jid = block.Joints[j]->ID;
					auto Yj = Y.segment<BoneFeatureType::Dimension>(j * BoneFeatureType::Dimension);
					BoneFeatureType::Get(Yj, frame[jid]);
				}

				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
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
		template <class PartFeatureType>
		class Pcad : public PartFeatureType
		{
		public:
			static_assert(std::is_base_of<IArmaturePartFeature, PartFeatureType>::value, "PartFeatureType must be derived type of IArmaturePartFeature");

			std::vector<Eigen::MatrixXf> m_pcas;
			std::vector<Eigen::RowVectorXf> m_means;

			int GetDimension(_In_ const ArmaturePart& block)
			{
				auto bid = block.Index;
				return m_pcas[bid].cols();
			}

			Eigen::MatrixXf& GetPca(int bid) { return m_pcas[bid]; }
			const Eigen::MatrixXf& GetPca(int bid) const { return m_pcas[bid]; }

			Eigen::RowVectorXf& GetMean(int bid) { return m_means[bid]; }
			const Eigen::RowVectorXf& GetMean(int bid) const { return m_means[bid]; }
			void InitPcas(int size) {
				m_pcas.resize(size);
				m_means.resize(size);
			}
			template <class DerivedPca, class DerivedMean>
			void SetPca(int bid, const Eigen::MatrixBase<DerivedPca>& principleComponents, const Eigen::MatrixBase<DerivedMean>& mean) {
				assert(!m_means.empty() && "Call InitPcas before SetPca");
				m_means[bid] = mean;
				m_pcas[bid] = principleComponents;
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				auto X = PartFeatureType::Get(block, frame);

				auto bid = block.Index;
				return (X - m_means[bid]) * m_pcas[bid];
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
			{
				auto bid = block.Index;
				auto X = (feature * m_pcas[bid].transpose() + m_means[bid]).eval();
				PartFeatureType::Set(block, frame, X);
			}
		};

		template <class PartFeatureType>
		class WithVelocity : public PartFeatureType
		{
		public:
			static_assert(std::is_base_of<IArmaturePartFeature, PartFeatureType>::value, "PartFeatureType must be derived type of IArmaturePartFeature");

			int GetDimension(_In_ const ArmaturePart& block)
			{
				return PartFeatureType::GetDimension(block) * 2;
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) override
			{
				int dim = PartFeatureType::GetDimension(block);
				Eigen::RowVectorXf Y(dim * 2);

				Y.segment(0, dim) = PartFeatureType::Get(block, frame);
				Y.segment(dim, dim) = Y.segment(0, dim);
				Y.segment(dim, dim) -= PartFeatureType::Get(block, last_frame);
				Y.segment(dim, dim) /= frame_time;

				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
			{
				int dim = PartFeatureType::GetDimension(block);
				assert(feature.cols() == dim * 2);
				PartFeatureType::Set(block, frame, feature.segment(0, dim));
			}
		};

		template <class PartFeatureType>
		class Localize : public PartFeatureType
		{
		public:
			static_assert(std::is_base_of<IArmaturePartFeature, PartFeatureType>::value, "PartFeatureType must be derived type of IArmaturePartFeature");

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				Eigen::RowVectorXf Y = PartFeatureType::Get(block, frame);

				if (block.parent() != nullptr)
				{
					Eigen::RowVectorXf ref = PartFeatureType::Get(*block.parent(), frame);
					Y -= ref;
				}
				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
			{
				assert(!"Localize could not be use to set frame");
			}
		};


		template <class BoneFeatureType>
		class EndEffector : public IArmaturePartFeature
		{
		public:
			EndEffector() = default;

			int GetDimension(_In_ const ArmaturePart& block)
			{
				return BoneFeatureType::Dimension;
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				Eigen::RowVectorXf Y(BoneFeatureType::Dimension);

				BoneFeatureType::Get(Y, frame[block.Joints.back()->ID]);

				return Y;
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
			{
				assert(!"End Effector only block feature could not be use to set frame");
			}
		};

		template <class PartFeatureType>
		class RelativeDeformation : public PartFeatureType
		{
		public:
			static_assert(std::is_base_of<IArmaturePartFeature, PartFeatureType>::value, "PartFeatureType must be derived type of IArmaturePartFeature");

			using PartFeatureType::PartFeatureType;

			void SetDefaultFrame(const BoneHiracheryFrame& frame)
			{
				m_rframe = m_rlframe = m_dframe = m_dframeInv = frame;
				for (auto& bone : m_dframeInv)
				{
					bone.LocalTransform().Inverse();
					bone.GlobalTransform().Inverse();
				}
			}

		protected:
			void GetRelativeFrame(const Causality::ArmaturePart & block, const Causality::BoneHiracheryFrame & frame)
			{
				for (auto joint : block.Joints)
				{
					auto i = joint->ID;
					auto& lt = m_rframe[i];
					lt.LocalTransform() = m_dframeInv[i].LocalTransform();
					lt.LocalTransform() *= frame[i].LocalTransform();

					lt.GlobalTransform() = m_dframeInv[i].LocalTransform();
					lt.GlobalTransform() *= frame[i].GlobalTransform();
				}
			}

			void SetAbsoluteFrame(const Causality::ArmaturePart & block, _Out_ Causality::BoneHiracheryFrame & frame)
			{
				for (auto joint : block.Joints)
				{
					auto i = joint->ID;
					auto& lt = frame[i];
					lt.LocalTransform() = m_dframe[i].LocalTransform();
					lt.LocalTransform() *= m_rframe[i].LocalTransform();
				}
			}

		public:
			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override
			{
				GetRelativeFrame(block, frame);
				return PartFeatureType::Get(block, m_rframe);
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) override
			{
				GetRelativeFrame(block, frame);
				return PartFeatureType::Get(block, m_rframe);
			}

			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override
			{
				PartFeatureType::Set(block, m_rframe, feature);
				SetAbsoluteFrame(block, frame);
			}

		private:
			BoneHiracheryFrame m_dframe, m_dframeInv, m_rframe, m_rlframe;
		};
	}


	//inline void IArmaturePartFeature::Get(_In_ const ShrinkedArmature& parts, _Out_ Eigen::RowVectorXf& feature, _In_ const BoneHiracheryFrame& frame)
	//{
	//	int stIdx = 0;
	//	for (int i = 0; i < parts.size(); i++)
	//	{
	//		int dim = GetDimension(*parts[i]);
	//		feature.segment(stIdx, dim) = Get(*parts[i], frame);
	//		stIdx += dim;
	//	}
	//}

}