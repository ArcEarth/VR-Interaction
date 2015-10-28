#pragma once
#include "Animations.h"
#include "CCA.h"
#include <vector>
#include <map>
#include "RegressionModel.h"
#include "PcaCcaMap.h"
#include "ArmatureParts.h"
#include "BoneFeatures.h"
#include "ArmaturePartFeatures.h"

namespace Causality
{
	struct TransformPair
	{
		int Jx, Jy;
		std::unique_ptr<IRegression> pRegression;
	};

	class CharacterController;

	class CcaArmatureTransform : virtual public ArmatureTransform
	{
	public:

		std::vector<PcaCcaMap> Maps;

	public:
		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
		{
		}
	};

	namespace ArmaturePartFeatures
	{
		class EndEffectorGblPosQuadratized : public IArmaturePartFeature
		{
		public:
			EndEffectorGblPosQuadratized();
			int GetDimension(_In_ const ArmaturePart& block) override;

			typedef BoneFeatures::QuadraticGblPosFeature BoneFeatureType;
			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override;

			// Inherited via IArmaturePartFeature
			virtual void Set(const ArmaturePart & block, BoneHiracheryFrame & frame, const Eigen::RowVectorXf & feature) override;
		};

		class AllJointRltLclRotLnQuatPcad : public Pcad<RelativeDeformation<AllJoints<BoneFeatures::LclRotLnQuatFeature>>>
		{
		public:
			typedef Pcad<RelativeDeformation<AllJoints<BoneFeatures::LclRotLnQuatFeature>>> BaseType;


		};
	}

	class BlockizedArmatureTransform : virtual public ArmatureTransform
	{
	protected:
		const ShrinkedArmature *pSblocks, *pTblocks;
	public:

		BlockizedArmatureTransform();

		BlockizedArmatureTransform(const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock);

		void SetFrom(const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock);
	};

	class BlockizedCcaArmatureTransform : public CcaArmatureTransform, public BlockizedArmatureTransform
	{
	public:
		std::unique_ptr<IArmaturePartFeature> pInputExtractor, pOutputExtractor;
	public:

		using BlockizedArmatureTransform::BlockizedArmatureTransform;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) const;
	};

	namespace ArmaturePartFeatures
	{
		class PerceptiveVector : public IArmaturePartFeature
		{
		private:
			CharacterController			*m_pController;
		public:
			float						Segma;
			bool						Quadratic;
			bool						Velocity;
		public:

			typedef BoneFeatures::GblPosFeature InputFeatureType;
			typedef CharacterFeature CharacterFeatureType;

			PerceptiveVector(CharacterController& controller);

			virtual int GetDimension(_In_ const ArmaturePart& block) override
			{
				return Quadratic ? 9 : 3;
			}

			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override;

			// Inherited via IArmaturePartFeature
			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const Eigen::RowVectorXf& feature) override;
		};
	}

	class CharacterClipinfo;

	class RBFInterpolationTransform : public BlockizedCcaArmatureTransform
	{
	private:
		CharacterController			*m_pController;

	public:
		RBFInterpolationTransform(gsl::array_view<CharacterClipinfo> clips);

		RBFInterpolationTransform(gsl::array_view<CharacterClipinfo> clips, const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock);

		gsl::array_view<CharacterClipinfo>		Clipinfos;

		mutable std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> pvs;
		std::unique_ptr<IArmaturePartFeature> pDependentBlockFeature;

		void Render(DirectX::CXMVECTOR color, DirectX::CXMMATRIX world);

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) const override;
	};

	// for >=0, to sepecific part
	enum PvInputTypeEnum
	{
		ActiveAndDrivenParts = -2,
		ActiveParts = -3,
		NoInputParts = -1,
	};

	//Part to Part transform
	struct P2PTransform
	{
		int DstIdx, SrcIdx;
		Eigen::MatrixXf HomoMatrix; // homogenians transfrom matrix
	};

	class CharacterController;

	class PartilizedTransform : public BlockizedArmatureTransform
	{
	public:
		std::vector<P2PTransform> ActiveParts;	// Direct controlled by input armature, with stylized IK

		PartilizedTransform(const ShrinkedArmature& sParts, CharacterController & controller);
		using BlockizedArmatureTransform::BlockizedArmatureTransform;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) const override;

		void TransformCtrlHandel(Eigen::RowVectorXf &xf, const Causality::P2PTransform & ctrl) const;

		void GenerateDrivenAccesseryControl();

	public:
		std::vector<P2PTransform> DrivenParts;	// These parts will be drive by active parts on Cca base than stylized IK
		std::vector<P2PTransform> AccesseryParts; // These parts will be animated based on active parts (and driven parts?)
	private:
		typedef std::pair<DirectX::Vector3, DirectX::Vector3> LineSegment;

		CharacterController			*m_pController;
		std::vector<LineSegment>	*m_pHandles;

		const Eigen::RowVectorXf& GetInputVector(int SrcIdx);

		typedef std::shared_ptr<IArmaturePartFeature> FeaturePtr;

		mutable
		FeaturePtr	m_pInputF;
		mutable
		FeaturePtr	m_pActiveF;
		mutable
		FeaturePtr	m_pDrivenF;
		mutable
		FeaturePtr	m_pAccesseryF;
	};
}