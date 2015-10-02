#pragma once
#include "Animations.h"
#include "CCA.h"
#include <vector>
#include <map>
#include "RegressionModel.h"
#include "PcaCcaMap.h"
#include "ArmatureBlock.h"
#include "BoneFeatures.h"

namespace Causality
{
	struct TransformPair
	{
		int Jx, Jy;
		std::unique_ptr<IRegression> pRegression;
	};

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

			typedef BoneFeatures::QuadraticGblPosFeature BoneFeatureType;
			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override;

			// Inherited via IArmaturePartFeature
			virtual void Set(const ArmaturePart & block, BoneHiracheryFrame & frame, const Eigen::RowVectorXf & feature) override;
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

	class ClipInfo;

	namespace ArmaturePartFeatures
	{
		class PerceptiveVector : public IArmaturePartFeature
		{
			std::map<std::string, ClipInfo*>*		pClipInfos;

		public:
			float						segma;
			bool						QuadraticInput;

			PerceptiveVector(std::map<std::string, ClipInfo*>* pClips);
		public:

			typedef BoneFeatures::GblPosFeature InputFeatureType;
			typedef CharacterFeature CharacterFeatureType;
			virtual Eigen::RowVectorXf Get(_In_ const ArmaturePart& block, _In_ const BoneHiracheryFrame& frame) override;

			// Inherited via IArmaturePartFeature
			virtual void Set(_In_ const ArmaturePart& block, _Out_ BoneHiracheryFrame& frame, _In_ const RowVectorX& feature) override;
		};
	}

	class RBFInterpolationTransform : public BlockizedCcaArmatureTransform
	{

	public:
		RBFInterpolationTransform(std::map<std::string, ClipInfo*>* pClips);

		RBFInterpolationTransform(std::map<std::string, ClipInfo*>* pClips, const ShrinkedArmature * pSourceBlock, const ShrinkedArmature * pTargetBlock);

		std::map<std::string, ClipInfo*>*		pClipInfoMap;

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

	struct BlockPvControl
	{
		int DstIdx, SrcIdx;
		MatrixX Transform; // homogenians transfrom matrix
	};

	class PartilizedTransform : public BlockizedArmatureTransform
	{
	public:
		std::vector<BlockPvControl> ActiveParts;	// Direct controlled by input armature, with stylized IK
		std::vector<BlockPvControl> DrivenParts;	// These parts will be drive by active parts on Cca base than stylized IK
		std::vector<BlockPvControl> AccesseryParts; // These parts will be animated based on active parts (and driven parts?)

		PartilizedTransform();

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const BoneHiracheryFrame& last_frame, float frame_time) const override;

	private:
		std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> * pHandles;

		std::unique_ptr<IArmaturePartFeature>	m_pInputF;
		std::unique_ptr<IArmaturePartFeature>	m_pActiveF;
		std::unique_ptr<IArmaturePartFeature>	m_pDrivenF;
		std::unique_ptr<IArmaturePartFeature>	m_pAccesseryF;

		mutable Eigen::MatrixXd m_Xs;
	};
}