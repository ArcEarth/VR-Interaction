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

	class BlockEndEffectorGblPosQuadratic : public BlockFeatureExtractor
	{
	public:
		BlockEndEffectorGblPosQuadratic();

		typedef BoneFeatures::QuadraticGblPosFeature BoneFeatureType;
		virtual Eigen::RowVectorXf Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override;

		// Inherited via BlockFeatureExtractor
		virtual void Set(const KinematicBlock & block, AffineFrame & frame, const Eigen::RowVectorXf & feature) override;
	};

	class BlockizedArmatureTransform : virtual public ArmatureTransform
	{
	protected:
		const BlockArmature *pSblocks, *pTblocks;
	public:

		BlockizedArmatureTransform();

		BlockizedArmatureTransform(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock);

		void SetFrom(const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock);
	};

	class BlockizedCcaArmatureTransform : public CcaArmatureTransform, public BlockizedArmatureTransform
	{
	public:
		std::unique_ptr<BlockFeatureExtractor> pInputExtractor, pOutputExtractor;
	public:

		using BlockizedArmatureTransform::BlockizedArmatureTransform;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const AffineFrame& last_frame, float frame_time) const;
	};

	class ClipInfo;

	class PerceptiveVectorBlockFeatureExtractor : public BlockFeatureExtractor
	{
		std::map<std::string, ClipInfo*>*		pClipInfos;

	public:
		float						segma;
		bool						QuadraticInput;

		PerceptiveVectorBlockFeatureExtractor(std::map<std::string, ClipInfo*>* pClips);
	public:

		typedef BoneFeatures::GblPosFeature InputFeatureType;
		typedef CharacterFeature CharacterFeatureType;
		virtual Eigen::RowVectorXf Get(_In_ const KinematicBlock& block, _In_ const AffineFrame& frame) override;

		// Inherited via BlockFeatureExtractor
		virtual void Set(_In_ const KinematicBlock& block, _Out_ AffineFrame& frame, _In_ const RowVectorX& feature) override;
	};

	class BlockizedSpatialInterpolateQuadraticTransform : public BlockizedCcaArmatureTransform
	{

	public:
		BlockizedSpatialInterpolateQuadraticTransform(std::map<std::string, ClipInfo*>* pClips);

		BlockizedSpatialInterpolateQuadraticTransform(std::map<std::string, ClipInfo*>* pClips, const BlockArmature * pSourceBlock, const BlockArmature * pTargetBlock);

		std::map<std::string, ClipInfo*>*		pClipInfoMap;

		mutable std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> pvs;
		std::unique_ptr<BlockFeatureExtractor> pDependentBlockFeature;

		void Render(DirectX::CXMVECTOR color, DirectX::CXMMATRIX world);

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame, _In_ const AffineFrame& last_frame, float frame_time) const override;
	};

}