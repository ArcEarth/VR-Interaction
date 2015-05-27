#pragma once
#include "BCL.h"
#include <unordered_map>
#include <memory>

namespace Causality
{
	class IArmature;

	// Pure pose and Dynamic data for a bone
	// "Structure" information is not stored here

	XM_ALIGN16
	struct BoneDisplacement
	{
	public:
		// Local Data
		XM_ALIGN16
		DirectX::Quaternion LclRotation;

		XM_ALIGN16
		DirectX::Vector3	LclScaling; // Local Scaling , adjust this transform to adjust bone length

		// Local Translation, represent the offset vector (Pe - Po) in current frame 
		// Typical value should always be (0,l,0), where l = length of the bone
		// Should be constant among time!!!
		XM_ALIGN16
		DirectX::Vector3	LclTranslation; 

		// Global Data (dulplicate with Local)
		// Global Rotation
		XM_ALIGN16
		DirectX::Quaternion GblRotation;
		XM_ALIGN16
		DirectX::Vector3	GblScaling;
		// Global Position for the begining joint of this bone
		// Aka : GblTranslation
		XM_ALIGN16
		DirectX::Vector3	OriginPosition;
		XM_ALIGN16
		DirectX::Vector3	EndPostion;
		//DirectX::XMVECTOR EndJointPosition() const;
		//bool DirtyFlag;

		void GetBoundingBox(DirectX::BoundingBox& out) const;
		// Update from Hirachy or Global
		// FK Caculation
		// Need this.LclRotation, this.LclScaling, and all Data for reference
		void UpdateGlobalData(const BoneDisplacement& refernece);
		// Easy-IK Caculation with No-X-Rotation-Constraint and bone length will be modified if the transform is not isometric
		void UpdateLocalDataByPositionOnly(const BoneDisplacement& reference);
		// Assuming Global position & orientation is known
		void UpdateLocalData(const BoneDisplacement& reference);
	public:
		// Static helper methods for caculate transform matrix
		// Caculate the Transform Matrix from "FromState" to "ToState"
		static DirectX::XMMATRIX TransformMatrix(const BoneDisplacement& from, const BoneDisplacement& to);
		// Ingnore the LclScaling transform, have better performence than TransformMatrix
		static DirectX::XMMATRIX RigidTransformMatrix(const BoneDisplacement& from, const BoneDisplacement& to);
		// Ingnore the LclScaling transform, may be useful in skinning with Dual-Quaternion
		static DirectX::XMDUALVECTOR RigidTransformDualQuaternion(const BoneDisplacement& from, const BoneDisplacement& to);

	public:
		// number of float elements per bone
		static const auto BoneWidth = 28;

		typedef Eigen::Map<Eigen::Matrix<float, BoneWidth, 1>, Eigen::Aligned> EigenType;
		EigenType AsEigenType()
		{
			return EigenType(&LclRotation.x);
		}
	};

	XM_ALIGN16
	struct BoneVelocity
	{
		XM_ALIGN16
		DirectX::Vector3 LinearVelocity;
		XM_ALIGN16
		DirectX::Vector3 AngelarVelocity;
	};

	enum RotationConstriantType
	{
		Constraint_None = 0x0,
		Constraint_Hinge = 0x1,
		Constraint_Ball = 0x2,
	};

	struct RotationConstriant
	{
		RotationConstriantType Type;
		DirectX::Vector3 UpperRotationBound;
		DirectX::Vector3 LowerRotationBound;
	};

	enum JointSemantic
	{
		Semantic_None = 0x0,
		Semantic_Hand = 0x1,
		Semantic_Wing = 0x2,
		Semantic_Foot = 0x4,
		Semantic_Tail = 0x8,
		Semantic_Mouse = 0x10,
		Semantic_Head = 0x20,
		Semantic_Eye = 0x40,
		Semantic_Nouse = 0x80,
		Semantic_Ear = 0x100,

		Semantic_Left = 0x1000000,
		Semantic_Right = 0x2000000,
		Semantic_Center = 0x4000000,
	};

	using JointSemanticProperty = CompositeFlag<JointSemantic>;
	// A Joint descript the structure information of a joint
	// Also represent the "bone" "end" with it
	// the state information could be retrived by using it's ID
	struct JointBasicData
	{
		// The Index for this joint (&it's the bone ending with it)
		// The Name for this joint
		int							ID;
		// Should be -1 for Root
		int							ParentID;
		std::string					Name;
	};

	struct ColorHistogram
	{
	};

	class Joint : public stdx::tree_node<Joint, false>, protected JointBasicData
	{
	public:
		JointSemanticProperty		Semantic;
		RotationConstriant			RotationConstraint;
		Vector3						Scale;
		Vector3						OffsetFromParent;

		// Saliency parameters & Extra Parameters
		Color						AverageColor;
		ColorHistogram				ColorDistribution;
		Vector3						PositionDistribution;
		BoundingBox					VelocityDistribution;

		float						IntrinsicSaliency;
		float						ExtrinsicSaliency;

	public:
		Joint()
		{
			JointBasicData::ID = -1;
			JointBasicData::ParentID = -1;
		}

		Joint(int id)
		{
			JointBasicData::ID = id;
			JointBasicData::ParentID = -1;
		}

		Joint(const JointBasicData& data)
			: JointBasicData(data)
		{
		}

		int ID() const
		{
			return JointBasicData::ID;
		}
		void SetID(int idx) { JointBasicData::ID = idx; }

		// will be -1 for root
		int ParentID() const
		{
			return JointBasicData::ParentID;
			//auto p = parent();
			//if (p)
			//	return parent()->ID();
			//else
			//	return -1;
		}

		const std::string& Name() const
		{
			return JointBasicData::Name;
		}

		void SetName(const std::string& name) { JointBasicData::Name = name; }
		void SetParentID(int id) { JointBasicData::ParentID = id; }

		const JointSemanticProperty& AssignSemanticsBasedOnName();

		//const JointSemanticProperty& Semantics() const;
		//JointSemanticProperty& Semantics();

		//const RotationConstriant&	RotationConstraint() const;
		//void SetRotationConstraint(const RotationConstriant&);
	};

	typedef time_seconds TimeScalarType;

	struct AnimationFrame // Meta data for an animation frame
	{
	public:
		TimeScalarType	Time;
		std::string		Name;
		bool			IsKeyframe;

		//concept static Interpolate(self&out, const self& lhs, const self& rhs, float t);

		//concept frame_type& operator[]
	};

	class BoneDisplacementFrame : public AnimationFrame, public std::vector<BoneDisplacement,DirectX::AlignedAllocator<BoneDisplacement>>
	{
	public:
		typedef BoneDisplacementFrame self_type;

		typedef public std::vector<BoneDisplacement, DirectX::AlignedAllocator<BoneDisplacement>> BaseType;
		using BaseType::operator[];
		//using BaseType::operator=;

		BoneDisplacementFrame() = default;
		explicit BoneDisplacementFrame(size_t size);
		// copy from default frame
		explicit BoneDisplacementFrame(const IArmature& armature);
		BoneDisplacementFrame(const BoneDisplacementFrame&) = default;
		BoneDisplacementFrame(BoneDisplacementFrame&& rhs) { *this = std::move(rhs); }
		BoneDisplacementFrame& operator=(const BoneDisplacementFrame&) = default;
		BoneDisplacementFrame& operator=(BoneDisplacementFrame&& rhs)
		{
			BaseType::_Assign_rv(std::move(rhs));
			return *this;
		}

		//const IArmature& Armature() const;
		//// Which skeleton this state fram apply for
		//IArmature* pArmature;

		void RebuildGlobal(const IArmature& armature);
		void RebuildLocal(const IArmature& armature);

		Eigen::VectorXf LocalRotationVector() const;
		void UpdateFromLocalRotationVector(const IArmature& armature,const Eigen::VectorXf fv);

		// Interpolate the local-rotation and scaling, "interpolate in Time"
		static void Interpolate(BoneDisplacementFrame& out, const BoneDisplacementFrame &lhs, const BoneDisplacementFrame &rhs, float t, const IArmature& armature);

		// Blend Two Animation Frame, "Interpolate in Space"
		static void Blend(BoneDisplacementFrame& out, const BoneDisplacementFrame &lhs, const BoneDisplacementFrame &rhs, float* blend_weights, const IArmature& armature);

		static void TransformMatrix(DirectX::XMFLOAT3X4* pOut, const self_type &from, const self_type& to);
		static void TransformMatrix(DirectX::XMFLOAT4X4* pOut, const self_type &from, const self_type& to);
		//void BlendMatrixFrom(DirectX::XMFLOAT3X4* pOut, const StateFrame &from)
		//{

		//}

		// number of float elements per bone
		static const auto BoneWidth = sizeof(BoneDisplacement) / sizeof(float);
		//! not the eigen vector in math !!!
		typedef Eigen::Map<VectorX, Eigen::Aligned> EigenVectorType;

		// return a 20N x 1 column vector of this frame, where 20 = BoneWidth
		EigenVectorType AsEigenVector()
		{
			return EigenVectorType(&(*this)[0].LclRotation.x,size() * BoneWidth);
		}

		typedef Eigen::Map<Eigen::Matrix<float, BoneWidth, Eigen::Dynamic>, Eigen::Aligned> EigenMatrixType;
		// return a 20 x N matrix of this frame
		EigenMatrixType AsEigenMatrix()
		{
			return EigenMatrixType(&(*this)[0].LclRotation.x, BoneWidth,size());
		}
	};

	class BoneVelocityFrame : public std::vector<BoneVelocity>
	{

	};

	class LinearFrame
	{};
	class SpineFrame
	{};
	class DicredFrame
	{};

	class AnimationManager
	{

	};

	class IFrameAnimation
	{

	};

	class LinearWarp
	{
	public:
		TimeScalarType operator()(TimeScalarType t)
		{
			return t;
		}
	};


	// The underline resources of an animation, not for "Play" Control
	// Handles Interpolations and warps in Time
	template <typename FrameType>
	class KeyframeAnimation : public IFrameAnimation
	{
	public:
		string					Name;
		std::vector<FrameType>	KeyFrames;
		TimeScalarType			Duration;
		TimeScalarType			FrameInterval;
		// A function map : (BeginTime,EndTime) -> (BeginTime,EndTime),  that handels the easing effect between frames
		// Restriction : TimeWarp(KeyFrameTime(i)) must equals to it self
		std::function<TimeScalarType(TimeScalarType)> TimeWarpFunction;
	protected:
		FrameType * m_pDefaultFrame; // Use to generate
	public:
		typedef KeyframeAnimation self_type;

		KeyframeAnimation() = default;
		KeyframeAnimation(const self_type& rhs) = default;
		self_type& operator=(const self_type& rhs) = default;

		KeyframeAnimation(self_type&& rhs)
		{
			*this = std::move(rhs);
		}

		inline self_type& operator=(const self_type&& rhs)
		{
			Name = std::move(rhs.Name);
			KeyFrames = std::move(rhs.KeyFrames);
			m_pDefaultFrame = rhs.m_pDefaultFrame;
			return *this;
		}

		const FrameType& DefaultFrame() const;

		// Duration of this clip
		TimeScalarType	Length() const;
		// Is this clip a loopable animation : last frame == first frame
		bool			Loopable()  const;

		// Frame operations
	public:
		bool InsertKeyFrame(TimeScalarType time, const FrameType& frame);
		//bool InsertKeyFrame(TimeScalarType time, FrameType&& frame);
		bool InsertKeyFrame(int idx, const FrameType& frame);
		//bool InsertKeyFrame(int idx, FrameType&& frame);
		bool RemoveKeyFrame(int idx);
		const FrameType& GetKeyFrame(int idx) const;
		bool ReplaceKeyFrame(int idx, const FrameType& frame);
		bool Clear();

	public:
		// Frame Retrival
		virtual bool GetFrameAt(FrameType& outFrame, TimeScalarType time) const;

	};

	class ArmatureKeyframeAnimation : public KeyframeAnimation<BoneDisplacementFrame>
	{
	public:
		typedef ArmatureKeyframeAnimation self_type;
		typedef KeyframeAnimation<BoneDisplacementFrame> base_type;
		typedef BoneDisplacementFrame frame_type;

		self_type() = default;
		self_type(const self_type& rhs) = default;
		self_type& operator=(const self_type& rhs) = default;

		self_type(self_type&& rhs)
		{
			*this = std::move(rhs);
		}
		explicit ArmatureKeyframeAnimation (std::istream& file);

		self_type& operator=(const self_type&& rhs)
		{
			pArmature = rhs.pArmature;
			frames = std::move(rhs.frames);
			base_type::operator=(std::move(rhs));
			return *this;
		}

		const IArmature& Armature() const { return *pArmature; }
		void SetArmature(IArmature& armature) { pArmature = &armature; }
		// get the pre-computed frame buffer which contains interpolated frame
		const std::vector<frame_type>& GetFrameBuffer() const { return frames; }
		std::vector<frame_type>& GetFrameBuffer() { return frames; }

		// Feature Matrix of this animation, const version
		const Eigen::MatrixXf& AnimMatrix() const { return animMatrix; }
		// Feature Matrix of this animation
		Eigen::MatrixXf& AnimMatrix() { return animMatrix; }

		std::vector<Eigen::ColPivHouseholderQR<MatrixXf>> QRs;

		bool InterpolateFrames(double frameRate);
		virtual bool GetFrameAt(BoneDisplacementFrame& outFrame, TimeScalarType time) const;

		enum DataType
		{
			LocalRotation = 0,
			GlobalRotation = 2,
			OriginPosition = 3,
			EndPositon = 4,
		};

		// Eigen interface
		//Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::Stride<sizeof(float), sizeof(BoneDisplacement)>> DataMatrix(DataType data) const
		//{
		//	return Eigen::Map<Eigen::Matrix3Xf, Eigen::Aligned, Eigen::Stride<sizeof(float), sizeof(BoneDisplacement)>>(&frames[0][0]);
		//}

	private:
		IArmature*			pArmature;
		vector<frame_type>	frames;
		Eigen::MatrixXf		animMatrix; // 20N x F matrix
	};

	class ArmatureTransform
	{
	public:
		typedef BoneDisplacementFrame frame_type;
		
		const IArmature& SourceArmature() const { return *pSource; }
		const IArmature& TargetArmature() const { return *pTarget; }
		void SetSourceArmature(const IArmature& armature) { pSource = &armature; }
		void SetTargetArmature(const IArmature& armature) { pTarget = &armature; }

		int GetBindIndex(int sourceIdx) const;
		int GetInverseBindIndex(int tragetIdx) const;

		void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const
		{
			auto lr = source_frame.LocalRotationVector();
			lr = (lr * transform_matrix).transpose();
			target_frame.UpdateFromLocalRotationVector(TargetArmature(),lr);
		}

		void TransformBack(_Out_ frame_type& source_frame, _In_ const frame_type& target_frame) const;

		Eigen::MatrixXf& InternalMatrix();
		const Eigen::MatrixXf& InternalMatrix() const;

	protected:
		const IArmature	*pSource, *pTarget;
		Eigen::MatrixXf transform_matrix;
	};


	template <typename FrameType>
	class RealTimeAnimation
	{
		FrameType *m_pDefaultFrame; // Use to generate
		const FrameType& DefaultFrame() const
		{
			return *m_pDefaultFrame;
		}
		FrameType& CurrentFrame() { return m_CurrentFrame; }
		const FrameType& CurrentFrame() const { return m_CurrentFrame; }
		FrameType m_CurrentFrame;
	};

	class ArmatureRealTimeAnimation : public RealTimeAnimation<BoneDisplacementFrame>
	{
	public:
		typedef BoneDisplacementFrame frame_type;

		const IArmature& Armature() const;

	private:
		IArmature* pArmature;
	};

	// Represent an semantic collection of animations
	// It's the possible pre-defined actions for an given object
	class BehavierSpace : protected std::map<std::string, ArmatureKeyframeAnimation>
	{
	public:
		typedef ArmatureKeyframeAnimation animation_type;
		typedef BoneDisplacementFrame frame_type;
		typedef BoneVelocityFrame velocity_frame_type;
		typedef std::map<std::string, ArmatureKeyframeAnimation> base_type;

		using base_type::begin;
		using base_type::end;
		using base_type::size;

		animation_type& operator[](const std::string& name)
		{
			auto itr = this->find(name);
			if (itr != this->end())
			{
				return itr->second;
			}
			else
			{
				throw std::out_of_range("No animation with specified name exist in this aniamtion space.");
			}
		}
		const animation_type& operator[](const std::string& name) const
		{
			auto itr = this->find(name);
			if (itr != this->end())
			{
				return itr->second;
			}
			else
			{
				throw std::out_of_range("No animation with specified name exist in this aniamtion space.");
			}
		}

		void AddAnimationClip(const std::string& name, animation_type&& animation)
		{
			this->emplace(std::make_pair(name, std::move(animation)));
		}

		animation_type& AddAnimationClip(const std::string& name)
		{
			this->emplace(std::make_pair(name,animation_type()));
			auto& clip = this->at(name);
			clip.SetArmature(*m_pArmature);
			return clip;
		}

		bool Contains(const std::string& name) const;

		const IArmature&		Armature() const;
		IArmature&				Armature();
		void					SetArmature(IArmature& armature);

		const frame_type&		RestFrame() const; // RestFrame should be the first fram in Rest Animation

		// Using Local Position is bad : X rotation of Parent Joint is not considerd
		Eigen::VectorXf			FrameFeatureVectorEndPointNormalized(const frame_type& frame) const;
		Eigen::MatrixXf			AnimationMatrixEndPosition(const ArmatureKeyframeAnimation& animation) const;
		void					CacAnimationMatrixEndPosition( _In_ const ArmatureKeyframeAnimation& animation, _Out_ Eigen::MatrixXf& fmatrix) const;
		// Basiclly, the difference's magnitude is acceptable, direction is bad
		Eigen::VectorXf			FrameFeatureVectorLnQuaternion(const frame_type& frame) const;
		void					CaculateAnimationFeatureLnQuaternionInto(_In_ const ArmatureKeyframeAnimation& animation, _Out_ Eigen::MatrixXf& fmatrix) const;

		// Evaluating a likelihood of the given frame is inside this space
		float					PoseDistancePCAProjection(const frame_type& frame) const;
		Eigen::RowVectorXf		PoseSquareDistanceNearestNeibor(const frame_type& frame) const;
		Eigen::RowVectorXf		StaticSimiliartyEculidDistance(const frame_type& frame) const;

		Eigen::VectorXf			CaculateFrameDynamicFeatureVectorJointVelocityHistogram(const frame_type& frame, const frame_type& prev_frame) const;
		Eigen::RowVectorXf		DynamicSimiliarityJvh(const frame_type& frame, const frame_type& prev_frame) const;

		// with 1st order dynamic
		float FrameLikilihood(const frame_type& frame, const frame_type& prev_frame) const;
		// without dynamic
		float FrameLikilihood(const frame_type& frame) const;

		vector<ArmatureTransform> GenerateBindings();

		//float DynamicSimiliarityJvh(const velocity_frame_type& velocity_frame) const;
	protected:
		void CaculateXpInv();

		IArmature*						m_pArmature;
		float			SegmaDis; // Distribution dervite of pose-distance

		std::vector<DirectX::Vector3> Sv; // Buffer for computing feature vector

		Eigen::VectorXf Wb; // The weights for different "Bones" , default to "Flat"

		Eigen::VectorXf X0; // Feature vector of Rest Frame
		Eigen::MatrixXf X; // Feature vector matrix = [ X1-X0 X2-X0 X3-X0 ... XN-X0]
		Eigen::MatrixXf XpInv; // PersudoInverse of X;
		Eigen::MatrixXf Xv; // Feature vector matrix for frame dyanmic = [ Xv1-Xv0 Xv2-X0 Xv3-Xv0 ... XvN-Xv0]
	};

	enum AnimationPlayState
	{
		Stopped = 0,
		Active = 1,
		Paused = 2,
	};

	enum AnimationRepeatBehavior
	{
		NotRepeat = 0,
		AlwaysRepeat = 1,
	};

	class StoryBoard
	{
	public:
		// Play Control
		void Begin();
		void Stop();
		void Pause();
		void Resume();

		// Play properties
		AnimationRepeatBehavior GetRepeatBehavior() const;
		void SetRepeatBehavior(AnimationRepeatBehavior behavior);

		double GetSpeedRatio() const;
		void SetSpeedRatio(double ratio) const;

		// Timeline basic
		TimeScalarType GetBeginTime() const;
		void SetBeginTime(TimeScalarType time);
		TimeScalarType GetEndTime() const;
		void SetEndTime(TimeScalarType time);
		TimeScalarType GetDuration();

		// Time seeking
		AnimationPlayState GetCurrentState() const;
		TimeScalarType GetCurrentTime() const; // Shit! Name Collision with Windows Header!!! Dame te outdated C-style header!!!
		void AdvanceTimeBy(TimeScalarType time);
		void Seek(TimeScalarType time);

		TypedEvent<StoryBoard> Completed;

		std::vector<IFrameAnimation*> ChildrenAnimation;
	protected:
		AnimationPlayState CurrentState;
	};

	// Represent a model "segmentation" and it's hireachy
	class IArmature
	{
	public:
		typedef BoneDisplacementFrame frame_type;

		virtual ~IArmature() {}

		virtual Joint* at(int index) = 0;
		virtual Joint* root() = 0;
		const Joint* root() const
		{
			return const_cast<IArmature*>(this)->root();
		}
		virtual size_t size() const = 0;
		virtual const frame_type& default_frame() const = 0;

		iterator_range<Joint::const_depth_first_iterator>
			joints() const 
		{
			return root()->nodes();
		}

		iterator_range<Joint::mutable_depth_first_iterator> 
			joints()
		{
			return root()->nodes();
		}

		// We say one Skeleton is compatiable with another Skeleton rhs
		// if and only if rhs is a "base tree" of "this" in the sense of "edge shrink"
		//bool IsCompatiableWith(IArmature* rhs) const;

		const Joint* at(int index) const
		{
			return const_cast<IArmature*>(this)->at(index);
		}



		Joint* operator[](int idx)
		{
			return at(idx);
		}

		const Joint* operator[](int idx) const
		{
			return at(idx);
		}

		//std::vector<size_t> TopologyOrder;
	};

	// The Skeleton which won't change it's structure in runtime
	class StaticArmature : public IArmature
	{
	public:
		typedef Joint joint_type;
		typedef StaticArmature self_type;

	private:
		size_t						RootIdx;
		std::vector<joint_type>		Joints;
		std::vector<size_t>			TopologyOrder;
		frame_type					DefaultFrame;

	public:
		template <template<class T> class Range>
		StaticArmature(Range<JointBasicData> data)
		{
			size_t jointCount = data.size();
			Joints.resize(jointCount);

			for (size_t i = 0; i < jointCount; i++)
			{
				Joints[i].SetID(i);

				int parentID = data[i].ParentID;
				if (parentID != i &&parentID >= 0)
				{
					Joints[parentID].append_children_back(&Joints[i]);
				}
				else
				{
					RootIdx = i;
				}
			}

			CaculateTopologyOrder();
		}

		// deserialization
		StaticArmature(std::istream& file);
		StaticArmature(size_t JointCount, int *JointsParentIndices, const char* const* Names);
		~StaticArmature();
		StaticArmature(const self_type& rhs) = delete;
		StaticArmature(self_type&& rhs);

		self_type& operator=(const self_type& rhs) = delete;
		self_type& operator=(self_type&& rhs)
		{
			this->DefaultFrame = std::move(rhs.DefaultFrame);
			this->Joints = std::move(rhs.Joints);
			this->TopologyOrder = std::move(rhs.TopologyOrder);
			this->RootIdx = rhs.RootIdx;
		}

		//void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);
		virtual joint_type* at(int index) override;
		virtual joint_type* root() override;
		virtual size_t size() const override;
		virtual const frame_type& default_frame() const override;
		frame_type& default_frame() { return DefaultFrame; }

		// A topolical ordered joint index sequence
		const std::vector<size_t>& joint_indices() const
		{
			return TopologyOrder;
		}

		auto joints() const -> decltype(adaptors::transform(TopologyOrder,function<const Joint&(int)>()))
		{
			using namespace boost::adaptors;
			function<const Joint&(int)> func = [this](int idx)->const joint_type& {return Joints[idx]; };
			return transform(TopologyOrder, func);
		}

		auto joints() -> decltype(adaptors::transform(TopologyOrder, function<Joint&(int)>()))
		{
			using namespace boost::adaptors;
			function<Joint&(int)> func = [this](int idx)->joint_type&{ return Joints[idx]; };
			return transform(TopologyOrder, func);
		}


		using IArmature::operator[];
	protected:
		void CaculateTopologyOrder();
	};

	// A armature which fellows a graph structure
	class GraphArmature
	{
	};

	// For future use
	class DynamicArmature : public IArmature
	{
		void CopyFrom(IArmature* pSrc);

		void ChangeRoot(Joint* pNewRoot);
		void RemoveJoint(unsigned int jointID);
		void RemoveJoint(Joint* pJoint);
		void AppendJoint(Joint* pTargetJoint, Joint* pSrcJoint);

		std::map<int, int> AppendSkeleton(Joint* pTargetJoint, DynamicArmature* pSkeleton, bool IsCoordinateRelative = false);

		// This method adjust Joint Index automaticly to DFS order
		// Anyway , lets just return the value since we have Rvalue && move sementic now
		std::map<int, int> Reindex();

	public:
		std::unordered_map<int, Joint*> Index;
		typedef std::pair<int, Joint*> Index_Item;

	protected:
		Joint* _root;
	};

	template<typename FrameType>
	bool KeyframeAnimation<FrameType>::GetFrameAt(FrameType & outFrame, TimeScalarType time) const
	{
		return false;
	}

}