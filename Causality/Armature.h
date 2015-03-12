#pragma once
#include "Common\tree.h"
#include "Common\DirectXMathExtend.h"
#include "Common\BCL.h"
#include <unordered_map>
#include <memory>
#include "Common\CompositeFlag.h"

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
		DirectX::Quaternion LocalOrientation;

		XM_ALIGN16
			DirectX::Vector3	Scaling; // Local Scaling , adjust this transform to adjust bone length

										 // Global Data (dulplicate with Local)
		DirectX::Quaternion GlobalOrientation;

		// Global Position for the begining joint of this bone
		XM_ALIGN16
			DirectX::Vector3	OriginPosition;

		XM_ALIGN16
			DirectX::Vector3	EndPostion;
		//DirectX::XMVECTOR EndJointPosition() const;
		bool DirtyFlag;

		void GetBoundingBox(DirectX::BoundingBox& out) const;
		// Update from Hirachy or Global
		// FK Caculation
		void UpdateGlobalData(const BoneDisplacement& refernece);
		// Easy-IK Caculation with No-Yaw-Rotation-Constraint and bone length will be modified if the transform is not isometric
		void UpdateLocalDataByPositionOnly(const BoneDisplacement& reference);
		// Assuming Global position & orientation is known
		void UpdateLocalData(const BoneDisplacement& reference);
	public:
		// Static helper methods for caculate transform matrix
		// Caculate the Transform Matrix from "FromState" to "ToState"
		static DirectX::XMMATRIX TransformMatrix(const BoneDisplacement& from, const BoneDisplacement& to);
		// Ingnore the Scaling transform, have better performence than TransformMatrix
		static DirectX::XMMATRIX RigidTransformMatrix(const BoneDisplacement& from, const BoneDisplacement& to);
		// Ingnore the Scaling transform, may be useful in skinning with Dual-Quaternion
		static DirectX::XMDUALVECTOR RigidTransformDualQuaternion(const BoneDisplacement& from, const BoneDisplacement& to);
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
	};

	using JointSemanticProperty = CompositeFlag<JointSemantic>;
	// A Joint descript the structure information of a joint
	// Also represent the "bone" "end" with it
	// the state information could be retrived by using it's ID
	struct JointData
	{
		// The Index for this joint (&it's the bone ending with it)
		// The Name for this joint
		int							ID;
		// Should be -1 for Root
		int							ParentID;
		std::string					Name;

		JointSemanticProperty		Semantic;
		RotationConstriant			RotationConstraint;
	};

	class Joint : public stdx::tree_node<Joint, false>, protected JointData
	{
	public:
		Joint()
		{
			JointData::ID = -1;
			JointData::ParentID = -1;
		}

		Joint(int id)
		{
			JointData::ID = id;
			JointData::ParentID = -1;
		}

		Joint(const JointData& data)
			: JointData(data)
		{
		}

		int ID() const
		{
			return JointData::ID;
		}
		void SetID(int idx) { JointData::ID = idx; }

		// will be -1 for root
		int ParentID() const
		{
			auto p = parent();
			if (p)
				return parent()->ID();
			else
				return -1;
		}

		const std::string& Name() const
		{
			return JointData::Name;
		}

		void Rename(const std::string& name);

		const JointSemanticProperty& Semantics() const;
		JointSemanticProperty& Semantics();

		const RotationConstriant&	RotationConstraint() const;
		void SetRotationConstraint(const RotationConstriant&);
	};

	typedef double TimeScalarType;

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
		typedef public std::vector<BoneDisplacement, DirectX::AlignedAllocator<BoneDisplacement>> BaseType;
		using BaseType::operator[];
		using BaseType::operator=;

		//const IArmature& Armature() const;
		//// Which skeleton this state fram apply for
		//IArmature* pArmature;

		bool RebuildGlobal(const IArmature& armature);
		bool RebuildLocal(const IArmature& armature);

		Eigen::VectorXf LocalRotationVector() const;
		void UpdateFromLocalRotationVector(const IArmature& armature,const Eigen::VectorXf fv);

		// Interpolate the local-rotation and scaling, "interpolate in Time"
		static void Interpolate(BoneDisplacementFrame& out, const BoneDisplacementFrame &lhs, const BoneDisplacementFrame &rhs, float t, const IArmature& armature);

		// Blend Two Animation Frame, "Interpolate in Space"
		static void Blend(BoneDisplacementFrame& out, const BoneDisplacementFrame &lhs, const BoneDisplacementFrame &rhs, float* blend_weights, const IArmature& armature);

		//void BlendMatrixFrom(DirectX::XMFLOAT3X4* pOut, const StateFrame &from)
		//{

		//}
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
		std::vector<FrameType> KeyFrames;

		std::shared_ptr<FrameType> m_pDefaultFrame; // Use to generate
		const FrameType& DefaultFrame() const;

		// A function map : (BeginTime,EndTime) -> (BeginTime,EndTime),  that handels the easing effect between frames
		// Restriction : TimeWarp(KeyFrameTime(i)) must equals to it self
		std::function<TimeScalarType(TimeScalarType)> TimeWarpFunction;

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
		bool GetFrameAt(FrameType& outFrame, TimeScalarType time) const;

	};

	class ArmatureKeyframeAnimation : public KeyframeAnimation<BoneDisplacementFrame>
	{
	public:
		typedef BoneDisplacementFrame frame_type;
	
		const IArmature& Armature() const;

		// get the pre-computed frame buffer which contains interpolated frame
		const std::vector<frame_type>& get_frame_buffer() const;

		bool pre_interpolate_frames(double frameRate);

	private:
		IArmature* pArmature;
		std::vector<frame_type> frames;
	};

	class ArmatureTransform
	{
	public:
		typedef BoneDisplacementFrame frame_type;
		
		const IArmature& SourceArmature() const;
		const IArmature& TargetArmature() const;

		int GetBindIndex(int sourceIdx) const;
		int GetInverseBindIndex(int tragetIdx) const;

		void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const
		{
			auto lr = source_frame.LocalRotationVector();
			lr = (lr * transform_matrix).transpose();
			target_frame.UpdateFromLocalRotationVector(TargetArmature(),lr);
		}

		void TransformBack(_Out_ frame_type& source_frame, _In_ const frame_type& target_frame) const;

		Eigen::MatrixXf InternalMatrix();
		const Eigen::MatrixXf InternalMatrix() const;

	protected:
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
	class AnimationSpace : protected std::map<std::string, ArmatureKeyframeAnimation>
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

		void RegisterAnimation(const std::string&, animation_type&& animation);

		bool Contains(const std::string& name) const;

		const IArmature&		Armature() const;

		const frame_type&		RestFrame() const; // RestFrame should be the first fram in Rest Animation

		// Using Local Position is bad : Yaw rotation of Parent Joint is not considerd
		Eigen::VectorXf CaculateFrameFeatureVectorEndPointNormalized(const frame_type& frame) const;

		// Basiclly, the difference's magnitude is acceptable, direction is bad
		Eigen::VectorXf CaculateFrameFeatureVectorLnQuaternion(const frame_type& frame) const;

		// Evaluating a likelihood of the given frame is inside this space
		float PoseDistancePCAProjection(const frame_type& frame) const;
		Eigen::RowVectorXf PoseDistanceNearestNeibor(const frame_type& frame) const;

		Eigen::VectorXf CaculateFrameDynamicFeatureVectorJointVelocityHistogram(const frame_type& frame, const frame_type& prev_frame) const;
		Eigen::RowVectorXf DynamicSimiliarityJvh(const frame_type& frame, const frame_type& prev_frame) const;

		float FrameLikilihood(const frame_type& frame, const frame_type& prev_frame) const;
		float FrameLikilihood(const frame_type& frame) const;

		//float DynamicSimiliarityJvh(const velocity_frame_type& velocity_frame) const;
	protected:
		void CaculateXpInv();

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
		virtual ~IArmature() {}

		virtual Joint* at(int index) = 0;
		virtual Joint* root() = 0;
		virtual size_t size() const = 0;

		// We say one Skeleton is compatiable with another Skeleton rhs
		// if and only if rhs is a "base tree" of "this" in the sense of "edge shrink"
		//bool IsCompatiableWith(IArmature* rhs) const;

		const Joint* at(int index) const
		{
			return const_cast<Joint*>(const_cast<IArmature*>(this)->at(index));
		}

		const Joint* root() const
		{
			return const_cast<IArmature*>(this)->root();
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

		template <template<class T> class Range>
		StaticArmature(Range<JointData> data)
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

		StaticArmature(size_t JointCount, int *JointsParentIndices);
		~StaticArmature();
		//void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);
		virtual Joint* at(int index) override;
		virtual Joint* root() override;
		virtual size_t size() const override;

		// A topolical ordered joint index sequence
		const std::vector<size_t>& joint_indices() const
		{
			return TopologyOrder;
		}


		using IArmature::operator[];
	protected:
		void CaculateTopologyOrder();

	private:
		size_t						RootIdx;
		std::vector<joint_type>		Joints;
		std::vector<size_t>			TopologyOrder;
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
	inline bool KeyframeAnimation<FrameType>::PreInterpolateFrames(double frameRate)
	{
		return false;
	}

}