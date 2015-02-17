#pragma once
#include "Common\tree.h"
#include "Common\DirectXMathExtend.h"
#include <unordered_map>
#include <memory>


namespace Kinematics
{

	class BoneState
	{
	public:
		// Local Data
		DirectX::Quaternion LocalOrientation;
		DirectX::Vector3	Scaling; // Local Scaling , adjust this transform to adjust bone length
		// Global Data (dulplicate with Local)
		DirectX::Quaternion GlobalOrientation;
		// Global Position for the begining joint of this bone
		DirectX::Vector3	OriginPosition;
		DirectX::Vector3	EndPostion;
		//DirectX::XMVECTOR EndJointPosition() const;

		// FK Caculation
		void UpdateGlobalData(const BoneState& refernece);
		// Easy-IK Caculation with No-Yaw-Rotation-Constraint and bone length will be modified if the transform is not isometric
		void SetGlobalPosition(const BoneState& reference);

		// Caculate the Transform Matrix from "FromState" to "ToState"
		static DirectX::XMMATRIX TransformMatrix(const BoneState& from, const BoneState& to);
		// Ingnore the Scaling transform, have better performence than TransformMatrix
		static DirectX::XMMATRIX RigidTransformMatrix(const BoneState& from, const BoneState& to);
		// Ingnore the Scaling transform, may be useful in skinning with Dual-Quaternion
		static DirectX::XMDUALVECTOR RigidTransformDualQuaternion(const BoneState& from, const BoneState& to);
	};

	enum JointConstriantType
	{
		JointConstraint_None,
		JointConstraint_Hinge,
	};

	// A Joint descript the structure information of a joint, the state information could be retrived by using it's ID
	class Joint : public stree::tree_node<Joint,false>
	{
	public:
		int ID; // The Index for this joint (&it's the bone ending with it)
		std::string Name;
		JointConstriantType ConstraintType;

		// will be -1 for root
		int ParentID() const
		{
			auto p = parent();
			if (p)
				return parent()->ID;
			else 
				return -1;
		}
	};

	class StateFrame : std::vector<BoneState>
	{
	public:
		typedef std::vector<BoneState> BaseType;
		using BaseType::operator[];
		using BaseType::operator=;
		std::string Name;
		// Which skeleton this state fram apply for
		SkeletonBase* Skeleton;

		// Interpolate the local-rotation and scaling
		void CreateFromInterpolate(const StateFrame &lhs, const StateFrame &rhs, float t)
		{
			assert((Skeleton == lhs.Skeleton) && (lhs.Skeleton == rhs.Skeleton));
			for (size_t i = 0; i < lhs.size(); i++)
			{
				(*this)[i].LocalOrientation = DirectX::XMQuaternionSlerp(lhs[i].LocalOrientation, rhs[i].LocalOrientation, t);
				(*this)[i].Scaling = DirectX::XMVectorLerp(lhs[i].Scaling, rhs[i].Scaling, t);
				if (Skeleton->ParentsMap[i] >= 0)
				{
					(*this)[i].UpdateGlobalData((*this)[Skeleton->ParentsMap[i]]);
				}
			}
			//for (auto& joint : Skeleton->Root()->descendants())
			//{
			//	auto pParent = joint.parent();
			//	if (pParent)
			//		(*this)[joint.ID].UpdateGlobalData((*this)[pParent->ID]);
			//}
		}

		//void BlendMatrixFrom(DirectX::XMFLOAT3X4* pOut, const StateFrame &from)
		//{

		//}
	};

	template <typename FrameType, typename TimeScalarType = float>
	class KeyframeAnimation
	{
	public:
		std::vector<FrameType> KeyFrames;

		// Frame operations
		bool InsertKeyFrame(TimeScalarType time, const FrameType& frame);
		bool InsertKeyFrame(TimeScalarType time, FrameType&& frame);
		bool InsertKeyFrame(int idx, const FrameType& frame);
		bool InsertKeyFrame(int idx, FrameType&& frame);
		bool RemoveKeyFrame(int idx);
		const FrameType& GetKeyFrame(int idx) const;
		bool ReplaceKeyFrame(int idx, const FrameType& frame);
		bool Clear();

		// Time seeking
		float CurrentTime() const;
		float AdvanceTimeBy(TimeScalarType time);
		float SeekTimeTo(TimeScalarType time);

		// Frame Retrival
		const FrameType& CurrentFrame() const;
		bool GetFrameAt(FrameType& outFrame, float time) const;

		// Optimization
		bool PreInterpolateFrames(float frameRate);
	};

	class SkeletonBase
	{
	public:
		virtual ~SkeletonBase() {}
		virtual Joint* at(int index) = 0;
		const Joint* at(int index) const
		{
			return const_cast<Joint*>(const_cast<SkeletonBase*>(this)->at(index));
		}
		Joint* operator[](int idx)
		{
			return at(idx);
		}
		const Joint* operator[](int idx) const
		{
			return at(idx);
		}
		virtual Joint* Root() = 0;
		Joint* Root() const
		{
			return const_cast<Joint*>(const_cast<SkeletonBase*>(this)->Root());
		}
		StateFrame RestFrame;
		std::vector<int> ParentsMap;
	};

	// The Skeleton which won't change it's structure in runtime
	class StaticSkeleton : public SkeletonBase
	{
	public:
		StaticSkeleton(size_t JointCount, int *JointsParentIndices);
		void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);
		virtual Joint* at(int index) override {
			return &Joints[index];
		}
		using SkeletonBase::operator[];

	private:
		std::vector<Joint> Joints;
	};

	class DynamicSkeleton
	{
		void CopyFrom(SkeletonBase* pSkeleton);

		void ChangeRoot(Joint* pNewRoot);
		void RemoveJoint(unsigned int jointID);
		void RemoveJoint(Joint* pJoint);
		void AppendJoint(Joint* pTargetJoint, Joint* pSrcJoint);

		std::map<int, int> AppendSkeleton(Joint* pTargetJoint, DynamicSkeleton* pSkeleton, bool IsCoordinateRelative = false);

		//  This method adjust Joint Index automaticly
		// Anyway , lets just return the value since we have Rvalue && move sementic now
		std::map<int, int> OptimizeIndex();

	public:
		std::unordered_map<int, Joint*> Index;
		typedef std::pair<int, Joint*> Index_Item;

	protected:
		Joint* _root;
	};
}