#include "Armature.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
#include <boost\range\algorithm.hpp>

using namespace DirectX;
using namespace Armatures;

void BoneState::UpdateGlobalData(const BoneState & reference)
{
	XMVECTOR ParQ = reference.GlobalOrientation;
	XMVECTOR Q = XMQuaternionMultiply(ParQ, LocalOrientation);
	GlobalOrientation = Q;
	OriginPosition = reference.EndPostion;
	XMVECTOR V = Scaling;
	V = XMVectorMultiply(V, g_XMIdentityR1.v);
	V = XMVector3Rotate(V, Q);
	V = XMVectorAdd(V, OriginPosition);
	EndPostion = V;
}

void BoneState::UpdateLocalData(const BoneState& reference)
{
	OriginPosition = reference.EndPostion;
	XMVECTOR ParQ = reference.GlobalOrientation;
	ParQ = XMQuaternionInverse(ParQ);
	XMVECTOR Q = GlobalOrientation;
	LocalOrientation = XMQuaternionMultiply(ParQ, Q);
	Q = (XMVECTOR)EndPostion - (XMVECTOR)reference.EndPostion;
	Q = XMVector3Length(Q);
	Q = XMVectorSelect(g_XMIdentityR1.v, Q, g_XMIdentityR1.v);
	Scaling = Q;
}


void BoneState::UpdateLocalDataByPositionOnly(const BoneState & reference)
{
	XMVECTOR v0 = (XMVECTOR)this->OriginPosition - (XMVECTOR)reference.OriginPosition;
	v0 = DirectX::XMVector3InverseRotate(v0, reference.GlobalOrientation);

	Scaling = { 1.0f, XMVectorGetX(XMVector3Length(v0)), 1.0f };

	// with Constraint No-Yaw
	v0 = DirectX::XMVector3Normalize(v0);
	XMFLOAT4A Sp;
	DirectX::XMStoreFloat4A(&Sp, v0);
	float Roll = -std::asinf(Sp.x);
	float Pitch = std::atan2f(Sp.z, Sp.y);
	this->LocalOrientation = XMQuaternionRotationRollPitchYaw(Pitch, 0.0f, Roll);
	this->GlobalOrientation = XMQuaternionMultiply(this->LocalOrientation, reference.GlobalOrientation);
}

DirectX::XMMATRIX BoneState::TransformMatrix(const BoneState & from, const BoneState & to)
{
	XMMATRIX MScaling = XMMatrixScalingFromVector((XMVECTOR)to.Scaling / (XMVECTOR)from.Scaling);
	XMVECTOR VRotationOrigin = XMVectorSelect(g_XMSelect1110.v, (DirectX::XMVECTOR)from.OriginPosition, g_XMSelect1110.v);
	XMMATRIX MRotation = XMMatrixRotationQuaternion(XMQuaternionInverse(from.GlobalOrientation));
	XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, to.OriginPosition, g_XMSelect1110.v);

	XMMATRIX M = XMMatrixTranslationFromVector(-VRotationOrigin);
	M = XMMatrixMultiply(M, MRotation);
	M = XMMatrixMultiply(M, MScaling);
	MRotation = XMMatrixRotationQuaternion(to.GlobalOrientation);
	M = XMMatrixMultiply(M, MRotation);
	M.r[3] += VTranslation;
	return M;
}

DirectX::XMMATRIX BoneState::RigidTransformMatrix(const BoneState & from, const BoneState & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR) to.OriginPosition - (XMVECTOR) from.OriginPosition;
	return XMMatrixRigidTransform(from.OriginPosition, rot, tra);
}

DirectX::XMDUALVECTOR BoneState::RigidTransformDualQuaternion(const BoneState & from, const BoneState & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR)to.OriginPosition - (XMVECTOR)from.OriginPosition;
	return XMDualQuaternionRigidTransform(from.OriginPosition, rot, tra);
}

Armatures::StaticArmature::StaticArmature(size_t JointCount, int * JointsParentIndices)
	: Joints(JointCount)
{
	ParentsMap.resize(JointCount);
	TopologyOrder.resize(JointCount);

	for (size_t i = 0; i < JointCount; i++)
	{
		ParentsMap[i] = JointsParentIndices[i];
		Joints[i].ID = i;
		Joints[i].ConstraintType = JointConstriantType::JointConstraint_None;
		if (ParentsMap[i] != i && ParentsMap[i] >= 0)
		{
			Joints[ParentsMap[i]].append_children_back(&Joints[i]);
		}
		else
		{
			rootIdx = i;
		}
	}

	using namespace boost;
	using namespace boost::adaptors;
	copy(*root() | transformed([](const Joint& joint) {return joint.ID; }), TopologyOrder.begin());
}

inline Armatures::StaticArmature::~StaticArmature()
{

}

//void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);

Joint * Armatures::StaticArmature::at(int index) {
	return &Joints[index];
}

Joint * Armatures::StaticArmature::root()
{
	return &Joints[rootIdx];
}

// Interpolate the local-rotation and scaling, "interpolate in Time"

void Armatures::BoneAnimationFrame::Interpolate(const BoneAnimationFrame & lhs, const BoneAnimationFrame & rhs, float t)
{
	assert((TargetArmature == lhs.TargetArmature) && (lhs.TargetArmature == rhs.TargetArmature));
	for (size_t i = 0; i < lhs.size(); i++)
	{
		(*this)[i].LocalOrientation = DirectX::XMQuaternionSlerp(lhs[i].LocalOrientation, rhs[i].LocalOrientation, t);
		(*this)[i].Scaling = DirectX::XMVectorLerp(lhs[i].Scaling, rhs[i].Scaling, t);
		if (TargetArmature->ParentsMap[i] >= 0)
		{
			(*this)[i].UpdateGlobalData((*this)[TargetArmature->ParentsMap[i]]);
		}
	}
	//for (auto& joint : Skeleton->Root()->descendants())
	//{
	//	auto pParent = joint.parent();
	//	if (pParent)
	//		(*this)[joint.ID].UpdateGlobalData((*this)[pParent->ID]);
	//}
}
