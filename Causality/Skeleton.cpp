#include "Skeleton.h"

using namespace DirectX;

void Kinematics::BoneState::UpdateGlobalData(const BoneState & refernece)
{
	XMVECTOR ParQ = refernece.GlobalOrientation;
	XMVECTOR Q = XMQuaternionMultiply(ParQ, LocalOrientation);
	GlobalOrientation = Q;
	OriginPosition = refernece.EndPostion;
	XMVECTOR V = Scaling;
	V = XMVectorMultiply(V, g_XMIdentityR1.v);
	V = XMVector3Rotate(V, Q);
	V = XMVectorAdd(V, OriginPosition);
	EndPostion = V;
}

void Kinematics::BoneState::SetGlobalPosition(const BoneState & reference)
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
	//XMVECTOR vRot = XMVector3Rotate(g_XMIdentityR1,Entity[state].Rotation);

	//Entity[state].Rotation = Quaternion::RotationQuaternion(g_XMIdentityR1,v0);

	this->GlobalOrientation = XMQuaternionMultiply(this->LocalOrientation, reference.GlobalOrientation);
}

DirectX::XMMATRIX Kinematics::BoneState::TransformMatrix(const BoneState & from, const BoneState & to)
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

DirectX::XMMATRIX Kinematics::BoneState::RigidTransformMatrix(const BoneState & from, const BoneState & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR) to.OriginPosition - (XMVECTOR) from.OriginPosition;
	return XMMatrixRigidTransform(from.OriginPosition, rot, tra);
}

DirectX::XMDUALVECTOR Kinematics::BoneState::RigidTransformDualQuaternion(const BoneState & from, const BoneState & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR)to.OriginPosition - (XMVECTOR)from.OriginPosition;
	return XMDualQuaternionRigidTransform(from.OriginPosition, rot, tra);
}
