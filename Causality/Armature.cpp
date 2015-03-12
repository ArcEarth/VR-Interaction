#include "Armature.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
#include <boost\range\algorithm.hpp>

using namespace DirectX;
using namespace Causality;

void BoneDisplacement::UpdateGlobalData(const BoneDisplacement & reference)
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

void BoneDisplacement::UpdateLocalData(const BoneDisplacement& reference)
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


void BoneDisplacement::UpdateLocalDataByPositionOnly(const BoneDisplacement & reference)
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

DirectX::XMMATRIX BoneDisplacement::TransformMatrix(const BoneDisplacement & from, const BoneDisplacement & to)
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

DirectX::XMMATRIX BoneDisplacement::RigidTransformMatrix(const BoneDisplacement & from, const BoneDisplacement & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR) to.OriginPosition - (XMVECTOR) from.OriginPosition;
	return XMMatrixRigidTransform(from.OriginPosition, rot, tra);
}

DirectX::XMDUALVECTOR BoneDisplacement::RigidTransformDualQuaternion(const BoneDisplacement & from, const BoneDisplacement & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GlobalOrientation);
	rot = XMQuaternionMultiply(rot, to.GlobalOrientation);
	XMVECTOR tra = (XMVECTOR)to.OriginPosition - (XMVECTOR)from.OriginPosition;
	return XMDualQuaternionRigidTransform(from.OriginPosition, rot, tra);
}

Causality::StaticArmature::StaticArmature(size_t JointCount, int * Parents)
	: Joints(JointCount)
{
	TopologyOrder.resize(JointCount);

	for (size_t i = 0; i < JointCount; i++)
	{
		Joints[i].SetID(i);
		if (Parents[i] != i && Parents[i] >= 0)
		{
			Joints[Parents[i]].append_children_back(&Joints[i]);
		}
		else
		{
			RootIdx = i;
		}
	}
	CaculateTopologyOrder();
}

Causality::StaticArmature::~StaticArmature()
{

}

//void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);

Joint * Causality::StaticArmature::at(int index) {
	return &Joints[index];
}

Joint * Causality::StaticArmature::root()
{
	return &Joints[RootIdx];
}

size_t Causality::StaticArmature::size() const
{
	return Joints.size();
}

void Causality::StaticArmature::CaculateTopologyOrder()
{
	using namespace boost;
	using namespace boost::adaptors;
	copy(root()->nodes() | transformed([](const Joint& joint) {return joint.ID(); }), TopologyOrder.begin());

}

// Interpolate the local-rotation and scaling, "interpolate in Time"

Eigen::VectorXf Causality::BoneDisplacementFrame::LocalRotationVector() const
{
	Eigen::VectorXf fvector(size() * 3);
	Eigen::Vector3f v{ 0,1,0 };

	for (size_t i = 0; i < size(); i++)
	{
		const auto& bone = (*this)[i];
		XMVECTOR q = XMLoadFloat4A(reinterpret_cast<const XMFLOAT4A*>(&bone.LocalOrientation));
		DirectX::Quaternion lq = XMQuaternionLn(bone.LocalOrientation);
		Eigen::Map<const Eigen::Vector3f> elq(&lq.x);
		fvector.block<3, 1>(i * 3, 0) = elq;
	}

	return fvector;
}

void Causality::BoneDisplacementFrame::UpdateFromLocalRotationVector(const IArmature& armature,const Eigen::VectorXf fv)
{
	auto& This = *this;
	auto& sarmature = static_cast<const StaticArmature&>(armature);
	for (auto i : sarmature.joint_indices())
	{
		auto data = fv.block<3, 1>(i * 3, 0).data();
		This[i].LocalOrientation = XMQuaternionExp(XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(data)));
		if (!armature[i]->is_root())
		{
			This[i].UpdateGlobalData(This[armature[i]->ParentID()]);
		}
	}
}

void Causality::BoneDisplacementFrame::Interpolate(BoneDisplacementFrame& out, const BoneDisplacementFrame & lhs, const BoneDisplacementFrame & rhs, float t, const IArmature& armature)
{
	//assert((Armature == lhs.pArmature) && (lhs.pArmature == rhs.pArmature));
	for (size_t i = 0; i < lhs.size(); i++)
	{
		out[i].LocalOrientation = DirectX::Quaternion::Slerp(lhs[i].LocalOrientation, rhs[i].LocalOrientation, t);
		out[i].Scaling = DirectX::XMVectorLerp(lhs[i].Scaling, rhs[i].Scaling, t);
		if (!armature[i]->is_root())
		{
			out[i].UpdateGlobalData(out[armature[i]->ParentID()]);
		}
	}
}

void Causality::BoneDisplacementFrame::Blend(BoneDisplacementFrame& out, const BoneDisplacementFrame & lhs, const BoneDisplacementFrame & rhs, float * blend_weights, const IArmature& armature)
{

}

Eigen::VectorXf Causality::AnimationSpace::CaculateFrameFeatureVectorLnQuaternion(const frame_type & frame) const
{
	Eigen::VectorXf fvector(frame.size() * 3);
	Eigen::Vector3f v { 0,1,0 };

	for (size_t i = 0; i < frame.size(); i++)
	{
		const auto& bone = frame[i];
		XMVECTOR q = XMLoadFloat4A(reinterpret_cast<const XMFLOAT4A*>(&bone.LocalOrientation));
		DirectX::Quaternion lq = XMQuaternionLn(bone.LocalOrientation);
		Eigen::Map<const Eigen::Vector3f> elq(&lq.x);
		fvector.block<3, 1>(i * 3, 0) = elq;
	}

	fvector = fvector.cwiseProduct(Wb);
	return fvector;
}

Eigen::VectorXf Causality::AnimationSpace::CaculateFrameFeatureVectorEndPointNormalized(const frame_type & frame) const
{
	int N = frame.size();
	Eigen::VectorXf fvector(N * 3);
	//Eigen::Vector3f v{ 0,1,0 };
	XMVECTOR v = g_XMIdentityR1.v;
	std::vector<Vector3> sv(N + 1); // Allocate one more space
	for (size_t i = 0; i < N; i++)
	{
		const auto& bone = frame[i];
		XMVECTOR q =  bone.EndPostion;
		q = XMVector3Rotate(v, q);
		XMStoreFloat4(&reinterpret_cast<XMFLOAT4&>(sv[i]),q);
		// HACK: equals to sv[i]=q
		// Float4 functions is significant faster
		// and all memery address is accessable = =
	}

	using namespace Eigen;
	Map<const VectorXf> eep(&sv[0].x, N*3);
	//Map<const Matrix3Xf, Aligned, Stride<1, sizeof(BoneDisplacement) / sizeof(float)>> eep(&frame[0].EndPostion.x, N);

	//Map<const Matrix3Xf,Aligned, Stride<1,sizeof(BoneDisplacement)/sizeof(float)>> eep(&frame[0].EndPostion.x, N);
	fvector = eep.cwiseProduct(Wb);//.asDiagonal();
	return fvector;
}

// !!! This will NOT work since the space will always be full-rank (with enough key frames)
// and the projection will always be the same as the input feature vector
// How about doing a PCA?
float Causality::AnimationSpace::PoseDistancePCAProjection(const frame_type & frame) const
{
	auto fv = CaculateFrameFeatureVectorLnQuaternion(frame);

	// Assupt the space have mutiple basis
	fv -= X0;
	auto w = XpInv * fv;
	auto w0 = 1.0f - w.sum();

	VectorX px = X * w;
	px += w0 * X0;
	
	// px is the projected point from fv to Space [X0 X]
	auto distance = (fv - px).norm();

	return distance;
}

Eigen::RowVectorXf Causality::AnimationSpace::PoseDistanceNearestNeibor(const frame_type & frame) const
{
	auto fv = CaculateFrameFeatureVectorLnQuaternion(frame);

	// Here X is an dense sampled frames feature vector from the animations
	auto D = X - fv.replicate(1, X.cols());
	auto dis = D.colwise().squaredNorm();
	return dis;
}

Eigen::VectorXf Causality::AnimationSpace::CaculateFrameDynamicFeatureVectorJointVelocityHistogram(const frame_type & frame, const frame_type & prev_frame) const
{
	using namespace Eigen;
	auto N = frame.size();
	VectorXf Jvh(N);
	for (size_t i = 0; i < N; i++)
	{
		auto disp = frame[i].EndPostion - prev_frame[i].EndPostion;
		Jvh(i) = disp.Length();
	}

	Jvh.normalize();
	return Jvh;
}

// Nothing but Bhattacharyya distance
Eigen::RowVectorXf Causality::AnimationSpace::DynamicSimiliarityJvh(const frame_type & frame, const frame_type & prev_frame) const
{
	auto fv = CaculateFrameDynamicFeatureVectorJointVelocityHistogram(frame, prev_frame);
	Eigen::MatrixXf Dis = Xv.cwiseProduct(fv.replicate(1, Xv.cols()));
	Dis = Dis.cwiseSqrt();
	return Dis.colwise().sum();;
}

//float Causality::AnimationSpace::DynamicSimiliarityJvh(const velocity_frame_type & velocity_frame) const
//{
//	
//}

void Causality::AnimationSpace::CaculateXpInv()
{
	auto Xt = X.transpose();
	XpInv = Xt * X;
	XpInv.ldlt().solveInPlace(Xt);
}

bool Causality::ArmatureKeyframeAnimation::pre_interpolate_frames(double frameRate)
{
	float delta = 1.0f / frameRate;
	auto& armature = *pArmature;
	float t = 0;
	for (size_t i = 1; i < KeyFrames.size(); i++)
	{
		const frame_type& lhs = KeyFrames[i-1];
		const frame_type& rhs = KeyFrames[i];
		for (float t = lhs.Time; t < rhs.Time; t += delta)
		{
			frames.emplace_back(armature.size());
			frame_type::Interpolate(frames.back(), lhs, rhs, t, armature);
		}
	}
	frames.shrink_to_fit();
}

void Causality::ArmatureTransform::TransformBack(frame_type & source_frame, const frame_type & target_frame) const
{
}
