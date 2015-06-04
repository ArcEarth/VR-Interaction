#include "pch_bcl.h"
#include "Armature.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
#include <boost\range\algorithm.hpp>
#include <regex>
#include <boost\assign.hpp>
#include <iostream>
#include <Eigen\Eigen>

using namespace DirectX;
using namespace Causality;

//void Bone::UpdateGlobalData(const Bone & reference)
//{
//	XMVECTOR ParQ = reference.GblRotation;
//	XMVECTOR Q = XMQuaternionMultiply(ParQ, LclRotation);
//	GblRotation = Q;
//
//	OriginPosition = reference.EndPostion;
//	//XMVECTOR V = LclScaling;
//	//V = XMVectorMultiply(V, g_XMIdentityR1.v);
//	//V = XMVector3Rotate(V, Q);
//
//	V = XMVectorAdd(V, OriginPosition);
//	EndPostion = V;
//}

void Bone::UpdateGlobalData(const Bone & reference)
{
	XMVECTOR ParQ = reference.GblRotation.LoadA();
	XMVECTOR Q = XMQuaternionMultiply(LclRotation.LoadA(), ParQ);
	XMVECTOR S = reference.GblScaling.LoadA() * LclScaling.LoadA();
	GblRotation.StoreA(Q);
	GblScaling.StoreA(S);

	OriginPosition = reference.EndPostion; // should be a constriant

	XMVECTOR V = LclTranslation.LoadA();
	V *= S;
	V = XMVector3Rotate(V, ParQ);
	V = XMVectorAdd(V, OriginPosition.LoadA());

	EndPostion.StoreA(V);
}

// This will assuming LclTranslation is not changed
void Bone::UpdateLocalData(const Bone& reference)
{
	OriginPosition = reference.EndPostion;
	XMVECTOR ParQ = reference.GblRotation;
	ParQ = XMQuaternionInverse(ParQ);
	XMVECTOR Q = GblRotation;
	LclRotation = XMQuaternionMultiply(ParQ, Q);

	Q = (XMVECTOR)EndPostion - (XMVECTOR)reference.EndPostion;
	Q = XMVector3Length(Q);
	Q = XMVectorSelect(g_XMIdentityR1.v, Q, g_XMIdentityR1.v);
	LclTranslation = Q;
	LclScaling = Vector3::One;
}


void Bone::UpdateLocalDataByPositionOnly(const Bone & reference)
{
	XMVECTOR v0 = (XMVECTOR)this->OriginPosition - (XMVECTOR)reference.OriginPosition;
	v0 = DirectX::XMVector3InverseRotate(v0, reference.GblRotation);

	LclScaling = { 1.0f, XMVectorGetX(XMVector3Length(v0)), 1.0f };

	// with Constraint No-X
	v0 = DirectX::XMVector3Normalize(v0);
	XMFLOAT4A Sp;
	DirectX::XMStoreFloat4A(&Sp, v0);
	float Roll = -std::asinf(Sp.x);
	float Pitch = std::atan2f(Sp.z, Sp.y);
	this->LclRotation = XMQuaternionRotationRollPitchYaw(Pitch, 0.0f, Roll);
	this->GblRotation = XMQuaternionMultiply(this->LclRotation, reference.GblRotation);
}

DirectX::XMMATRIX Bone::TransformMatrix(const Bone & from, const Bone & to)
{
	using DirectX::operator+=;

	XMMATRIX MScaling = XMMatrixScalingFromVector((XMVECTOR)to.LclScaling / (XMVECTOR)from.LclScaling);
	XMVECTOR VRotationOrigin = XMVectorSelect(g_XMSelect1110.v, (DirectX::XMVECTOR)from.OriginPosition, g_XMSelect1110.v);
	XMMATRIX MRotation = XMMatrixRotationQuaternion(XMQuaternionInverse(from.GblRotation));
	XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, to.OriginPosition, g_XMSelect1110.v);

	XMMATRIX M = XMMatrixTranslationFromVector(-VRotationOrigin);
	M = XMMatrixMultiply(M, MRotation);
	M = XMMatrixMultiply(M, MScaling);
	MRotation = XMMatrixRotationQuaternion(to.GblRotation);
	M = XMMatrixMultiply(M, MRotation);
	M.r[3] += VTranslation;
	return M;
}

DirectX::XMMATRIX Bone::RigidTransformMatrix(const Bone & from, const Bone & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GblRotation);
	rot = XMQuaternionMultiply(rot, to.GblRotation);
	XMVECTOR tra = (XMVECTOR)to.OriginPosition - (XMVECTOR)from.OriginPosition;
	return XMMatrixRigidTransform(from.OriginPosition, rot, tra);
}

DirectX::XMDUALVECTOR Bone::RigidTransformDualQuaternion(const Bone & from, const Bone & to)
{
	XMVECTOR rot = XMQuaternionInverse(from.GblRotation);
	rot = XMQuaternionMultiply(rot, to.GblRotation);
	XMVECTOR tra = (XMVECTOR)to.OriginPosition - (XMVECTOR)from.OriginPosition;
	return XMDualQuaternionRigidTransform(from.OriginPosition, rot, tra);
}

StaticArmature::StaticArmature(std::istream & file)
{
	size_t jointCount;
	file >> jointCount;

	Joints.resize(jointCount);
	DefaultFrame.resize(jointCount);

	// Joint line format: 
	// Hip(Name) -1(ParentID)
	// 1.5(Pitch) 2.0(Yaw) 0(Roll) 0.5(BoneLength)
	for (size_t idx = 0; idx < jointCount; idx++)
	{
		auto& joint = Joints[idx];
		auto& bone = DefaultFrame[idx];
		((JointBasicData&)joint).ID = idx;
		file >> ((JointBasicData&)joint).Name >> ((JointBasicData&)joint).ParentID;
		if (joint.ParentID() != idx && joint.ParentID() >= 0)
		{
			Joints[joint.ParentID()].append_children_back(&joint);
		}
		else
		{
			RootIdx = idx;
		}

		Vector4 vec;
		file >> vec.x >> vec.y >> vec.z >> vec.w;
		bone.LclRotation = XMQuaternionRotationRollPitchYawFromVectorRH(vec);
		bone.LclScaling.y = vec.w;
	}

	CaculateTopologyOrder();
	DefaultFrame.RebuildGlobal(*this);
}

StaticArmature::StaticArmature(size_t JointCount, int * Parents, const char* const* Names)
	: Joints(JointCount)
{
	for (size_t i = 0; i < JointCount; i++)
	{
		Joints[i].SetID(i);
		Joints[i].SetName(Names[i]);
		((JointBasicData&)Joints[i]).ParentID = Parents[i];
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

StaticArmature::~StaticArmature()
{

}

inline StaticArmature::StaticArmature(self_type && rhs)
{
	using std::move;
	RootIdx = rhs.RootIdx;
	Joints = move(rhs.Joints);
	TopologyOrder = move(rhs.TopologyOrder);
	DefaultFrame = move(rhs.DefaultFrame);
}

//void GetBlendMatrices(_Out_ DirectX::XMFLOAT4X4* pOut);

Joint * StaticArmature::at(int index) {
	return &Joints[index];
}

Joint * StaticArmature::root()
{
	return &Joints[RootIdx];
}

size_t StaticArmature::size() const
{
	return Joints.size();
}

const IArmature::frame_type & StaticArmature::default_frame() const
{
	return DefaultFrame;
	// TODO: insert return statement here
}

void StaticArmature::CaculateTopologyOrder()
{
	using namespace boost;
	using namespace boost::adaptors;
	TopologyOrder.resize(size());
	copy(root()->nodes() | transformed([](const Joint& joint) {return joint.ID(); }), TopologyOrder.begin());

}

// Interpolate the local-rotation and scaling, "interpolate in Time"

AffineFrame::AffineFrame(size_t size)
	: BaseType(size)
{
}

AffineFrame::AffineFrame(const IArmature & armature)
	: BaseType(armature.default_frame())
{
}

void AffineFrame::RebuildGlobal(const IArmature & armature)
{
	for (auto& joint : armature.joints())
	{
		auto& bone = at(joint.ID());
		if (joint.is_root())
		{
			bone.GblRotation = bone.LclRotation;
			bone.GblScaling = Vector3::One;
			bone.EndPostion = bone.LclTranslation;
		}
		else
		{
			//bone.OriginPosition = at(joint.ParentID()).EndPostion;

			bone.UpdateGlobalData(at(joint.ParentID()));
		}
	}
}

Eigen::VectorXf AffineFrame::LocalRotationVector() const
{
	Eigen::VectorXf fvector(size() * 3);
	Eigen::Vector3f v{ 0,1,0 };

	for (size_t i = 0; i < size(); i++)
	{
		const auto& bone = (*this)[i];
		XMVECTOR q = XMLoadFloat4A(reinterpret_cast<const XMFLOAT4A*>(&bone.LclRotation));
		DirectX::Quaternion lq = XMQuaternionLn(bone.LclRotation);
		Eigen::Map<const Eigen::Vector3f> elq(&lq.x);
		fvector.block<3, 1>(i * 3, 0) = elq;
	}

	return fvector;
}

void AffineFrame::UpdateFromLocalRotationVector(const IArmature& armature, const Eigen::VectorXf fv)
{
	auto& This = *this;
	auto& sarmature = static_cast<const StaticArmature&>(armature);
	for (auto i : sarmature.joint_indices())
	{
		auto data = fv.block<3, 1>(i * 3, 0).data();
		This[i].LclRotation = XMQuaternionExp(XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(data)));
		if (!armature[i]->is_root())
		{
			This[i].UpdateGlobalData(This[armature[i]->ParentID()]);
		}
	}
}

void AffineFrame::Interpolate(AffineFrame& out, const AffineFrame & lhs, const AffineFrame & rhs, float t, const IArmature& armature)
{
	//assert((Armature == lhs.pArmature) && (lhs.pArmature == rhs.pArmature));
	for (size_t i = 0; i < lhs.size(); i++)
	{
		out[i].LclRotation = DirectX::Quaternion::Slerp(lhs[i].LclRotation, rhs[i].LclRotation, t);
		out[i].LclScaling = DirectX::XMVectorLerp(lhs[i].LclScaling, rhs[i].LclScaling, t);
		if (!armature[i]->is_root())
		{
			out[i].UpdateGlobalData(out[armature[i]->ParentID()]);
		}
	}
}

void AffineFrame::Blend(AffineFrame& out, const AffineFrame & lhs, const AffineFrame & rhs, float * blend_weights, const IArmature& armature)
{

}

void Causality::AffineFrame::TransformMatrix(DirectX::XMFLOAT3X4 * pOut, const self_type & from, const self_type & to)
{
	using namespace std;
	auto n = min(from.size(), to.size());
	for (int i = 0; i <= n; ++i)
	{
		XMMATRIX mat = Bone::TransformMatrix(from[i], to[i]);
		mat = XMMatrixTranspose(mat);
		XMStoreFloat3x4(pOut + i, mat);
	}
}

void Causality::AffineFrame::TransformMatrix(DirectX::XMFLOAT4X4 * pOut, const self_type & from, const self_type & to)
{
	using namespace std;
	auto n = min(from.size(), to.size());
	for (int i = 0; i <= n; ++i)
	{
		XMMATRIX mat = Bone::TransformMatrix(from[i], to[i]);
		mat = XMMatrixTranspose(mat);
		XMStoreFloat4x4(pOut + i, mat);
	}
}

Eigen::VectorXf BehavierSpace::FrameFeatureVectorLnQuaternion(const frame_type & frame) const
{
	Eigen::VectorXf fvector(frame.size() * 3);
	Eigen::Vector3f v{ 0,1,0 };

	for (size_t i = 0; i < frame.size(); i++)
	{
		const auto& bone = frame[i];
		XMVECTOR q = XMLoadFloat4A(reinterpret_cast<const XMFLOAT4A*>(&bone.LclRotation));
		DirectX::Quaternion lq = XMQuaternionLn(bone.LclRotation);
		Eigen::Map<const Eigen::Vector3f> elq(&lq.x);
		fvector.block<3, 1>(i * 3, 0) = elq;
	}

	fvector = fvector.cwiseProduct(Wb);
	return fvector;
}

KinematicBlock* Causality::ShrinkChainToBlock(const Joint* pJoint)
{
	if (pJoint == nullptr || pJoint->is_null()) return nullptr;
	KinematicBlock* pBlock = new KinematicBlock;

	auto pChild = pJoint;
	do
	{
		pBlock->Joints.push_back(pChild);
		pJoint = pChild;
		pChild = pChild->first_child();
	} while (pChild && !pChild->next_sibling());

	for (auto& child : pJoint->children())
	{
		auto childBlcok = ShrinkChainToBlock(&child);
		pBlock->append_children_back(childBlcok);
	}
	return pBlock;
}

BehavierSpace::animation_type & BehavierSpace::operator[](const std::string & name)
{
	for (auto& clip : m_AnimClips)
	{
		if (clip.Name == name) return clip;
	}
	throw std::out_of_range("No animation with specified name exist in this aniamtion space.");
}

const BehavierSpace::animation_type & BehavierSpace::operator[](const std::string & name) const
{
	for (const auto& clip : m_AnimClips)
	{
		if (clip.Name == name) return clip;
	}
	throw std::out_of_range("No animation with specified name exist in this aniamtion space.");
}

void Causality::BehavierSpace::AddAnimationClip(animation_type && animation)
{
#ifdef _DEBUG
	for (auto& clip : m_AnimClips)
		if (clip.Name == animation.Name)
		throw std::out_of_range("Name of animation is already existed.");
#endif
	m_AnimClips.emplace_back(std::move(animation));
}

BehavierSpace::animation_type & BehavierSpace::AddAnimationClip(const std::string & name)
{
#ifdef _DEBUG
	for (auto& clip : m_AnimClips)
		if (clip.Name == name)
		throw std::out_of_range("Name of animation is already existed.");
#endif
	m_AnimClips.emplace_back(name);
	auto& anim = m_AnimClips.back();
	anim.SetArmature(*m_pArmature);
	anim.SetDefaultFrame(m_pArmature->default_frame());
	return anim;
}

bool Causality::BehavierSpace::Contains(const std::string & name) const
{
	for (auto& clip : m_AnimClips)
	{
		if (clip.Name == name) return true;
	}
	return false;
}

void Causality::BehavierSpace::UpdateBlock()
{
	auto& armature = *m_pArmature;
	m_Blocks.SetArmature(armature);	
}

const IArmature & BehavierSpace::Armature() const { return *m_pArmature; }

IArmature & BehavierSpace::Armature() { return *m_pArmature; }

void BehavierSpace::SetArmature(IArmature & armature) {
	assert(this->Clips().empty()); 
	m_pArmature = &armature; 
	UpdateBlock();
}

const BehavierSpace::frame_type & BehavierSpace::RestFrame() const { return Armature().default_frame(); }

Eigen::VectorXf BehavierSpace::FrameFeatureVectorEndPointNormalized(const frame_type & frame) const
{
	int N = frame.size();
	Eigen::VectorXf fvector(N * 3);
	//Eigen::Vector3f v{ 0,1,0 };
	XMVECTOR v = g_XMIdentityR1.v;
	std::vector<Vector3> sv(N + 1); // Allocate one more space
	for (int i = 0; i < N; i++)
	{
		const auto& bone = frame[i];
		XMVECTOR q = bone.EndPostion;
		q = XMVector3Rotate(v, q);
		XMStoreFloat4(&reinterpret_cast<XMFLOAT4&>(sv[i]), q);
		// HACK: equals to sv[i]=q
		// Float4 functions is significant faster
		// and all memery address is accessable = =
	}

	using namespace Eigen;
	Map<const VectorXf> eep(&sv[0].x, N * 3);
	//Map<const Matrix3Xf, Aligned, Stride<1, sizeof(Bone) / sizeof(float)>> eep(&frame[0].EndPostion.x, N);

	//Map<const Matrix3Xf,Aligned, Stride<1,sizeof(Bone)/sizeof(float)>> eep(&frame[0].EndPostion.x, N);
	fvector = eep.cwiseProduct(Wb);//.asDiagonal();
	return fvector;
}

Eigen::MatrixXf Causality::BehavierSpace::AnimationMatrixEndPosition(const ArmatureFrameAnimation & animation) const
{
	const auto & frames = animation.GetFrameBuffer();
	int K = animation.GetFrameBuffer().size();
	int N = animation.Armature().size();
	Eigen::MatrixXf fmatrix(N * 3, K);
	CacAnimationMatrixEndPosition(animation, fmatrix);
	return fmatrix;
}

void Causality::BehavierSpace::CacAnimationMatrixEndPosition(const ArmatureFrameAnimation & animation, Eigen::MatrixXf & fmatrix) const
{
	const auto & frames = animation.GetFrameBuffer();
	int K = animation.GetFrameBuffer().size();
	int N = animation.Armature().size();
	if (fmatrix.rows() != N * 3 || fmatrix.cols() != K)
		fmatrix.resize(N * 3, K);
	for (size_t i = 0; i < K; i++)
	{
		auto& frame = frames[i];
		auto fvector = fmatrix.col(i);
		//Eigen::Vector3f v{ 0,1,0 };
		XMVECTOR v = g_XMIdentityR1.v;
		std::vector<Vector3> sv(N + 1); // Allocate one more space
		for (int i = 0; i < N; i++)
		{
			const auto& bone = frame[i];
			XMVECTOR q = bone.EndPostion;
			q = XMVector3Rotate(v, q);
			XMStoreFloat4(&reinterpret_cast<XMFLOAT4&>(sv[i]), q);
			// HACK: equals to sv[i]=q
			// Float4 functions is significant faster
			// and all memery address is accessable = =
		}

		using namespace Eigen;
		Map<const VectorXf> eep(&sv[0].x, N * 3);
		//Map<const Matrix3Xf, Aligned, Stride<1, sizeof(Bone) / sizeof(float)>> eep(&frame[0].EndPostion.x, N);

		//Map<const Matrix3Xf,Aligned, Stride<1,sizeof(Bone)/sizeof(float)>> eep(&frame[0].EndPostion.x, N);
		fvector = eep.cwiseProduct(Wb);//.asDiagonal();
	}
}

// !!! This will NOT work since the space will always be full-rank (with enough key frames)
// and the projection will always be the same as the input feature vector
// How about doing a PCA?
float BehavierSpace::PoseDistancePCAProjection(const frame_type & frame) const
{
	auto fv = FrameFeatureVectorLnQuaternion(frame);

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

Eigen::RowVectorXf BehavierSpace::PoseSquareDistanceNearestNeibor(const frame_type & frame) const
{
	auto fv = FrameFeatureVectorLnQuaternion(frame);

	// Here X is an dense sampled frames feature vector from the animations
	auto D = X - fv.replicate(1, X.cols());
	auto dis = D.colwise().squaredNorm();
	return dis;
}

Eigen::RowVectorXf Causality::BehavierSpace::StaticSimiliartyEculidDistance(const frame_type & frame) const
{
	using namespace Eigen;
	auto dis = PoseSquareDistanceNearestNeibor(frame);
	dis /= -(SegmaDis*SegmaDis);
	dis = dis.array().exp();
	return dis;
}

Eigen::VectorXf BehavierSpace::CaculateFrameDynamicFeatureVectorJointVelocityHistogram(const frame_type & frame, const frame_type & prev_frame) const
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
Eigen::RowVectorXf BehavierSpace::DynamicSimiliarityJvh(const frame_type & frame, const frame_type & prev_frame) const
{
	auto fv = CaculateFrameDynamicFeatureVectorJointVelocityHistogram(frame, prev_frame);
	Eigen::MatrixXf Dis = Xv.cwiseProduct(fv.replicate(1, Xv.cols()));
	Dis = Dis.cwiseSqrt();
	return Dis.colwise().sum();;
}

//float BehavierSpace::DynamicSimiliarityJvh(const velocity_frame_type & velocity_frame) const
//{
//	
//}

float Causality::BehavierSpace::FrameLikilihood(const frame_type & frame, const frame_type & prev_frame) const
{
	auto stsim = StaticSimiliartyEculidDistance(frame);
	auto dysim = DynamicSimiliarityJvh(frame, prev_frame);
	stsim = stsim.cwiseProduct(dysim);
	return stsim.minCoeff();
}

float Causality::BehavierSpace::FrameLikilihood(const frame_type & frame) const
{
	auto minDis = PoseSquareDistanceNearestNeibor(frame).minCoeff();
	return expf(-minDis / (SegmaDis*SegmaDis));
}

vector<ArmatureTransform> BehavierSpace::GenerateBindings()
{
	vector<ArmatureTransform> bindings;
	auto& armature = Armature();

	for (auto& joint : armature.joints())
	{
		joint.AssignSemanticsBasedOnName();
	}
	return bindings;
}

void BehavierSpace::CaculateXpInv()
{
	auto Xt = X.transpose();
	XpInv = Xt * X;
	XpInv.ldlt().solveInPlace(Xt);
}

ArmatureFrameAnimation::ArmatureFrameAnimation(std::istream & file)
{
	using namespace std;
	using namespace DirectX;
	self_type animation;
	int keyframs, joints;
	file >> joints >> keyframs;
	animation.KeyFrames.resize(keyframs);
	for (int i = 0; i < keyframs; i++)
	{
		auto& frame = animation.KeyFrames[i];
		frame.resize(joints);
		double seconds;
		file >> frame.Name;
		file >> seconds;
		frame.Time = (time_seconds)seconds;
		for (auto& bone : frame)
		{
			Vector3 vec;
			auto& scl = bone.LclScaling;
			//char ch;
			file >> vec.x >> vec.y >> vec.z >> scl.y;
			bone.LclRotation = XMQuaternionRotationRollPitchYawFromVector(vec);
		}
	}
}

Causality::ArmatureFrameAnimation::ArmatureFrameAnimation(const std::string & name)
	: base_type(name)
{
}

bool ArmatureFrameAnimation::InterpolateFrames(double frameRate)
{
	float delta = (float)(1.0 / frameRate);
	auto& armature = *pArmature;
	float t = 0;
	for (size_t i = 1; i < KeyFrames.size(); i++)
	{
		const frame_type& lhs = KeyFrames[i - 1];
		const frame_type& rhs = KeyFrames[i];
		for (t = (float)lhs.Time.count(); t < (float)rhs.Time.count(); t += delta)
		{
			frames.emplace_back(armature.size());
			frame_type::Interpolate(frames.back(), lhs, rhs, (float)t, armature);
		}
	}
	frames.shrink_to_fit();
	return true;
}

bool Causality::ArmatureFrameAnimation::GetFrameAt(AffineFrame & outFrame, TimeScalarType time) const
{
	double t = fmod(time.count(), Duration.count());
	int frameIdx = round(t / FrameInterval.count());
	frameIdx = frameIdx % frames.size(); // ensure the index is none negative
	outFrame = frames[frameIdx];
	return true;
}

void ArmatureTransform::TransformBack(frame_type & source_frame, const frame_type & target_frame) const
{
}

iterator_range<std::sregex_token_iterator> words_from_string(const std::string& str)
{
	using namespace std;
	regex wordPattern("[_\\s]?[A-Za-z][a-z]*\\d*");
	sregex_token_iterator wbegin(str.begin(), str.end(), wordPattern);
	iterator_range<sregex_token_iterator> words(wbegin, sregex_token_iterator());
	return words;
}

using namespace std;

/*
namespace std
{
	//inline
	namespace literals {
		//inline
		namespace string_literals
		{
			std::string operator""s(const char* str, std::size_t len)
			{
				return std::string(str, len);
			}
		}
	}
}
*/

std::map<std::string, JointSemanticProperty>
name2semantic = boost::assign::map_list_of
(string("hand"), JointSemanticProperty(Semantic_Hand))
(string("foreleg"), JointSemanticProperty(Semantic_Hand | Semantic_Foot))
(string("arm"), JointSemanticProperty(Semantic_Hand))
(string("claw"), JointSemanticProperty(Semantic_Hand))
(string("wing"), JointSemanticProperty(Semantic_Hand | Semantic_Wing))
(string("head"), JointSemanticProperty(Semantic_Head))
(string("l"), JointSemanticProperty(Semantic_Left))
(string("r"), JointSemanticProperty(Semantic_Right))
(string("left"), JointSemanticProperty(Semantic_Left))
(string("right"), JointSemanticProperty(Semantic_Right))
(string("leg"), JointSemanticProperty(Semantic_Foot))
(string("foot"), JointSemanticProperty(Semantic_Foot))
(string("tail"), JointSemanticProperty(Semantic_Tail))
(string("ear"), JointSemanticProperty(Semantic_Ear))
(string("eye"), JointSemanticProperty(Semantic_Eye))
(string("noise"), JointSemanticProperty(Semantic_Nouse));

const JointSemanticProperty & Joint::AssignSemanticsBasedOnName()
{
	using namespace std;
	using namespace boost::adaptors;

	auto words = words_from_string(Name());
	for (auto& word : words)
	{
		string word_str;
		if (*word.first == '_' || *word.first == ' ')
			word_str = std::string(word.first + 1, word.second);
		else
			word_str = std::string(word.first, word.second);

		for (auto& c : word_str)
		{
			c = std::tolower(c);
		}

		this->Semantic += name2semantic[word_str];
	}
	return this->Semantic;
}

VectorX Causality::KinematicBlock::GetFeatureVector(const AffineFrame & frame) const
{
	return VectorX();
}

void Causality::KinematicBlock::SetFeatureVector(AffineFrame & frame, const VectorX & feature) const
{
}

int Causality::KinematicBlock::FeatureDimension() const
{
	return Joints.size() * FeatureType::Dimension;
}

void Causality::BlockArmature::SetArmature(const IArmature & armature)
{
	m_pRoot.reset(ShrinkChainToBlock(armature.root()));
	int idx = 0, accumIdx = 0;
	for (auto& block : m_pRoot->nodes())
	{
		m_BlocksCache.push_back(&block);
		block.Index = idx++;
		block.AccumulatedJointCount = accumIdx;
		accumIdx += block.Joints.size();
	}
}


Eigen::PermutationMatrix<Eigen::Dynamic> Causality::BlockArmature::GetJointPermutationMatrix(size_t feature_dim) const
{
	using namespace Eigen;

	// Perform a Column permutation so that feature for block wise is consist
	// And remove the position data!!!

	auto N = m_pArmature->size();
	vector<int> indices(N);
	int idx = 0;

	for ( auto pBlock : m_BlocksCache)
	{
		for (auto& pj : pBlock->Joints)
		{
			indices[pj->ID()] = idx++;
		}
	}

	MatrixXi a_indvec = VectorXi::Map(indices.data(), indices.size()).replicate(feature_dim, 1);
	a_indvec.array() *= FeatureType::Dimension;
	a_indvec.colwise() += Matrix<int, -1, 1>::LinSpaced(feature_dim, 0, feature_dim - 1);

#ifdef _DEBUG
	cout << a_indvec << endl;
#endif

	Eigen::PermutationMatrix<Eigen::Dynamic> perm(VectorXi::Map(a_indvec.data(), N*feature_dim));
	return perm;
}
