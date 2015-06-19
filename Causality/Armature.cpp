#include "pch_bcl.h"
#include "Armature.h"
#include "Animations.h"
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

	LclLength = XMVectorGetX(XMVector3Length(V));

	V *= S;

	GblLength = XMVectorGetX(XMVector3Length(V));

	V = XMVector3Rotate(V, ParQ);
	V = XMVectorAdd(V, OriginPosition.LoadA());

	EndPostion.StoreA(V);
}

// This will assuming LclTranslation is not changed
void Bone::UpdateLocalData(const Bone& reference)
{
	OriginPosition = reference.EndPostion;
	XMVECTOR InvParQ = reference.GblRotation;
	InvParQ = XMQuaternionInverse(InvParQ); // PqInv
	XMVECTOR Q = GblRotation;
	Q = XMQuaternionMultiply(Q, InvParQ);
	LclRotation.StoreA(Q);

	Q = (XMVECTOR)EndPostion - (XMVECTOR)reference.EndPostion;
	//Q = XMVector3Length(Q);
	//Q = XMVectorSelect(g_XMIdentityR1.v, Q, g_XMIdentityR1.v);

	GblLength = XMVectorGetX(XMVector3Length(Q));

	Q = XMVector3Rotate(Q,InvParQ);

	XMVECTOR S = GblScaling.LoadA();
	Q /= S;

	LclLength = XMVectorGetX(XMVector3Length(Q));

	LclTranslation = Q;

	S /= reference.GblScaling.LoadA();
	LclScaling = S;
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
	DefaultFrame = new AffineFrame(jointCount);

	// Joint line format: 
	// Hip(Name) -1(ParentID)
	// 1.5(Pitch) 2.0(Yaw) 0(Roll) 0.5(BoneLength)
	for (size_t idx = 0; idx < jointCount; idx++)
	{
		auto& joint = Joints[idx];
		auto& bone = default_frame()[idx];
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
	default_frame().RebuildGlobal(*this);
}

StaticArmature::StaticArmature(size_t JointCount, int * Parents, const char* const* Names)
	: Joints(JointCount)
{
	DefaultFrame = new AffineFrame(JointCount);
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
	return *DefaultFrame;
	// TODO: insert return statement here
}

void Causality::StaticArmature::set_default_frame(frame_type & frame) { DefaultFrame = &frame; }

void StaticArmature::CaculateTopologyOrder()
{
	using namespace boost;
	using namespace boost::adaptors;
	TopologyOrder.resize(size());
	copy(root()->nodes() | transformed([](const Joint& joint) {return joint.ID(); }), TopologyOrder.begin());

}

// Lerp the local-rotation and scaling, "interpolate in Time"

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

const Bone & Causality::IArmature::default_bone(int index) const
{
	return default_frame()[at(index)->ID()];
}
