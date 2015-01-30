#include "stdafx.h"
#include "NaiveWarpper.h"
#include "DXMathExtend.h"
#include <iostream>
using namespace DirectX;
using namespace Kinematics;
using namespace std;

NaiveWarpper::NaiveWarpper(HumanSkeleton* pHuman,Geometrics::DynamicMetaBallModel* pAvatar)
	: m_pHuman(pHuman) , m_pAvatar(pAvatar)
{
}


NaiveWarpper::~NaiveWarpper(void)
{
}

HolographicPoint NaiveWarpper::Inverse_Animation_Transform(DirectX::FXMVECTOR Point , unsigned int Max_Binding_Count) const 
{
	Kinematics::Joint* pBone = m_pAvatar->Skeleton->FindClosestBone(Point,Current);

	//const Geometrics::MetaBallModel& Body = m_pAvatar->GetAnimatedMetaballModel();
	//unsigned int Index = Body.FindClosestMetballindex(Point);
	//Index = Body[Index].BindingIndex;
	//Kinematics::Joint* pBone = (*m_pAvatar->Skeleton)[Index];

	HolographicPoint Hp;

	Hp.clear();
	Hp.Position = pBone->InverseBlendPoint(Point);
	Hp.Indices[0] = pBone->ID;
	Hp.Weights[0] = 1.0f;

	return Hp;
}

Kinematics::IndexedSkeleton& NaiveWarpper::Dictionary() const
{
	return *m_pAvatar->Skeleton;
}


XMVECTOR NaiveWarpper::Animation_Transform(const HolographicPoint& Hpoint) const
{
	return m_pAvatar->SkinningSpacePoint((XMVECTOR)Hpoint.Position,Hpoint.Weights,Hpoint.Indices);
}

// return the distance of this snap
float NaiveWarpper::SnapToSurface(__inout HolographicPoint& HPoint) const 
{
	Vector3 SurfacePoint = (Vector3)HPoint.Position;
	XMVECTOR point = HPoint.Position;

	Joint* pBindBone = m_pAvatar->Skeleton->at(HPoint.Indices[0]);
	bool result;
	XMVECTOR vtrJ = pBindBone->Entity[Default].Position;
//		XMVECTOR vtrP = point - vtrJ;
	if (pBindBone->Parent!=nullptr){
		XMVECTOR vtrB = pBindBone->Parent->Entity[Default].Position;
		vtrB = Projection(point,vtrJ,vtrB);
		vtrJ = point - vtrB;
		bool boneInseide = m_pAvatar->Flesh->Contains(vtrB);
		if (boneInseide)
			result = m_pAvatar->Flesh->RayIntersection(SurfacePoint,vtrB,vtrJ);
		else {
			vtrJ = pBindBone->Parent->Entity[Default].Position - vtrB;
			result = m_pAvatar->Flesh->RayIntersection(SurfacePoint,vtrB,vtrJ);
		}

	}
	else
	{
		XMVECTOR vtrP = point - vtrJ;
		result = m_pAvatar->Flesh->RayIntersection(SurfacePoint,vtrJ,vtrP);
	}

	if (!result)
	{
		return g_INFINITY;
	}

	float r = Vector3::Distance(SurfacePoint,point);
	if (m_pAvatar->Flesh->Contains(point))
		r = -r;

	HPoint.Position = SurfacePoint;
	m_pAvatar->Weighting(SurfacePoint,HPoint.Weights,HPoint.Indices);

	return r;
}

MetaballBasedWarpper::MetaballBasedWarpper(HumanSkeleton* pHuman,Geometrics::DynamicMetaBallModel* pAvatar)
	: m_pHuman(pHuman) , m_pAvatar(pAvatar)
{
}


MetaballBasedWarpper::~MetaballBasedWarpper(void)
{
}

HolographicPoint MetaballBasedWarpper::Inverse_Animation_Transform(DirectX::FXMVECTOR Point , unsigned int Max_Binding_Count) const 
{
	HolographicPoint Hp;
	Hp.clear();
	const Geometrics::MetaBallModel& Body = m_pAvatar->GetAnimatedMetaballModel();
	if (Body.size() == 0)
	{
		Hp.Position = Point;
		Hp.Weights[0] = 0.0f;
		return Hp;
	}
		
	unsigned int Index = Body.FindClosestMetballindex(Point);
	Index = Body[Index].BindingIndex;
	Kinematics::Joint* pBone = (*m_pAvatar->Skeleton)[Index];

	if (pBone == nullptr)
	{
		Hp.Position = Point;
		Hp.Weights[0] = 0.0f;
	} else {
		Hp.Position = pBone->InverseBlendPoint(Point);
		Hp.Indices[0] = pBone->ID;
		Hp.Weights[0] = 1.0f;
	}
	return Hp;
}

Kinematics::IndexedSkeleton& MetaballBasedWarpper::Dictionary() const
{
	return *m_pAvatar->Skeleton;
}


XMVECTOR MetaballBasedWarpper::Animation_Transform(const HolographicPoint& Hpoint) const
{
	return m_pAvatar->SkinningSpacePoint((XMVECTOR)Hpoint.Position,Hpoint.Weights,Hpoint.Indices);
}

XMVECTOR FindSurfacePointBasedOnBindingBone(FXMVECTOR vPos,const Joint* pBindBone,const Geometrics::MetaBallModel* pMesh)
{
		bool result;
		XMVECTOR vtrJ = pBindBone->Entity[Default].Position;
		Vector3 SurfacePoint;
		if (pBindBone->Parent!=nullptr){
			XMVECTOR vtrB = pBindBone->Parent->Entity[Default].Position;
			vtrB = Projection(vPos,vtrJ,vtrB);
			vtrJ = vPos - vtrB;
			bool boneInseide = pMesh->Contains(vtrB);
			if (boneInseide)
				result = pMesh->RayIntersection(SurfacePoint,vtrB,vtrJ);
			else {
				vtrJ = pBindBone->Parent->Entity[Default].Position - vtrB;
				result = pMesh->RayIntersection(SurfacePoint,vtrB,vtrJ);
			}
		}
		else
		{
			XMVECTOR vtrP = vPos - vtrJ;
			result = pMesh->RayIntersection(SurfacePoint,vtrJ,vtrP);
		}
		if (!result)
			return g_XMQNaN;
		else
			return SurfacePoint;
}


// return the distance of this snap
float MetaballBasedWarpper::SnapToSurface(__inout HolographicPoint& HPoint) const 
{
	XMVECTOR point = HPoint.Position;
	XMVECTOR Sp;
	if (!m_pAvatar->Flesh->Contains(point))
		Sp = m_pAvatar->Flesh->FindClosestSurfacePoint(point);
	else
	{
		Joint* pBindBone = m_pAvatar->Skeleton->at(HPoint.Indices[0]);
		Sp = FindSurfacePointBasedOnBindingBone(point,pBindBone,m_pAvatar->Flesh);
	}

	if (XMVector3Equal(Sp,g_XMZero) || XMVector4Equal(Sp,g_XMQNaN))
		return g_INFINITY;

	float r = Vector3::Distance(Sp,point);
	if (m_pAvatar->Flesh->Contains(point))
		r = -r;

	HPoint.Position = Sp;
	m_pAvatar->Weighting(Sp,HPoint.Weights,HPoint.Indices);

	return r;
}


//DirectX::XMVECTOR NaiveWarpper::Warp_Bind_Closest_Bone_And_Inverse_Animation_Transform( DirectX::FXMVECTOR point , _Out_ Kinematics::Joint** ppBindedBone) const
//{
////	Kinematics::Joint* pABone = m_pAvatar->Skeleton->FindClosestJoint(point,Current);
//	Kinematics::Joint* pABone = m_pAvatar->Skeleton->FindClosestBone(point,Current);
//	if (ppBindedBone)
//		*ppBindedBone = pABone;
//	XMMATRIX matIvr = pABone->InverseBlendMatrix();
//	XMVECTOR vtrP = XMVector3TransformCoord(point,matIvr);
//
//	return vtrP;
//
//	//Vector3 SurfacePoint = Warp_Surface_Mapping(vtrP,pABone );
//	//return IWarpper::Warp_Global_To_Relative(SurfacePoint,pABone->Entity[Current].Position,pABone->Entity[Current].Orientation);
//
//}
//
//DirectX::XMVECTOR NaiveWarpper::Warp_Inverse_Animation_Transform( DirectX::FXMVECTOR point ,const XMUINT4 *BlendIndices /*= nullptr*/, const XMFLOAT4 *BlendWeights /*= nullptr*/ ) const
//{
//	unsigned int * Idx = (unsigned int*) BlendIndices;
//	float * Wts = (float*) BlendWeights;
//	XMMATRIX BlendMatrix = m_pAvatar->Skeleton->Index[Idx[0]]->BlendMatrix() * Wts[0];
//	for (unsigned int i = 1; i < 4; i++)
//	{
//		BlendMatrix += m_pAvatar->Skeleton->Index[Idx[i]]->BlendMatrix() * Wts[i];
//	}
//#ifdef _DEBUG
//	XMVECTOR determinant;
//	BlendMatrix = XMMatrixInverse(&determinant,BlendMatrix);
//	if (!XMVectorGetIntX(determinant))
//		throw exception("Inverse matrix don't exist");
//#else
//	BlendMatrix = XMMatrixInverse(nullptr,BlendMatrix);
//#endif // _DEBUG
//
//	return XMVector3TransformCoord(point,BlendMatrix);
//}
//
//DirectX::XMVECTOR NaiveWarpper::Warp_Animation_Transform( DirectX::FXMVECTOR point) const
//{
//	return m_pAvatar->SkinningSpacePoint(point);
//}
//
//DirectX::Vector3 NaiveWarpper::Warp_Snap_To_Surface( CXMVECTOR point , Kinematics::Joint* pBindBone) const
//{
//	Vector3 SurfacePoint = (Vector3)point;
//	bool result;
//	XMVECTOR vtrJ = pBindBone->Entity[Default].Position;
////		XMVECTOR vtrP = point - vtrJ;
//	if (pBindBone->Parent!=nullptr){
//		XMVECTOR vtrB = pBindBone->Parent->Entity[Default].Position;
//		vtrB = FindProjectionPointToSegment(point,vtrJ,vtrB);
//		vtrJ = point - vtrB;
//		result = m_pAvatar->Flesh->RayIntersection(SurfacePoint,vtrB,vtrJ);
//	}
//	else
//	{
//		XMVECTOR vtrP = point - vtrJ;
//		result = m_pAvatar->Flesh->RayIntersection(SurfacePoint,vtrJ,vtrP);
//	}
//
//	if (m_pAvatar->Flesh->Contains(point))
//		return SurfacePoint;
//
//	float r = Vector3::Distance(SurfacePoint,point);
//	if (r<0.2f) return SurfacePoint;
//	else return (Vector3)point;
//
//#ifdef _DEBUG
//		if (!result)
//			std::cout<<"Can't find intersection pint : "<<(Vector3)point<<" => "<<SurfacePoint<<std::endl;
//#endif // _DEBUG
//
//}
