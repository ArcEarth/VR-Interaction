#pragma once
#include "HumanSkeleton.h"
#include "DynamicMetaBallModel.h"
#include "SpaceWarpperInterface.h"


//class NaiveWarpper
//	: public IWarpper
//{
//public:
//	NaiveWarpper(Kinematics::HumanSkeleton* pHuman,Geometrics::DynamicMetaBallModel* pAvatar);
//	~NaiveWarpper(void);
//
//	virtual DirectX::XMVECTOR Warp_Animation_Transform( DirectX::FXMVECTOR point) const;
//	virtual DirectX::XMVECTOR Warp_Bind_Closest_Bone_And_Inverse_Animation_Transform( DirectX::FXMVECTOR point , _Out_ Kinematics::Joint** ppBindedBone) const;
//	virtual DirectX::XMVECTOR Warp_Inverse_Animation_Transform( DirectX::FXMVECTOR point ,const DirectX::XMUINT4 *BlendIndices = nullptr, const DirectX::XMFLOAT4 *BlendWeights = nullptr) const;
//	virtual DirectX::Vector3 Warp_Snap_To_Surface(DirectX::CXMVECTOR point , Kinematics::Joint* pBindBone ) const;
//
//private:
//	Kinematics::HumanSkeleton* m_pHuman;
//	Geometrics::DynamicMetaBallModel* m_pAvatar;
//};

class NaiveWarpper
	: public IWarpper
{
public:
	NaiveWarpper(Kinematics::HumanSkeleton* pHuman,Geometrics::DynamicMetaBallModel* pAvatar);
	virtual ~NaiveWarpper();

	virtual Kinematics::IndexedSkeleton& Dictionary() const;

	virtual HolographicPoint Inverse_Animation_Transform(DirectX::FXMVECTOR Point , unsigned int Max_Binding_Count) const;
	virtual DirectX::XMVECTOR Animation_Transform(const HolographicPoint& Hpoint) const;

	// return the distance of this snap
	virtual float SnapToSurface(__inout HolographicPoint& HPoint) const;

private:
	const Kinematics::HumanSkeleton* m_pHuman;
	const Geometrics::DynamicMetaBallModel* m_pAvatar;
};

class MetaballBasedWarpper
	: public IWarpper
{
public:
	MetaballBasedWarpper(Kinematics::HumanSkeleton* pHuman,Geometrics::DynamicMetaBallModel* pAvatar);
	virtual ~MetaballBasedWarpper();

	virtual Kinematics::IndexedSkeleton& Dictionary() const;

	virtual HolographicPoint Inverse_Animation_Transform(DirectX::FXMVECTOR Point , unsigned int Max_Binding_Count) const;
	virtual DirectX::XMVECTOR Animation_Transform(const HolographicPoint& Hpoint) const;

	// return the distance of this snap
	virtual float SnapToSurface(__inout HolographicPoint& HPoint) const;

private:
	const Kinematics::HumanSkeleton* m_pHuman;
	const Geometrics::DynamicMetaBallModel* m_pAvatar;
};
