#pragma once;
#include "KinematicsSkeleton.h"


struct HolographicPoint
{
	DirectX::Vector3 Position;
	uint32_t Indices[4];
	float Weights[4];

	HolographicPoint()
	{
	}

	HolographicPoint(const DirectX::Vector3& _Pos,const Kinematics::Joint* _pBindBone)
	{
		this->Position = _Pos;
		Weights[0] = 1.0f;
		Indices[0] = _pBindBone->ID;
	}

	void clear()
	{
		memset(Indices,0,sizeof(uint32_t)*4);
		memset(Weights,0,sizeof(float)*4);
	}

};

class IWarpper
{
public:

	virtual ~IWarpper() {}
	virtual Kinematics::IndexedSkeleton& Dictionary() const = 0;
	// Turn on Rigid Binding will get the Holograhic Point only have one blend indices
	virtual HolographicPoint Inverse_Animation_Transform(DirectX::FXMVECTOR Point , unsigned int Max_Binding_Count) const = 0;
	virtual DirectX::XMVECTOR Animation_Transform(const HolographicPoint& Hpoint) const = 0;
	// return the distance of this snap
	virtual float SnapToSurface(__inout HolographicPoint& HPoint) const = 0;

	DirectX::XMVECTOR SnapToSurface(DirectX::CXMVECTOR Point , Kinematics::Joint* pBindBone) const
	{
		HolographicPoint Hp((DirectX::Vector3)Point,pBindBone);
		SnapToSurface(Hp);
		return Hp.Position;
	}

	HolographicPoint Cursor_Transform(DirectX::FXMVECTOR Point ,__out bool *Snaped = nullptr , float SnapedRadius = 0.15f) const
	{
		auto Hp = Inverse_Animation_Transform(Point,1);
		auto Sp = Hp;
		if (Hp.Weights[0] == 0.0f)
		{
			Snaped = false;
			return Hp;
		}
		float r = SnapToSurface(Sp);
		bool snap = r<SnapedRadius;
		if (Snaped)
			*Snaped = snap;
		if (snap) 
			return Sp;
		else 
			return Hp;
	}
};

//
//class IWarpper
//{
//public:
//	// This is the critical mapping method that allowed user to easy access the surface of avatar.
//	// Transform the point in Avatar's Default space into a surface point in Avatar's default space
//	// "Snap to the surface!"
//	// Anyway , this method should need a parameter represent the model
//	// Ps.since it need the model info , it's DEFAULT Space only
//	//virtual XMVECTOR Warp_Snap_To_Surface(DirectX::CXMVECTOR point , Kinematics::Joint* pBindBone) const = 0;
//	//// inverse skinning a point in space
//	//// Test Case : Warp(Animation_Transform(Warp_Inverse_Animation_Transform(p,null)) == p
//	//virtual DirectX::XMVECTOR Warp_Bind_Closest_Bone_And_Inverse_Animation_Transform(DirectX::FXMVECTOR , _Out_ Kinematics::Joint** ppBindedBone) const = 0;
//	//// Aka : skinning this point to animated!
//	//virtual DirectX::XMVECTOR Warp_Animation_Transform(DirectX::FXMVECTOR) const = 0;
//
//	inline static DirectX::XMVECTOR Warp_Global_To_Relative(DirectX::FXMVECTOR Point ,DirectX::FXMVECTOR Origin,DirectX::FXMVECTOR Orientation){
//		return DirectX::XMVector3InverseRotate(DirectX::XMVectorSubtract(Point,Origin) , Orientation);
//	}
//
//	inline static DirectX::XMVECTOR Warp_Global_To_Relative(DirectX::FXMVECTOR Point ,Kinematics::Joint* pTargetJoint , Kinematics::State state){
//		return DirectX::XMVector3InverseRotate(DirectX::XMVectorSubtract(Point,pTargetJoint->Entity[state].Position) , pTargetJoint->Entity[state].Orientation);
//	}
//
//	inline static DirectX::XMVECTOR Warp_Relative_To_Global(DirectX::FXMVECTOR Point ,DirectX::FXMVECTOR Origin,DirectX::FXMVECTOR Orientation){
//		return DirectX::XMVectorAdd(DirectX::XMVector3Rotate(Point, Orientation),Origin);
//	}
//
//	inline static DirectX::XMVECTOR Warp_Relative_To_Global(DirectX::FXMVECTOR Point ,Kinematics::Joint* pTargetJoint , Kinematics::State state){
//		return DirectX::XMVectorAdd(DirectX::XMVector3Rotate(Point, pTargetJoint->Entity[state].Orientation),pTargetJoint->Entity[state].Position);
//	}
////	virtual DirectX::Vector3 WarpBySpecificCoordinate();
//};
