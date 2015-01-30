#pragma once
#include <D3Dx10math.h>
#include "DXMathExtend.h"
#include <fstream>
#include <vector>
#include <deque>
#include <set>
#include <NuiApi.h>


//static const int jointReadArray[] = {
//	0, 1, 1, 2, 2, 3,   //the body
//	2, 8, 8, 9, 9, 11,  //the right arm
//	2, 4, 4, 5, 5, 7,  //the left arm
//	0, 16, 16, 17, 17, 19,  //the right leg
//	0, 12, 12, 13, 13, 15   //the left leg
//};	// TODO: why do this? Also, wrists and ankles are not used.
//class Bone : std::pair<int,int>
//{
//public:
//	Bone();
//	~Bone();
//
//	D3DXQUATERNION Orientation;
//};
//
//Bone::Bone()
//{
//}
//
//Bone::~Bone()
//{
//}

//const std::vector<SkeletonClass::HumanBoneEnum> BonesInLeftArm = {SkeletonClass::Humerus_Left,SkeletonClass::Ulna_Left};

class SkeletonClass
{
public:
	enum HumanJointEnum
	{
		HIP_CENTER = 0,
		SPINE_CENTER,
		SHOULDER_CENTER,
		HEAD,
		SHOULDER_LEFT,
		ELBOW_LEFT,
		WRIST_LEFT,
		HAND_LEFT,
		SHOULDER_RIGHT,
		ELBOW_RIGHT,
		WRIST_RIGHT,
		HAND_RIGHT,
		HIP_LEFT,
		KNEE_LEFT,
		ANKLE_LEFT,
		FOOT_LEFT,
		HIP_RIGHT,
		KNEE_RIGHT,
		ANKLE_RIGHT,
		FOOT_RIGHT,
	};
	enum HumanBoneEnum
	{
		Illegal = -1,
		LumbarVertebra = 0,
		ThoracicVertebra,
		CervicalVertebra,
		Clavicle_Right,
		Humerus_Right,
		Radius_Right, // Forgot about the wrist
		Clavicle_Left,
		Humerus_Left,
		Radius_Left,
		Pelvic_Right,
		Femur_Right,
		Tibia_Right, // Forgot about the ankle
		Pelvic_Left,
		Femur_Left,
		Tibia_Left,
	};

public:
	bool TransformFlag; 
	std::vector<int> m_LeftArmIndecis,m_RightArmIndecis;
	bool Available;
public:
	SkeletonClass(void);
	~SkeletonClass(void);

	bool Initialize(unsigned JointsCount,unsigned BonesCount,int * BonesDescription);

	bool SetJointsDefaultPosition();
	//This is a cripy function....
	bool BuildDeaulftSkeleton();
	void ClearDefaultSkeleton();

	bool Update(const NUI_SKELETON_DATA &SkeletonData);

	//Get the joint position's interface
	const D3DXVECTOR3 GetJointPosition(HumanJointEnum Joint,bool IsAbsulote);

	//Find the binding bone for a point with a specific forbidon bone list
	HumanBoneEnum FindBindingBone(const D3DXVECTOR3 &p,const std::set<HumanBoneEnum> &ExceptionList);
	//This function will translate a point relate to hip-center to a coordinate data relate to the given bone's root joint
	bool InverseTransformPointBindToBone(D3DXVECTOR3 &Point, HumanBoneEnum BindingBone);

	//Return the length of bone in default skeleton
	float GetBoneLength(HumanBoneEnum bone);
	D3DXVECTOR3 GetBoneVector(HumanBoneEnum bone) const;

	D3DXMATRIX GetBindingBoneTransformMatrix(HumanBoneEnum BindingBone);

	//change the transform factor
	bool MakeArmLonger(float delta = 0.1f);
	bool MakeArmShorter(float delta = 0.1f);
	bool SetArmTransformScale(float scale);
	float GetArmLength();
	float GetArmLength(bool IsLeft);
	bool TurnArmTransform(bool Switch);

	//Interface for geometry to build animation weight
	const D3DXVECTOR4 *GetSkeletonDefaultPosition();

	//Interface for graphic's animation
	const D3DXVECTOR4 *GetSkeletonDisplacement();
	const D3DXVECTOR3 GetBodyCentreDisplaceMent();
	const D3DXVECTOR3 GetPosition();
	const D3DXMATRIX &GetFacingMatrix();
	const float &GetFacingAngle();
	const unsigned &GetJointsCount();
	const unsigned &GetBonesCount();
	const unsigned &size();
	const bool IsDefaultSkeletonAvailable() const;

	std::vector<D3DXVECTOR3> Joints;
	// Skeleton orientations
	std::vector<std::pair<int,int>> Bones;
//	const Joints& = m_JointsCurrentPosition;

private:
	//Signal for mul-thread sync
	volatile bool m_IsUpdating;
	D3DXVECTOR3 m_Position;
	D3DXMATRIX m_FacingMatrix;
	float m_FacingAngle;
	D3DXVECTOR4 *m_JointsDefaultPosition;
//	D3DXVECTOR4 *m_JointsCurrentPosition;
	D3DXVECTOR4 *m_JointsDisplacement;
	unsigned int m_JointsCount,m_BonesCount;
	float m_TransformScale,m_TransformCutoff;
	std::deque<D3DXVECTOR3> m_SkeletonCentreHistory;
	bool m_DefaultAvailable;

	bool TransformJoint(D3DXVECTOR3 &TransformPoint,const D3DXVECTOR3 &AnchorPoint);
	bool TransformJoint(D3DXVECTOR3 &TransformPoint,const D3DXVECTOR3 &AnchorPoint,const float Factor ,bool Isleft);
};

