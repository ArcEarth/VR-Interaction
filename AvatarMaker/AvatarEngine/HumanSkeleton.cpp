#include "stdafx.h"
#include "HumanSkeleton.h"
#include <DirectXColors.h>
#include <DirectXCollision.h>
#include <array>

using namespace Kinematics;
using namespace DirectX::Colors;
using namespace DirectX;
using namespace std;

const static int HumanBoneConnection[] = {
	0, 1, 1, 2, 2, 3,   //the body
	2, 4, 4, 5, 5, 6, 6, 7,  //the left arm
	2, 8, 8, 9, 9, 10 ,10 ,11,  //the right arm
	0, 12, 12, 13, 13, 14, 14, 15,   //the left leg
	0, 16, 16, 17, 17, 18, 18, 19,  //the right leg
};

using DirectX::XMFLOAT4A;

const XMFLOAT4A HumanSkeleton::HumanBoneColorSchedule[HumanSkeleton::JointCount] = 
{
	XMFLOAT4A(Red), // HIP_CENTER
	XMFLOAT4A(Orange), // SPINE_CENTER
	XMFLOAT4A(Yellow), // SHOULDER_CENTER
	XMFLOAT4A(GreenYellow), // HEAD
	XMFLOAT4A(LawnGreen), // SHOULDER_LEFT
	XMFLOAT4A(SkyBlue), // ELBOW_LEFT
	XMFLOAT4A(DeepSkyBlue), // WRIST_LEFT
	XMFLOAT4A(Orchid), // HAND_LEFT
	XMFLOAT4A(LawnGreen), // SHOULDER_RIGHT
	XMFLOAT4A(SkyBlue), // ELBOW_RIGHT
	XMFLOAT4A(DeepSkyBlue), // WRIST_RIGHT
	XMFLOAT4A(Orchid), // HAND_RIGHT
	XMFLOAT4A(Orange), // HIP_LEFT
	XMFLOAT4A(Yellow), // KNEE_LEFT
	XMFLOAT4A(LawnGreen), // ANKLE_LEFT
	XMFLOAT4A(SkyBlue), // FOOT_LEFT
	XMFLOAT4A(Orange), // HIP_RIGHT
	XMFLOAT4A(Yellow), // KNEE_RIGHT
	XMFLOAT4A(LawnGreen), // ANKLE_RIGHT
	XMFLOAT4A(SkyBlue), // FOOT_RIGHT
};

HumanSkeleton::HumanSkeleton()
	: IndexedSkeleton(20,HumanBoneConnection)
{
	ScaleArmReachScaleFactor = 1.0f;
	FleshRadius = 0.0f;
	ScaleArmReach = false;
	this->for_all([&](Joint* Itr)
	{
		Itr->Radius = 0.05f;
	});
	for (int i = 0; i < 20; i++)
	{
		BoneLengthes[i].first = 0;
		BoneLengthes[i].second = 0.0f;
	}
}

HumanSkeleton::~HumanSkeleton(void)
{
}

DirectX::Vector3 HumanSkeleton::GetActualJoint(unsigned int jointIndex) const{
	return (*this)[jointIndex]->Entity[Current].Position;
}

std::auto_ptr<DirectX::BoundingBox> HumanSkeleton::GetBoundingBox() const
{
	auto_ptr<DirectX::BoundingBox> pBox(new DirectX::BoundingBox);
	std::array<Vector3,20> JointPositions;
	for (auto& idx : this->Index)
	{
		JointPositions[idx.first] = idx.second->Entity[Current].Position;
	}

	BoundingBox::CreateFromPoints(*pBox,this->JointCount,JointPositions.data(),sizeof(Vector3));
	return pBox;
}

float HumanSkeleton::Height() const
{
	return GetActualJoint(HEAD).y - min(GetActualJoint(FOOT_LEFT).y,GetActualJoint(FOOT_RIGHT).y) + FleshRadius * 2;
}

void HumanSkeleton::Update(const DirectX::Vector3 JointPosistions[])
{
//	Root->Entity[Current].Rotation = BoneOrientations[0];
	Root->Entity[Current].Scale.Set(1.0f,JointPosistions[0].Length(),1.0f);
//	Root->Entity[Current].Orientation = BoneOrientations[0];
	Root->Entity[Current].Position = JointPosistions[0];
	this->Root->for_all_descendants([&](Joint* pJoint)
	{
//		pJoint->Entity[Current].Rotation = BoneOrientations[pJoint->ID];
		if (BoneLengthes[pJoint->ID].first < 1000) {
			BoneLengthes[pJoint->ID].second += (JointPosistions[pJoint->ID]-JointPosistions[pJoint->Parent->ID]).Length();
			BoneLengthes[pJoint->ID].first ++ ;
		}
		pJoint->Entity[Current].Scale.Set(1.0f,BoneLengthes[pJoint->ID].second / BoneLengthes[pJoint->ID].first,1.0f);
	});

	IndexedSkeleton::Update();

	ScaleArm();
}


/// <summary>
/// Updates the specified joint positions.
/// </summary>
/// <param name="JointPosistions">The joint positions.</param>
/// <param name="BoneOrientations">The bone orientations.</param>
void HumanSkeleton::Update(const DirectX::Vector3 JointPosistions[JointCount],const DirectX::Quaternion BoneOrientations[BoneCount]){
	//std::vector<DirectX::Quaternion> Orientations(BoneOrientations,BoneOrientations+BoneCount);
	Root->Entity[Current].Rotation = BoneOrientations[0];
	Root->Entity[Current].Scale.Set(1.0f,JointPosistions[0].Length(),1.0f);
	Root->Entity[Current].Orientation = BoneOrientations[0];
	Root->Entity[Current].Position = JointPosistions[0];
	this->Root->for_all_descendants([&](Joint* pJoint)
	{
		pJoint->Entity[Current].Rotation = BoneOrientations[pJoint->ID];
		if (BoneLengthes[pJoint->ID].first < 1000) {
			BoneLengthes[pJoint->ID].second += (JointPosistions[pJoint->ID]-JointPosistions[pJoint->Parent->ID]).Length();
			BoneLengthes[pJoint->ID].first ++ ;
		}
		pJoint->Entity[Current].Scale.Set(1.0f,BoneLengthes[pJoint->ID].second / BoneLengthes[pJoint->ID].first,1.0f);
	});

	IndexedSkeleton::Update();

	const static HumanJointEnum Limbs[] = {ELBOW_LEFT,WRIST_LEFT,HAND_LEFT,ELBOW_RIGHT,WRIST_RIGHT,HAND_RIGHT,KNEE_LEFT,ANKLE_LEFT,FOOT_LEFT,KNEE_RIGHT,ANKLE_RIGHT,FOOT_RIGHT};

	for (auto& idx : Limbs)
	{
		Index.at(idx)->Deduce_Hierarchical_from_Global(Current,nullptr);
	}

	//IndexedSkeleton::Update();
	ScaleArm();
}

void HumanSkeleton::ScaleArm()
{
	static const float ScaleCutOff = 0.8f;
	if (ScaleArmReach)
	{
		for (unsigned int arm = 0 ; arm < 2 ; arm++)
		{
			unsigned int Metacarpals = Metacarpals_Left + arm*4;
			unsigned int Radius = Radius_Left + arm*4;
			unsigned int Humerus = Humerus_Left + arm*4;

			float ArmLength = BoneLengthes[Metacarpals].second / BoneLengthes[Metacarpals].first
							+ BoneLengthes[Radius].second / BoneLengthes[Radius].first
							+ BoneLengthes[Humerus].second / BoneLengthes[Humerus].first;

			float factor = (GetActualJoint(HAND_LEFT + arm*4) - GetActualJoint(SHOULDER_LEFT + arm*4)).Length();
			factor = factor/ArmLength - ScaleCutOff;

			if (factor > 0.0f)
			{
				float A = max(ScaleArmReachScaleFactor / ArmLength - 1.0f , 0.0f);
				float F = 1.0f + A*factor*factor*16.0f;
				(*this)[ELBOW_LEFT + arm*4]->Entity[Current].Scale.y *= F;
				(*this)[WRIST_LEFT + arm*4]->Entity[Current].Scale.y *= F;
				(*this)[ HAND_LEFT + arm*4]->Entity[Current].Scale.y *= F;
			}
		}
	}

	IndexedSkeleton::Update();
}
